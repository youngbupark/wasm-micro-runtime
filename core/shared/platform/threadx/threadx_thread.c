#include "tx_api.h"
#include "platform_api_vmcore.h"
#include "platform_api_extension.h"

/* clang-format off */
#define bh_assert(v) do {                                   \
    if (!(v)) {                                             \
        printf("\nASSERTION FAILED: %s, at %s, line %d\n",  \
               #v, __FILE__, __LINE__);                     \
        abort();                                            \
    }                                                       \
} while (0)

#define TX_FAILED(s) ((s) != TX_SUCCESS)

/* clang-format on */

typedef struct os_thread_wait_node {
    korp_sem sem;
    os_thread_wait_list next;
} os_thread_wait_node;

typedef struct os_thread_data {
    /* Next thread data */
    struct os_thread_data *next;

    /* Thread handle */
    korp_tid tid;
    /* Thread entry function */
    thread_start_routine_t start;
    /* Entry function arguments */
    void *arg;

    /* Thread local root */
    void *tlr;
    /* Lock for waiting list */
    korp_mutex wait_list_lock;
    /* Waiting list of other threads who are joining this thread */
    os_thread_wait_list thread_wait_list;
    /* Thread stack size */
    unsigned stack_size;

    /* Thread stack */
    char stack[1];
} os_thread_data;

typedef struct os_thread_obj {
    korp_thread thread;
    /* Whether the thread is terminated and this thread object is to
     be freed in the future. */
    bool to_be_freed;
    struct os_thread_obj *next;
} os_thread_obj;

static bool is_thread_sys_inited = false;

/* Thread data of supervisor thread */
static os_thread_data supervisor_thread_data;

/* Lock for thread data list */
static korp_mutex thread_data_lock;

/* Thread data list */
static os_thread_data *thread_data_list = NULL;

/* Lock for thread object list */
static korp_mutex thread_obj_lock;

/* Thread object list */
static os_thread_obj *thread_obj_list = NULL;

static void
thread_data_list_add(os_thread_data *thread_data)
{
    os_mutex_lock(&thread_data_lock);

    if (!thread_data_list)
        thread_data_list = thread_data;
    else {
        /* If already in list, just return */
        os_thread_data *p = thread_data_list;
        while (p) {
            if (p == thread_data) goto done;

            p = p->next;
        }

        /* Set as head of list */
        thread_data->next = thread_data_list;
        thread_data_list = thread_data;
    }

done:
    os_mutex_unlock(&thread_data_lock);
}

static void
thread_data_list_remove(os_thread_data *thread_data)
{
    os_mutex_lock(&thread_data_lock);
    if (thread_data_list) {
        if (thread_data_list == thread_data)
            thread_data_list = thread_data_list->next;
        else {
            /* Search and remove it from list */
            os_thread_data *p = thread_data_list;
            while (p && p->next != thread_data)
                p = p->next;
            if (p && p->next == thread_data)
                p->next = p->next->next;
        }
    }
    os_mutex_unlock(&thread_data_lock);
}

static os_thread_data *
thread_data_list_lookup(korp_tid tid)
{
    os_mutex_lock(&thread_data_lock);
    if (thread_data_list) {
        os_thread_data *p = thread_data_list;
        while (p) {
            if (p->tid == tid) {
                /* Found */
                os_mutex_unlock(&thread_data_lock);
                return p;
            }
            p = p->next;
        }
    }

    os_mutex_unlock(&thread_data_lock);
    return NULL;
}

static void
thread_obj_list_add(os_thread_obj *thread_obj)
{
    os_mutex_lock(&thread_obj_lock);
    if (!thread_obj_list)
        thread_obj_list = thread_obj;
    else {
        /* Set as head of list */
        thread_obj->next = thread_obj_list;
        thread_obj_list = thread_obj;
    }
    os_mutex_unlock(&thread_obj_lock);
}

static void
thread_obj_list_reclaim()
{
    os_thread_obj *p, *p_prev;
    os_mutex_lock(&thread_obj_lock);
    p_prev = NULL;
    p = thread_obj_list;
    while (p) {
        if (p->to_be_freed) {
            if (p_prev == NULL) { /* p is the head of list */
                thread_obj_list = p->next;
                BH_FREE(p);
                p = thread_obj_list;
            }
            else { /* p is not the head of list */
                p_prev->next = p->next;
                BH_FREE(p);
                p = p_prev->next;
            }
        }
        else {
            p_prev = p;
            p = p->next;
        }
    }
    os_mutex_unlock(&thread_obj_lock);
}

int
os_thread_sys_init()
{
    if (is_thread_sys_inited)
        return BHT_OK;

    if (TX_FAILED(tx_mutex_create(&thread_data_lock, "mutex-thread-data", TX_NO_INHERIT))) {
        goto fail;
    }

    if (TX_FAILED(tx_mutex_create(&thread_obj_lock, "mutex-thread-obj", TX_NO_INHERIT))) {
        goto fail;
    }

    /* Initialize supervisor thread data */
    memset(&supervisor_thread_data, 0, sizeof(supervisor_thread_data));

    supervisor_thread_data.tid = tx_thread_identify();
    /* Set as head of thread data list */
    thread_data_list = &supervisor_thread_data;

    is_thread_sys_inited = true;
    return BHT_OK;

fail:
    return BHT_ERROR;
}

void
os_thread_sys_destroy(void)
{
    if (is_thread_sys_inited) {
        is_thread_sys_inited = false;
    }
}

static os_thread_data *
thread_data_current()
{
    korp_tid tid = tx_thread_identify();
    return thread_data_list_lookup(tid);
}

static void
os_thread_cleanup(void)
{
    os_thread_data *thread_data = thread_data_current();
    bh_assert(thread_data != NULL);

    os_mutex_lock(&thread_data->wait_list_lock);
    if (thread_data->thread_wait_list) {
        /* Signal each joining thread */
        os_thread_wait_list head = thread_data->thread_wait_list;
        while (head) {
            os_thread_wait_list next = head->next;
            tx_semaphore_put(&head->sem);
            /* head will be freed by joining thread */
            head = next;
        }
        thread_data->thread_wait_list = NULL;
    }
    os_mutex_unlock(&thread_data->wait_list_lock);

    thread_data_list_remove(thread_data);
    /* Set flag to true for the next thread creating to
     free the thread object */
    ((os_thread_obj *)thread_data->tid)->to_be_freed = true;
    BH_FREE(thread_data);
}

static void
os_thread_wrapper(ULONG entry_input)
{
    os_thread_data *t_data = (VOID *)entry_input;

    /* Set thread custom data */
    t_data->tid = tx_thread_identify();
    thread_data_list_add(t_data);

    ((thread_start_routine_t)t_data->start)(t_data->arg);
    os_thread_cleanup();
}

int
os_thread_create(korp_tid *p_tid, thread_start_routine_t start, void *arg,
                 unsigned int stack_size)
{
    return os_thread_create_with_prio(p_tid, start, arg, stack_size,
                                      BH_THREAD_DEFAULT_PRIORITY);
}

int
os_thread_create_with_prio(korp_tid *p_tid, thread_start_routine_t start,
                           void *arg, unsigned int stack_size, int prio)
{
    korp_tid tid;
    os_thread_data *thread_data;
    unsigned thread_data_size;

    if (!p_tid || !stack_size)
        return BHT_ERROR;

    /* Free the thread objects of terminated threads */
    thread_obj_list_reclaim();

    /* Create and initialize thread object */
    if (!(tid = BH_MALLOC(sizeof(os_thread_obj))))
        return BHT_ERROR;

    memset(tid, 0, sizeof(os_thread_obj));

    /* Create and initialize thread data */
    if (stack_size < APP_THREAD_STACK_SIZE_MIN)
        stack_size = APP_THREAD_STACK_SIZE_MIN;
    thread_data_size = offsetof(os_thread_data, stack) + stack_size;

    if (!(thread_data = BH_MALLOC(thread_data_size))) {
        goto fail_thread_data_alloc;
    }

    memset(thread_data, 0, thread_data_size);

    if (TX_FAILED(tx_mutex_create(&thread_data->wait_list_lock, "wait_list_lock", TX_NO_INHERIT))) {
        goto fail;
    }

    thread_data->stack_size = stack_size;
    thread_data->tid = tid;
    thread_data->start = start;
    thread_data->arg = arg;

    /* Create the thread */
    if (TX_FAILED((tx_thread_create(tid, "thread", 
        os_thread_wrapper, (ULONG)(VOID *)thread_data,
        thread_data->stack, stack_size,
        prio, 0, TX_NO_TIME_SLICE, TX_AUTO_START)))) {
        goto fail;
    }

    bh_assert(tid == thread_data->tid);

    /* Set thread custom data */
    thread_data_list_add(thread_data);
    thread_obj_list_add((os_thread_obj *)tid);
    *p_tid = tid;
    return BHT_OK;

fail:
    BH_FREE(thread_data);

fail_thread_data_alloc:
    BH_FREE(tid);
    return BHT_ERROR;
}

korp_tid
os_self_thread()
{
    return (korp_tid)tx_thread_identify();
}

int
os_thread_join(korp_tid thread, void **value_ptr)
{
    (void)value_ptr;
    os_thread_data *thread_data;
    os_thread_wait_node *node;

    /* Create wait node and append it to wait list */
    if (!(node = BH_MALLOC(sizeof(os_thread_wait_node))))
        return BHT_ERROR;

    if (TX_FAILED(tx_semaphore_create(&node->sem, "thread_join", 0))) {
        goto fail;
    }

    node->next = NULL;

    /* Get thread data */
    thread_data = thread_data_list_lookup(thread);
    bh_assert(thread_data != NULL);

    os_mutex_lock(&thread_data->wait_list_lock);
    if (!thread_data->thread_wait_list)
        thread_data->thread_wait_list = node;
    else {
        /* Add to end of waiting list */
        os_thread_wait_node *p = thread_data->thread_wait_list;

        while (p->next) p = p->next;
        p->next = node;
    }
    os_mutex_unlock(&thread_data->wait_list_lock);

    /* Wait the sem */
    tx_semaphore_get(&node->sem, TX_WAIT_FOREVER);

    /* Wait some time for the thread to be actually terminated */
    tx_thread_sleep(TX_MS_TO_TICKS(100));

fail:
    /* Destroy resource */
    BH_FREE(node);

    return BHT_OK;
}

int
os_mutex_init(korp_mutex *mutex)
{
    bh_assert(mutex);
    return TX_FAILED(tx_mutex_create(mutex, "os_mutex", TX_NO_INHERIT)) ? BHT_ERROR : BHT_OK;
}

int
os_recursive_mutex_init(korp_mutex *mutex)
{
    bh_assert(mutex);
    return TX_FAILED(tx_mutex_create(mutex, "os_recursive_mutex", TX_NO_INHERIT)) ? BHT_ERROR : BHT_OK;
}

int
os_mutex_destroy(korp_mutex *mutex)
{
    return TX_FAILED(tx_mutex_delete(mutex)) ? BHT_ERROR : BHT_OK;
}

int
os_mutex_lock(korp_mutex *mutex)
{
    return TX_FAILED(tx_mutex_get(mutex, TX_WAIT_FOREVER)) ? BHT_ERROR : BHT_OK;
}

int
os_mutex_unlock(korp_mutex *mutex)
{
    bh_assert(mutex);
    return TX_FAILED(tx_mutex_put(mutex)) ? BHT_ERROR : BHT_OK;
}

int
os_cond_init(korp_cond *cond)
{
    if (os_mutex_init(&cond->wait_list_lock) != BHT_OK)
        return BHT_ERROR;

    cond->thread_wait_list = NULL;
    return BHT_OK;
}

int
os_cond_destroy(korp_cond *cond)
{
    bh_assert(cond);
    os_mutex_destroy(&cond->thread_wait_list);
    return BHT_OK;
}

static int
os_cond_wait_internal(korp_cond *cond, korp_mutex *mutex, bool timed, int mills)
{
    os_thread_wait_node *node;

    /* Create wait node and append it to wait list */
    if (!(node = BH_MALLOC(sizeof(os_thread_wait_node))))
        return BHT_ERROR;

    if (TX_FAILED(tx_semaphore_create(&node->sem, "cond_wait", 0))) {
        goto fail;
    }
    node->next = NULL;

    os_mutex_lock(&cond->wait_list_lock);
    if (!cond->thread_wait_list)
        cond->thread_wait_list = node;
    else {
        /* Add to end of wait list */
        os_thread_wait_node *p = cond->thread_wait_list;
        while (p->next)
            p = p->next;
        p->next = node;
    }
    os_mutex_unlock(&cond->wait_list_lock);

    /* Unlock mutex, wait sem and lock mutex again */
    os_mutex_unlock(mutex);
    tx_semaphore_get(&node->sem, timed ? TX_MS_TO_TICKS(mills) : TX_WAIT_FOREVER);
    os_mutex_lock(mutex);

    /* Remove wait node from wait list */
    os_mutex_lock(&cond->wait_list_lock);
    if (cond->thread_wait_list == node) {
        cond->thread_wait_list = node->next;
    } else {
        /* Remove from the wait list */
        os_thread_wait_node *p = cond->thread_wait_list;
        while (p->next != node)
            p = p->next;
        p->next = node->next;
    }
    BH_FREE(node);
    os_mutex_unlock(&cond->wait_list_lock);

    return BHT_OK;

fail:
    return BHT_ERROR;
}

int
os_cond_wait(korp_cond *cond, korp_mutex *mutex)
{
    return os_cond_wait_internal(cond, mutex, false, 0);
}

int
os_cond_reltimedwait(korp_cond *cond, korp_mutex *mutex, uint64 useconds)
{
    if (useconds == BHT_WAIT_FOREVER) {
        return os_cond_wait_internal(cond, mutex, false, 0);
    }
    else {
        uint64 mills_64 = useconds / 1000;
        int32 mills;

        if (mills_64 < (uint64)INT32_MAX) {
            mills = (int32)mills_64;
        }
        else {
            mills = INT32_MAX;
            os_printf("Warning: os_cond_reltimedwait exceeds limit, "
                      "set to max timeout instead\n");
        }
        return os_cond_wait_internal(cond, mutex, true, mills);
    }
}

int
os_cond_signal(korp_cond *cond)
{
    /* Signal the head wait node of wait list */
    os_mutex_lock(&cond->wait_list_lock);
    if (cond->thread_wait_list)
        tx_semaphore_put(&cond->thread_wait_list->sem);
    os_mutex_unlock(&cond->wait_list_lock);

    return BHT_OK;
}

int
os_cond_broadcast(korp_cond *cond)
{
    /* Signal all of the wait node of wait list */
    os_mutex_lock(&cond->wait_list_lock);
    if (cond->thread_wait_list) {
        os_thread_wait_node *p = cond->thread_wait_list;
        while (p) {
            tx_semaphore_put(&p->sem);
            p = p->next;
        }
    }

    os_mutex_unlock(&cond->wait_list_lock);
    return BHT_OK;
}

uint8 *
os_thread_get_stack_boundary()
{
    return (uint8 *)tx_thread_identify()->tx_thread_stack_end;
}

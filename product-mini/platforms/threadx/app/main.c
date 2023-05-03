#include <stdlib.h>
#include <string.h>
#include "tx_api.h"
#include "bh_platform.h"
#include "bh_assert.h"
#include "bh_log.h"
#include "wasm_export.h"
#include "board_init.h"
#include "test_wasm.h"

#define CONFIG_GLOBAL_HEAP_BUF_SIZE   WASM_GLOBAL_HEAP_SIZE
#define CONFIG_APP_STACK_SIZE         8192
#define CONFIG_APP_HEAP_SIZE          8192
#define CONFIG_MAIN_THREAD_STACK_SIZE 4096

#define MAIN_THREAD_STACK_SIZE        (CONFIG_MAIN_THREAD_STACK_SIZE)
#define MAIN_THREAD_PRIORITY          5

static int app_argc;
static char **app_argv;

static TX_THREAD iwasm_main_thread;

VOID board_setup(void);

/**
 * Find the unique main function from a WASM module instance
 * and execute that function.
 *
 * @param module_inst the WASM module instance
 * @param argc the number of arguments
 * @param argv the arguments array
 *
 * @return true if the main function is called, false otherwise.
 */

static void *
app_instance_main(wasm_module_inst_t module_inst)
{
    const char *exception;
    wasm_function_inst_t func;
    wasm_exec_env_t exec_env;
    unsigned argv[2] = { 0 };

    if (wasm_runtime_lookup_function(module_inst, "main", NULL)
        || wasm_runtime_lookup_function(module_inst, "__main_argc_argv",
                                        NULL)) {
        LOG_VERBOSE("Calling main funciton\n");
        wasm_application_execute_main(module_inst, app_argc, app_argv);
    }
    else if ((func = wasm_runtime_lookup_function(module_inst, "app_main",
                                                  NULL))) {
        exec_env =
            wasm_runtime_create_exec_env(module_inst, CONFIG_APP_HEAP_SIZE);
        if (!exec_env) {
            os_printf("Create exec env failed\n");
            return NULL;
        }

        LOG_VERBOSE("Calling app_main funciton\n");
        wasm_runtime_call_wasm(exec_env, func, 0, (uint32_t*)argv);

        if (!wasm_runtime_get_exception(module_inst)) {
            os_printf("result: 0x%x\n", argv[0]);
        }

        wasm_runtime_destroy_exec_env(exec_env);
    }
    else
    {
        os_printf("Failed to lookup function main or app_main to call\n");
        return NULL;
    }

    if ((exception = wasm_runtime_get_exception(module_inst)))
        os_printf("%s\n", exception);

    return NULL;
}

static char
global_heap_buf[CONFIG_GLOBAL_HEAP_BUF_SIZE] = { 0 };

void
iwasm_main(ULONG thread_input)
{
    uint8 *wasm_file_buf = NULL;
    uint32 wasm_file_size;
    wasm_module_t wasm_module = NULL;
    wasm_module_inst_t wasm_module_inst = NULL;
    RuntimeInitArgs init_args;
    char error_buf[128];
    int log_verbose_level = 2;

    memset(&init_args, 0, sizeof(RuntimeInitArgs));

    init_args.mem_alloc_type = Alloc_With_Pool;
    init_args.mem_alloc_option.pool.heap_buf = global_heap_buf;
    init_args.mem_alloc_option.pool.heap_size = sizeof(global_heap_buf);

    /* initialize runtime environment */
    if (!wasm_runtime_full_init(&init_args)) {
        printf("Init runtime environment failed.\n");
        return;
    }

    bh_log_set_verbose_level(log_verbose_level);

    /* load WASM byte buffer from byte buffer of include file */
    wasm_file_buf = (uint8 *)helloworld_app;
    wasm_file_size = sizeof(helloworld_app);

    /* load WASM module */
    if (!(wasm_module = wasm_runtime_load(wasm_file_buf, wasm_file_size,
                                          error_buf, sizeof(error_buf)))) {
        printf("%s\n", error_buf);
        goto fail1;
    }

    /* instantiate the module */
    if (!(wasm_module_inst = wasm_runtime_instantiate(
              wasm_module, CONFIG_APP_STACK_SIZE, CONFIG_APP_HEAP_SIZE,
              error_buf, sizeof(error_buf)))) {
        printf("%s\n", error_buf);
        goto fail2;
    }

    /* invoke the main function */
    app_instance_main(wasm_module_inst);

    /* destroy the module instance */
    wasm_runtime_deinstantiate(wasm_module_inst);

fail2:
    /* unload the module */
    wasm_runtime_unload(wasm_module);

fail1:
    /* destroy runtime environment */
    wasm_runtime_destroy();
}


void
tx_application_define(void *first_unused_memory)
{
	tx_thread_create(&iwasm_main_thread, "main_thread", iwasm_main, 0, first_unused_memory,
		MAIN_THREAD_STACK_SIZE, MAIN_THREAD_PRIORITY, MAIN_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
}

void
main(void)
{
	board_init();
    tx_kernel_enter();
}

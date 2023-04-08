#include "platform_api_vmcore.h"

// It does not implement memory allocation funcs since it uses Alloc_With_Pool type.

void *
os_malloc(unsigned size)
{
    return NULL;
}

void *
os_realloc(void *ptr, unsigned size)
{
    return NULL;
}

void
os_free(void *ptr)
{
}

int
os_dumps_proc_mem_info(char *out, unsigned int size)
{
    return -1;
}
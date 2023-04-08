#include "platform_api_vmcore.h"

void *
os_mmap(void *hint, size_t size, int prot, int flags)
{
    if ((uint64)size >= UINT32_MAX)
        return NULL;
    return BH_MALLOC(size);
}

void
os_munmap(void *addr, size_t size)
{
    BH_FREE(addr);
}

int
os_mprotect(void *addr, size_t size, int prot)
{
    return 0;
}

/**
 * Flush cpu data cache, in some CPUs, after applying relocation to the
 * AOT code, the code may haven't been written back to the cpu data cache,
 * which may cause unexpected behaviour when executing the AOT code.
 * Implement this function if required, or just leave it empty.
 */
void
os_dcache_flush()
{
}
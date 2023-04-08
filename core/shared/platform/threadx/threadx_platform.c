#include "platform_api_vmcore.h"
#include "platform_api_extension.h"

#define os_printf printf
#define os_vprintf vprintf

int
os_thread_sys_init();

void
os_thread_sys_destroy();

int
bh_platform_init()
{
    return os_thread_sys_init();
}

void
bh_platform_destroy()
{
    os_thread_sys_destroy();
}

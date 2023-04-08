#include "platform_api_vmcore.h"

uint64
os_time_get_boot_microsecond()
{
    return (uint64)tx_time_get() * TX_US_TICK_VALUE;
}

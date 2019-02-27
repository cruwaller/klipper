#include "generic/misc.h" // dynmem_start
#include <esp_attr.h> // DRAM_ATTR

#define DYNMEM_POOL (16 * 1024) // 16k

// must be 32bit aligned
DRAM_ATTR WORD_ALIGNED_ATTR static uint32_t dynmem_pool[DYNMEM_POOL];

// Return the start of memory available for dynamic allocations
void *
dynmem_start(void)
{
    return dynmem_pool;
}

// Return the end of memory available for dynamic allocations
void *
dynmem_end(void)
{
    return (uint8_t*)&dynmem_pool[DYNMEM_POOL];
}

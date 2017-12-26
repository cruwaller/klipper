#include "mempool.h"
#include <mpu.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>

#define MRI_SEMIHOST_STDIO 0
#define __debugbreak()
#define __mriNewlib_SemihostWrite(x,y,z)   0
#define __mriNewlib_SemihostRead(x,y,z)    0

/****************************************************************
 * startup init functions
 ****************************************************************/

#define STACK_SIZE  3072

POOL * _AHB0 = NULL;
POOL * _AHB1 = NULL;

extern unsigned int __bss_start__;
extern unsigned int __bss_end__;
extern unsigned int __StackTop;
extern unsigned int __end__;

extern uint8_t __AHB0_block_start;
extern uint8_t __AHB0_dyn_start;
extern uint8_t __AHB0_end;
extern uint8_t __AHB1_block_start;
extern uint8_t __AHB1_dyn_start;
extern uint8_t __AHB1_end;

static void fillUnusedRAM(void);
static void configureStackSizeLimit(unsigned int stackSizeLimit);
static unsigned int alignTo32Bytes(unsigned int value);
static void configureMpuToCatchStackOverflowIntoHeap(unsigned int maximumHeapAddress);

unsigned int g_maximumHeapAddress;


extern "C" void exit (int status) {
    while(1);
}
extern "C" void abort(void)
{
    //if (MRI_ENABLE)
    //    __debugbreak();

    exit(1);
}
/*void _exit(int status) {
    exit(status);
    }*/
extern "C" int main(void);
extern "C" void __libc_init_array(void);
extern "C" void _start(void)
{
    size_t bssSize = (uintptr_t)&__bss_end__ - (uintptr_t)&__bss_start__;
    int mainReturnValue;

    memset(&__bss_start__, 0, bssSize);
    fillUnusedRAM();

    if (STACK_SIZE) {
        configureStackSizeLimit(STACK_SIZE);
    }

    // zero the data sections in AHB0 and AHB1
    //memset(&__AHB0_block_start, 0, &__AHB0_dyn_start - &__AHB0_block_start);
    //memset(&__AHB1_block_start, 0, &__AHB1_dyn_start - &__AHB1_block_start);

    // MemoryPool stuff - needs to be initialised before __libc_init_array
    // so static ctors can use them
    //_AHB0 = pool_create(&__AHB0_dyn_start, ((uintptr_t)&__AHB0_end - (uintptr_t)&__AHB0_dyn_start));
    //_AHB1 = pool_create(&__AHB1_dyn_start, ((uintptr_t)&__AHB1_end - (uintptr_t)&__AHB1_dyn_start));

    __libc_init_array();
    mainReturnValue = main();
    exit(mainReturnValue);
    exit(0);
}

static __attribute__((naked)) void fillUnusedRAM(void)
{
    __asm (
        ".syntax unified\n"
        ".thumb\n"
        // Fill 2 words (8 bytes) at a time with 0xdeadbeef.
        " ldr   r2, =__FillStart\n"
        " movw  r0, #0xbeef\n"
        " movt  r0, #0xdead\n"
        " mov   r1, r0\n"
        // Don't fill past current stack pointer value.
        " mov   r3, sp\n"
        " bics  r3, r3, #7\n"
        "1$:\n"
        " strd  r0, r1, [r2], #8\n"
        " cmp   r2, r3\n"
        " blo   1$\n"
        " bx    lr\n"
    );
}

static void configureStackSizeLimit(unsigned int stackSizeLimit)
{
    // Note: 32 bytes are reserved to fall between top of heap and top of stack for minimum MPU guard region.
    g_maximumHeapAddress = alignTo32Bytes((unsigned int)&__StackTop - stackSizeLimit - 32);
    configureMpuToCatchStackOverflowIntoHeap(g_maximumHeapAddress);
}

static unsigned int alignTo32Bytes(unsigned int value)
{
    return (value + 31) & ~31;
}

static void configureMpuToCatchStackOverflowIntoHeap(unsigned int maximumHeapAddress)
{
#define MPU_REGION_SIZE_OF_32_BYTES ((5-1) << MPU_RASR_SIZE_SHIFT)  // 2^5 = 32 bytes.

    prepareToAccessMPURegion(getHighestMPUDataRegionIndex());
    setMPURegionAddress(maximumHeapAddress);
    setMPURegionAttributeAndSize(MPU_REGION_SIZE_OF_32_BYTES | MPU_RASR_ENABLE);
    enableMPUWithDefaultMemoryMap();
}

static void configureMpuRegionToAccessAllMemoryWithNoCaching(void)
{
    static const uint32_t regionToStartAtAddress0 = 0U;
    static const uint32_t regionReadWrite = 1  << MPU_RASR_AP_SHIFT;
    static const uint32_t regionSizeAt4GB = 31 << MPU_RASR_SIZE_SHIFT; /* 4GB = 2^(31+1) */
    static const uint32_t regionEnable    = MPU_RASR_ENABLE;
    static const uint32_t regionSizeAndAttributes = regionReadWrite | regionSizeAt4GB | regionEnable;
    uint32_t regionIndex = STACK_SIZE ? getHighestMPUDataRegionIndex() - 1 : getHighestMPUDataRegionIndex();

    prepareToAccessMPURegion(regionIndex);
    setMPURegionAddress(regionToStartAtAddress0);
    setMPURegionAttributeAndSize(regionSizeAndAttributes);
}

#if 0
extern "C" int __real__read(int file, char *ptr, int len);
extern "C" int __wrap__read(int file, char *ptr, int len)
{
    if (MRI_SEMIHOST_STDIO && file < 3)
        return __mriNewlib_SemihostRead(file, ptr, len);
    return __real__read(file, ptr, len);
}


extern "C" int __real__write(int file, char *ptr, int len);
extern "C" int __wrap__write(int file, char *ptr, int len)
{
    if (MRI_SEMIHOST_STDIO && file < 3)
        return __mriNewlib_SemihostWrite(file, ptr, len);
    return __real__write(file, ptr, len);
}


extern "C" int __real__isatty(int file);
extern "C" int __wrap__isatty(int file)
{
    /* Hardcoding the stdin/stdout/stderr handles to be interactive tty devices, unlike mbed.ar */
    if (file < 3)
        return 1;
    return __real__isatty(file);
}


extern "C" int __wrap_semihost_connected(void)
{
    /* MRI makes it look like there is no mbed interface attached since it disables the JTAG portion but MRI does
       support some of the mbed semihost calls when it is running so force it to return -1, indicating that the
       interface is attached. */
    return -1;
}



extern "C" void abort(void)
{
    //if (MRI_ENABLE)
    //    __debugbreak();

    exit(1);
}


extern "C" void __cxa_pure_virtual(void)
{
    abort();
}


/* Trap calls to malloc/free/realloc in ISR. */
extern "C" void __malloc_lock(void)
{
    if (__get_IPSR() != 0)
        __debugbreak();
}

extern "C" void __malloc_unlock(void)
{
}
#endif

/* Turn off the errno macro and use actual external global variable instead. */
#undef errno
extern int errno;

static int doesHeapCollideWithStack(unsigned int newHeap);

/* Dynamic memory allocation related syscalls. */
extern "C" caddr_t _sbrk(int incr)
{
    static unsigned char *heap = (unsigned char *)&__end__;
    unsigned char        *prev_heap = heap;
    unsigned char        *new_heap = heap + incr;

    if (doesHeapCollideWithStack((unsigned int)new_heap)) {
        //errno = ENOMEM;
        return (caddr_t) - 1;
    }

    heap = new_heap;
    return (caddr_t) prev_heap;
}

static int doesHeapCollideWithStack(unsigned int newHeap)
{
    return ((newHeap >= __get_MSP()) ||
            (STACK_SIZE && newHeap >= g_maximumHeapAddress));
}

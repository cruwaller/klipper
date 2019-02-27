
#include "board/irq.h"

#ifdef USE_IRQ_MUTEX

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define LOCK_TYPE 0 // 0 = None, 1 = MUTEX, 2 = portMUX

#if (LOCK_TYPE == 1)
static xSemaphoreHandle irq_lock = NULL;
#  define IRQ_MUTEX_LOCK(_lock)    while (xSemaphoreTake(_lock, portMAX_DELAY) != pdPASS);
#  define IRQ_MUTEX_UNLOCK(_lock)  xSemaphoreGive(_lock)
#elif (LOCK_TYPE == 2)
static portMUX_TYPE irq_lock = portMUX_INITIALIZER_UNLOCKED;
#  define IRQ_MUTEX_LOCK(_lock)   portENTER_CRITICAL(&_lock);
#  define IRQ_MUTEX_UNLOCK(_lock) portEXIT_CRITICAL(&_lock);
#else
#  define IRQ_MUTEX_LOCK(_lock)
#  define IRQ_MUTEX_UNLOCK(_lock)
#endif

void irq_disable(void)
{
    IRQ_MUTEX_LOCK(irq_lock);
}

void irq_enable(void)
{
    IRQ_MUTEX_UNLOCK(irq_lock);
}

irqstatus_t irq_save(void)
{
    IRQ_MUTEX_LOCK(irq_lock);
    return 0xBABE;
}

void irq_restore(irqstatus_t flag)
{
    if (flag == 0xBABE)
        IRQ_MUTEX_UNLOCK(irq_lock);
}

void irq_wait(void)
{
}

void irq_poll(void)
{
}

#endif /* USE_IRQ_MUTEX */

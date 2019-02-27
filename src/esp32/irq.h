#ifndef __GENERIC_IRQ_H
#define __GENERIC_IRQ_H

#include <stdint.h>

/* TODO: IRQ disable should pause timer? */
//#define IRQ_STOP_TMR 1
//#define USE_IRQ_MUTEX 1

typedef unsigned long irqstatus_t;

#if (IRQ_STOP_TMR)

// Implemented in timer.c
void irq_disable(void);
void irq_enable(void);

#define irq_save() 0; irq_disable();
#define irq_restore(_flag) (void)(_flag); irq_enable();
#define irq_wait()
#define irq_poll()


#elif defined(USE_IRQ_MUTEX)

void irq_disable(void);
void irq_enable(void);
irqstatus_t irq_save(void);
void irq_restore(irqstatus_t flag);
void irq_wait(void);
void irq_poll(void);

#else

#define irq_disable()
#define irq_enable()
#define irq_save() 0
#define irq_restore(_flag) (void)(_flag)
#define irq_wait()
#define irq_poll()

#endif

#endif // irq.h

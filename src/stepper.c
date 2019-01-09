// Handling of stepper drivers.
//
// Copyright (C) 2016  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_is_before
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "stepper.h" // command_config_stepper

#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
#include <stdio.h>
#endif

DECL_CONSTANT(STEP_DELAY, CONFIG_STEP_DELAY);

/****************************************************************
 * Steppers
 ****************************************************************/

struct stepper_move {
    uint32_t interval;
    int16_t add;
    uint16_t count;
    struct stepper_move *next;
    uint8_t flags : 8;
};

enum { MF_DIR=1<<0 };

struct stepper {
    struct timer time;
    struct gpio_out step_pin, dir_pin;
    struct end_stop *endstop;
    struct stepper_move *first, **plast;
    uint32_t interval;
    uint32_t position;
    uint32_t min_stop_interval;
#if !CONFIG_STEP_DELAY
    uint16_t count;
#define next_step_time time.waketime
#else
    uint32_t count;
    uint32_t next_step_time;
#endif
    int16_t add;
    // gcc (pre v6) does better optimization when uint8_t are bitfields
    uint8_t flags : 8;
};

enum { POSITION_BIAS=0x40000000 };

enum { SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_INVERT_STEP=1<<2, SF_HAVE_ADD=1<<3,
       SF_LAST_RESET=1<<4, SF_NO_NEXT_CHECK=1<<5, ES_CHECK_ENDSTOP=1<<6 };


// Setup a stepper for the next move in its queue
static uint_fast8_t
stepper_load_next(struct stepper *s, uint32_t min_next_time)
{
    struct stepper_move *m = s->first;
    if (!m) {
        if ((s->interval - s->add) < s->min_stop_interval
            && !(s->flags & SF_NO_NEXT_CHECK))
            shutdown("No next step");
        s->count = 0;
        return SF_DONE;
    }

    s->next_step_time += m->interval;
    s->add = m->add;
    s->interval = m->interval + m->add;
#if !CONFIG_STEP_DELAY
    (void)min_next_time;
        // On slow mcus see if the add can be optimized away
        s->flags = m->add ? s->flags | SF_HAVE_ADD : s->flags & ~SF_HAVE_ADD;
        s->count = m->count;
#else
        // On faster mcus, it is necessary to schedule unstep events
        // and so there are twice as many events.  Also check that the
        // next step event isn't too close to the last unstep.
        if (unlikely(timer_is_before(s->next_step_time, min_next_time))) {
            if ((int32_t)(s->next_step_time - min_next_time)
                < (int32_t)(-timer_from_us(1000)))
#if (CONFIG_SIMULATOR == 0)
                shutdown("Stepper too far in past");
#endif
            s->time.waketime = min_next_time;
        } else {
            s->time.waketime = s->next_step_time;
        }
        s->count = m->count * 2;
#endif
    if (m->flags & MF_DIR) {
        s->position = -s->position + m->count;
        gpio_out_toggle_noirq(s->dir_pin);
    } else {
        s->position += m->count;
    }

    s->first = m->next;
    move_free(m);
    return SF_RESCHEDULE;
}

// Timer callback - step the given stepper.
uint_fast8_t
stepper_event(struct timer *t)
{
    struct stepper *s = container_of(t, struct stepper, time);
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
//    printf("stepper_event: interval %u, count %u add %d next_step_time %u wakeup_time: %u\n",
//           s->interval, s->count, s->add, s->next_step_time, s->time.waketime);
#endif
#if (!CONFIG_STEP_DELAY)
        // On slower mcus it is possible to simply step and unstep in
        // the same timer event.
        gpio_out_toggle_noirq(s->step_pin);
        uint16_t count = s->count - 1;
        if (likely(count)) {
            s->count = count;
            s->time.waketime += s->interval;
            gpio_out_toggle_noirq(s->step_pin);
            if (s->flags & SF_HAVE_ADD)
                s->interval += s->add;
            return SF_RESCHEDULE;
        }
        uint_fast8_t ret = stepper_load_next(s, 0);
        gpio_out_toggle_noirq(s->step_pin);
        return ret;
#else // (!CONFIG_STEP_DELAY)

    // On faster mcus, it is necessary to schedule the unstep event
    uint32_t step_delay = timer_from_us(CONFIG_STEP_DELAY);
    uint32_t min_next_time = timer_read_time() + step_delay;
    gpio_out_toggle_noirq(s->step_pin);
    s->count--;
    if (likely(s->count & 1))
        // Schedule unstep event
        goto reschedule_min;
    if (likely(s->count)) {
        s->next_step_time += s->interval;
        s->interval += s->add;
        if (unlikely(timer_is_before(s->next_step_time, min_next_time)))
            // The next step event is too close - push it back
            goto reschedule_min;
        s->time.waketime = s->next_step_time;
        return SF_RESCHEDULE;
    }
    if (likely(s->flags & ES_CHECK_ENDSTOP)) {
        extern uint_fast8_t end_stop_checkpin(struct end_stop *e);
        if (unlikely(end_stop_checkpin(s->endstop)))
            return SF_DONE; // Stop immediately
    }
    return stepper_load_next(s, min_next_time);
reschedule_min:
    s->time.waketime = min_next_time;
    return SF_RESCHEDULE;

#endif // (!CONFIG_STEP_DELAY)
}

void
command_config_stepper(uint32_t *args)
{
    struct stepper *s = oid_alloc(args[0], command_config_stepper, sizeof(*s));
    s->time.func = NULL;
    if (!CONFIG_INLINE_STEPPER_HACK)
        s->time.func = stepper_event;
    s->flags = args[4] ? SF_INVERT_STEP : 0;
    s->step_pin = gpio_out_setup(args[1], s->flags & SF_INVERT_STEP);
    s->dir_pin = gpio_out_setup(args[2], 0);
    s->min_stop_interval = args[3];
    s->position = -POSITION_BIAS;
    move_request_size(sizeof(struct stepper_move));
}
DECL_COMMAND(command_config_stepper,
             "config_stepper oid=%c step_pin=%c dir_pin=%c"
             " min_stop_interval=%u invert_step=%c");

// Return the 'struct stepper' for a given stepper oid
struct stepper *
stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper);
}

// Schedule a set of steps with a given timing
void
command_queue_step(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_move *m = move_alloc();
    m->interval = args[1];
    m->count = args[2];
    if (!m->count)
        shutdown("Invalid count parameter");
    m->add = args[3];
    m->next = NULL;
    m->flags = 0;

#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    printf("queue_step: next_step_time %u + interval %u => count %u add %d \n",
           s->next_step_time, m->interval, m->count, m->add);
#endif

    irq_disable();
    uint8_t flags = s->flags;
    if (!!(flags & SF_LAST_DIR) != !!(flags & SF_NEXT_DIR)) {
        flags ^= SF_LAST_DIR;
        m->flags |= MF_DIR;
    }
    flags &= ~SF_NO_NEXT_CHECK;
    if (m->count == 1 && (m->flags || flags & SF_LAST_RESET))
        // count=1 moves after a reset or dir change can have small intervals
        flags |= SF_NO_NEXT_CHECK;
    s->flags = flags & ~SF_LAST_RESET;
    if (s->count) {
        if (s->first)
            *s->plast = m;
        else
            s->first = m;
        s->plast = &m->next;
    } else {
        s->first = m;
        stepper_load_next(s, s->next_step_time + m->interval);
        sched_add_timer(&s->time);
    }
    irq_enable();
}
DECL_COMMAND(command_queue_step,
             "queue_step oid=%c interval=%u count=%hu add=%hi");

// Set the direction of the next queued step
void
command_set_next_step_dir(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    printf("Stepper dir: %u \n", nextdir);
#endif
}
DECL_COMMAND(command_set_next_step_dir, "set_next_step_dir oid=%c dir=%c");

// Set an absolute time that the next step will be relative to
void
command_reset_step_clock(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t waketime = args[1];
    irq_disable();
    if (s->count)
        shutdown("Can't reset time when stepper active");
    s->next_step_time = waketime;
    s->flags |= SF_LAST_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_step_clock, "reset_step_clock oid=%c clock=%u");

// Return the current stepper position.  Caller must disable irqs.
static uint32_t
stepper_get_position(struct stepper *s)
{
    uint32_t position = s->position;
#if (!CONFIG_STEP_DELAY)
    position -= s->count;
#else
    position -= s->count / 2;
#endif
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    printf("Stepper get position: %d \n", (position & 0x80000000) ? -position : position);
#endif
    if (position & 0x80000000)
        return -position;
    return position;
}

// Report the current position of the stepper
void
command_stepper_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper *s = stepper_oid_lookup(oid);
    irq_disable();
    uint32_t position = stepper_get_position(s);
    irq_enable();
    sendf("stepper_position oid=%c pos=%i", oid, position - POSITION_BIAS);
}
DECL_COMMAND(command_stepper_get_position, "stepper_get_position oid=%c");

// Stop all moves for a given stepper (used in end stop homing).  IRQs
// must be off.
void
stepper_stop(struct stepper *s)
{
    sched_del_timer(&s->time);
    s->next_step_time = 0;
    s->position = -stepper_get_position(s);
    s->count = 0;
    s->flags &= SF_INVERT_STEP;
    gpio_out_write(s->dir_pin, 0);
    gpio_out_write(s->step_pin, s->flags & SF_INVERT_STEP);
    while (s->first) {
        struct stepper_move *next = s->first->next;
        move_free(s->first);
        s->first = next;
    }
}

void
stepper_shutdown(void)
{
    uint8_t i;
    struct stepper *s;
    foreach_oid(i, s, command_config_stepper) {
        s->first = NULL;
        stepper_stop(s);
    }
}
DECL_SHUTDOWN(stepper_shutdown);


/*****************************************
 * HOMING
 ****************************************/
void
stepper_set_endstop(struct end_stop *e, uint8_t oid)
{
    struct stepper *s = stepper_oid_lookup(oid);
    s->endstop = e;
}

void
stepper_endstop_enable(struct stepper *s, uint8_t enable)
{
    if (!s->endstop)
        shutdown("Endstop is not configured");
    if (enable)
        s->flags |= ES_CHECK_ENDSTOP;
    else
        s->flags &= ~ES_CHECK_ENDSTOP;
}

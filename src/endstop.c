// Handling of end stops.
//
// Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // struct gpio
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "stepper.h" // stepper_stop

#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
#include <stdio.h>
#endif

struct end_stop {
    struct timer time;
    struct gpio_in pin;
    uint32_t rest_time, sample_time, nextwake;
    uint8_t flags, stepper_count, sample_count, trigger_count;
    struct stepper *steppers[0];
};

enum { ESF_PIN_HIGH=1<<0, ESF_HOMING=1<<1, ESF_REPORT=1<<2 };

static struct task_wake endstop_wake;

static void
stop_steppers(struct end_stop *e)
{
    e->flags = ESF_REPORT;
    uint8_t count = e->stepper_count;
    while (count--)
        if (e->steppers[count])
            stepper_stop(e->steppers[count]);
    sched_wake_task(&endstop_wake);
}

static uint_fast8_t end_stop_oversample_event(struct timer *t);

// Timer callback for an end stop
static uint_fast8_t
end_stop_event(struct timer *t)
{
    struct end_stop *e = container_of(t, struct end_stop, time);
    uint8_t val = gpio_in_read(e->pin);
    uint32_t nextwake = e->time.waketime + e->rest_time;
    if ((val ? ~e->flags : e->flags) & ESF_PIN_HIGH) {
        // No match - reschedule for the next attempt
        e->time.waketime = nextwake;
        return SF_RESCHEDULE;
    }
    e->nextwake = nextwake;
    e->time.func = end_stop_oversample_event;
    return end_stop_oversample_event(t);
}

// Timer callback for an end stop that is sampling extra times
static uint_fast8_t
end_stop_oversample_event(struct timer *t)
{
    struct end_stop *e = container_of(t, struct end_stop, time);
    uint8_t val = gpio_in_read(e->pin);
    if ((val ? ~e->flags : e->flags) & ESF_PIN_HIGH) {
        // No longer matching - reschedule for the next attempt
        e->time.func = end_stop_event;
        e->time.waketime = e->nextwake;
        e->trigger_count = e->sample_count;
        return SF_RESCHEDULE;
    }
    uint8_t count = e->trigger_count - 1;
    if (!count) {
        stop_steppers(e);
        return SF_DONE;
    }
    e->trigger_count = count;
    e->time.waketime += e->sample_time;
    return SF_RESCHEDULE;
}

uint_fast8_t
end_stop_checkpin(struct end_stop *e)
{
    uint8_t val = gpio_in_read(e->pin);
    uint8_t res = (val ? e->flags : ~e->flags) & ESF_PIN_HIGH;
    if (res)
        // Match -> stop
        stop_steppers(e);
    return res;
}

void
command_config_end_stop(uint32_t *args)
{
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    printf("config_end_stop() oid %u stepper_count %d, pin %d, pull_up %d\n",
           args[0], args[3], args[1], args[2]);
#endif
    uint8_t stepper_count = args[3];
    struct end_stop *e = oid_alloc(
        args[0], command_config_end_stop
        , sizeof(*e) + sizeof(e->steppers[0]) * stepper_count);
    e->pin = gpio_in_setup(args[1], args[2]);
    e->stepper_count = stepper_count;
    e->sample_count = 1;
}
DECL_COMMAND(command_config_end_stop,
             "config_end_stop oid=%c pin=%c pull_up=%c stepper_count=%c");

void
command_end_stop_set_stepper(uint32_t *args)
{
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    printf("end_stop_set_stepper() oid %u, pos %d, stepper_oid %d\n",
           args[0], args[1], args[2]);
#endif
    struct end_stop *e = oid_lookup(args[0], command_config_end_stop);
    uint8_t pos = args[1];
    if (pos >= e->stepper_count)
        shutdown("Set stepper past maximum stepper count");
    e->steppers[pos] = stepper_oid_lookup(args[2]);
#if (STEPPER_POLL_END_STOP)
    stepper_set_endstop(e, args[2]);
#endif
}
DECL_COMMAND(command_end_stop_set_stepper,
             "end_stop_set_stepper oid=%c pos=%c stepper_oid=%c");

// Home an axis
void
command_end_stop_home(uint32_t *args)
{
    struct end_stop *e = oid_lookup(args[0], command_config_end_stop);
    sched_del_timer(&e->time);
    e->time.waketime = args[1];
    e->sample_time = args[2];
    e->sample_count = args[3];
    if (!e->sample_count) {
        // Disable end stop checking
        e->flags = 0;
#if (STEPPER_POLL_END_STOP)
        uint8_t count = e->stepper_count;
        while (count--)
            if (e->steppers[count])
                stepper_endstop_enable(e->steppers[count], 0);
#endif
        return;
    }
    e->rest_time = args[4];
    e->time.func = end_stop_event;
    e->trigger_count = e->sample_count;
    e->flags = ESF_HOMING | (args[5] ? ESF_PIN_HIGH : 0);
#if (STEPPER_POLL_END_STOP)
    uint8_t count = e->stepper_count;
    while (count--)
        if (e->steppers[count])
            stepper_endstop_enable(e->steppers[count], 1);
#else
    sched_add_timer(&e->time);
#endif
}
DECL_COMMAND(command_end_stop_home,
             "end_stop_home oid=%c clock=%u sample_ticks=%u sample_count=%c"
             " rest_ticks=%u pin_value=%c");

static void
end_stop_report(uint8_t oid, struct end_stop *e)
{
    irq_disable();
    uint8_t eflags = e->flags;
    e->flags &= ~ESF_REPORT;
    irq_enable();

    sendf("end_stop_state oid=%c homing=%c pin=%c"
          , oid, !!(eflags & ESF_HOMING), gpio_in_read(e->pin));
}

void
command_end_stop_query(uint32_t *args)
{
    uint8_t oid = args[0];
    struct end_stop *e = oid_lookup(oid, command_config_end_stop);
    end_stop_report(oid, e);
}
DECL_COMMAND(command_end_stop_query, "end_stop_query oid=%c");

void
end_stop_task(void)
{
    if (!sched_check_wake(&endstop_wake))
        return;
    uint8_t oid;
    struct end_stop *e;
    foreach_oid(oid, e, command_config_end_stop) {
        if (!(e->flags & ESF_REPORT))
            continue;
        end_stop_report(oid, e);
    }
}
DECL_TASK(end_stop_task);

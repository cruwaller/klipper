#include "autoconf.h" // CONFIG_*

#include <string.h>

#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_is_before
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "stepper.h" // command_config_stepper
#include "spicmds.h"
#include "byteorder.h" // be32_to_cpu

#if (CONFIG_SIMULATOR == 1)
#include <stdio.h>
#endif

//#define TIMER_ADVANCE timer_from_us(10)
#define TIMER_ADVANCE 0

struct stepper_tmc5x * stepper_tmc5x_oid_lookup(uint8_t oid);
void stepper_tmc5x_stop(struct stepper_tmc5x *s);

/* TMC51xx registers */
#define REG_RAMPMODE   0x20
#define REG_XACTUAL    0x21
#define REG_VACTUAL    0x22
#define REG_VSTART     0x23
#define REG_A1         0x24
#define REG_V1         0x25
#define REG_AMAX       0x26
#define REG_VMAX       0x27
#define REG_DMAX       0x28
#define REG_D1         0x2A
#define REG_VSTOP      0x2B
#define REG_TZEROWAIT  0x2C
#define REG_XTARGET    0x2D

#define REG_RAMP_STAT  0x35

// Define RAMP modes
#define RAMPMODE_POSITIONING  0x00
#define RAMPMODE_VELOCITY_POS 0x01
#define RAMPMODE_VELOCITY_NEG 0x02
#define RAMPMODE_HOLD         0x03


/****************************************************************
 * SPI stepper driver
 ****************************************************************/
#define FILL_IN_ISR 1

/* Need to be a 32bit aligned */
struct stepper_tmc5x_move { // 24bytes
    struct stepper_tmc5x_move *next;
    uint32_t interval;
#if FILL_IN_ISR
    // 16bytes
    uint32_t target;
    uint32_t vstart;
    uint32_t vmax;
    uint16_t amax;
    uint16_t dmax;
#else
    // 32bytes
    uint16_t len;
    uint8_t cmd[30]; // totally 6 commands ( 6 * 5B )
#endif
};

struct stepper_tmc5x {
    struct timer timer;
    struct spidev_s *spi;
    struct stepper_tmc5x_move *first, **plast;
    uint32_t next_step_time;
    uint32_t homing_report_interval;
};


/*****************************************
 * PRIVATE
 ****************************************/

#if FILL_IN_ISR
static uint8_t move_cmd[30];
static void
send_move_command(struct spidev_s *spi,
                  struct stepper_tmc5x_move *m)
{
    uint8_t * const cmd = move_cmd;
    // VMAX
    uint32_t temp = cpu_to_be32(m->vmax);
    cmd[0] = 0x80 | REG_VMAX;
    memcpy(&cmd[1], &temp, 4);
    // AMAX
    temp = cpu_to_be32(m->amax);
    cmd[5] = 0x80 | REG_AMAX;
    memcpy(&cmd[6], &temp, 4);
    // DMAX
    temp = cpu_to_be32(m->dmax);
    cmd[10] = 0x80 | REG_DMAX;
    memcpy(&cmd[11], &temp, 4);
    // VSTART
    uint32_t vstart = m->vstart;
    temp = cpu_to_be32(vstart);
    cmd[15] = 0x80 | REG_VSTART;
    memcpy(&cmd[16], &temp, 4);
    // VSTOP
    vstart = ++vstart < 10 ? 10 : vstart;
    temp = cpu_to_be32(vstart);
    cmd[20] = 0x80 | REG_VSTOP;
    memcpy(&cmd[21], &temp, 4);
    // XTARGET
    temp = cpu_to_be32(m->target);
    cmd[25] = 0x80 | REG_XTARGET;
    memcpy(&cmd[26], &temp, 4);
    spidev_transfer(spi, 0, 30, cmd);
}
#endif

// Timer callback - step the given stepper.
static uint_fast8_t
stepper_tmc5x_event(struct timer *t)
{
    struct stepper_tmc5x *s = container_of(t, struct stepper_tmc5x, timer);
    struct stepper_tmc5x_move *next, *m = s->first;
#if (CONFIG_SIMULATOR == 1)
    printf("SPI STEP EVENT %u [next_step_time %u]\n", s->timer.waketime, s->next_step_time);
#endif
    if (!m)
        goto ready;
#if FILL_IN_ISR
    send_move_command(s->spi, m);
#else
    spidev_transfer(s->spi, 0, m->len, m->cmd);
#endif
    s->first = next = m->next;
    move_free(m);
    if (!next)
        goto ready;
    s->next_step_time = s->timer.waketime + next->interval;
    s->timer.waketime = s->next_step_time;
    return SF_RESCHEDULE;
ready:
    s->next_step_time = s->timer.waketime;
    return SF_DONE; // No more moves
}

/*****************************************
 * MOVEMENT
 ****************************************/

void
command_stepper_tmc5x_config(uint32_t *args)
{
#if (CONFIG_SIMULATOR == 1)
    printf("command_stepper_tmc5x_config()\n");
#endif
    struct stepper_tmc5x *s = oid_alloc(args[0], command_stepper_tmc5x_config, sizeof(*s));
    s->timer.func = stepper_tmc5x_event;
    s->spi = spidev_oid_lookup(args[1]);
    s->first = NULL;
    s->plast = NULL;
    s->homing_report_interval = 0;
    s->next_step_time = 0 - TIMER_ADVANCE;
    move_request_size(sizeof(struct stepper_tmc5x_move));
#if (CONFIG_SIMULATOR == 1)
    printf("stepper_tmc5x_config oid %u spi_oid %u\n", args[0], args[1]);
#endif
}
DECL_COMMAND(command_stepper_tmc5x_config,
             "stepper_tmc5x_config oid=%c spi_oid=%c");

// Schedule next move
void
command_stepper_tmc5x_queue(uint32_t *args)
{
#if (CONFIG_SIMULATOR == 1)
    printf("command_stepper_tmc5x_queue()\n");
#endif
    struct stepper_tmc5x *s = stepper_tmc5x_oid_lookup(args[0]);
#if (CONFIG_SIMULATOR == 1)
    printf("stepper_tmc5x_queue: next_step_time %u + m->interval %u => count %d, amax %u, dmax %u, vmax %u, vstart %u\n",
           s->next_step_time, args[1], args[2], args[3], args[4], args[5], args[6]);
#endif
    struct stepper_tmc5x_move *m = move_alloc();
    m->next = NULL;
#if FILL_IN_ISR
    m->target = args[2];
    m->amax = args[3];
    m->dmax = args[4];
    m->vmax = args[5];
    m->vstart = args[6];
#else
    uint8_t * const msg = m->cmd;
    // VMAX
    uint32_t temp = cpu_to_be32(args[5]);
    msg[0] = 0x80 | REG_VMAX;
    memcpy(&msg[1], &temp, 4);
    // AMAX
    temp = cpu_to_be32(args[3]);
    msg[5] = 0x80 | REG_AMAX;
    memcpy(&msg[6], &temp, 4);
    // DMAX
    temp = cpu_to_be32(args[4]);
    msg[10] = 0x80 | REG_DMAX;
    memcpy(&msg[11], &temp, 4);
    // VSTART
    uint32_t vstart = args[6];
    temp = cpu_to_be32(vstart);
    msg[15] = 0x80 | REG_VSTART;
    memcpy(&msg[16], &temp, 4);
    // VSTOP
    vstart = ++vstart < 10 ? 10 : vstart;
    temp = cpu_to_be32(vstart);
    msg[20] = 0x80 | REG_VSTOP;
    memcpy(&msg[21], &temp, 4);
    // XTARGET
    temp = cpu_to_be32(args[2]);
    msg[25] = 0x80 | REG_XTARGET;
    memcpy(&msg[26], &temp, 4);
    m->len = 30;
#endif
    m->interval = args[1];
    irq_disable();
    if (unlikely(s->first)) {
        *s->plast = m;
    } else {
        s->first = m;
        s->timer.waketime = s->next_step_time + m->interval;
        sched_add_timer(&s->timer);
    }
    s->plast = &m->next;
    irq_enable();
}
DECL_COMMAND(command_stepper_tmc5x_queue,
             "stepper_tmc5x_queue oid=%c interval=%u target=%u"
             " amax=%hu dmax=%hu vmax=%u vstart=%u");

static void
reset_step_clock(struct stepper_tmc5x *s, uint32_t waketime) {
#if (CONFIG_SIMULATOR == 1)
    printf("reset_step_clock()\n");
#endif
    irq_disable();
    if (s->first)
        shutdown("Can't reset time when stepper active");
    s->next_step_time = waketime - TIMER_ADVANCE;
    irq_enable();
}

// Set an absolute time that the next step will be relative to
void
command_stepper_tmc5x_reset_step_clock(uint32_t *args)
{
#if (CONFIG_SIMULATOR == 1)
    printf("command_stepper_tmc5x_reset_step_clock()\n");
#endif
    struct stepper_tmc5x *s = stepper_tmc5x_oid_lookup(args[0]);
    uint32_t waketime = args[1];
    reset_step_clock(s, waketime);
}
DECL_COMMAND(command_stepper_tmc5x_reset_step_clock,
             "stepper_tmc5x_reset_step_clock oid=%c clock=%u");

/*****************************************
 * POSITION
 ****************************************/
void
command_stepper_tmc5x_get_position(uint32_t *args)
{
#if (CONFIG_SIMULATOR == 1)
    printf("command_stepper_tmc5x_get_position()\n");
#endif
    struct stepper_tmc5x *s = stepper_tmc5x_oid_lookup(args[0]);
    uint8_t msg[5] = { REG_XACTUAL, 0x00, 0x00, 0x00, 0x00 };
    spidev_transfer(s->spi, 0, 5, msg);
    spidev_transfer(s->spi, 1, 5, msg);
    uint32_t value;
    memcpy(&value, &msg[1], 4);
    value = be32_to_cpu(value);
#if (CONFIG_SIMULATOR == 1)
    printf("stepper_tmc5x_get_position: XACTUAL = %u\n", value);
#endif
    sendf("stepper_tmc5x_position oid=%c position=%u",
          args[0], value);
}
DECL_COMMAND(command_stepper_tmc5x_get_position,
             "stepper_tmc5x_get_position oid=%c");

void
command_stepper_tmc5x_set_position(uint32_t *args)
{
#if (CONFIG_SIMULATOR == 1)
    printf("command_stepper_tmc5x_set_position()\n");
#endif
    struct stepper_tmc5x *s = stepper_tmc5x_oid_lookup(args[0]);
    uint32_t position = cpu_to_be32(args[2]);
    // Set mode to 'hold'
    uint8_t msg[5] = { REG_RAMPMODE | 0x80, 0x00, 0x00, 0x00, RAMPMODE_HOLD };
    spidev_transfer(s->spi, 0, 5, msg);
    // Write actual register
    msg[0] = 0x80 | REG_XACTUAL;
    memcpy(&msg[1], &position, 4);
    spidev_transfer(s->spi, 0, 5, msg);
    // Write target register
    msg[0] = 0x80 | REG_XTARGET;
    spidev_transfer(s->spi, 0, 5, msg);
    // Set mode back to 'positioning'
    memset(msg, 0, 5);
    msg[0] = 0x80 | REG_RAMPMODE;
    msg[4] = RAMPMODE_POSITIONING;
    spidev_transfer(s->spi, 0, 5, msg);
    if (args[1]) {
        reset_step_clock(s, 0);
    }
#if (CONFIG_SIMULATOR == 1)
    printf("stepper_tmc5x_set_position: position = %u, reset clock %u\n", args[2], args[1]);
#endif
}
DECL_COMMAND(command_stepper_tmc5x_set_position,
             "stepper_tmc5x_set_position oid=%c reset_clock=%c position=%u");


/*****************************************
 * HOMING
 ****************************************/

static struct task_wake home_wake;

static uint_fast8_t
stepper_tmc5x_homing_event(struct timer *t)
{
    struct stepper_tmc5x *s = container_of(t, struct stepper_tmc5x, timer);
    s->timer.waketime += s->homing_report_interval;
    sched_wake_task(&home_wake);
    return SF_RESCHEDULE;
}

// handle homing
void
command_stepper_tmc5x_home(uint32_t *args)
{
#if (CONFIG_SIMULATOR == 1)
    printf("command_stepper_tmc5x_home()\n");
#endif
    struct stepper_tmc5x *s = stepper_tmc5x_oid_lookup(args[0]);
    s->homing_report_interval = args[2];
    if (args[1]) {
        uint8_t len = args[3];
        if (len) {
            spidev_transfer(s->spi, 0, len,
                            (uint8_t*)(uintptr_t)args[4]);
        }
        s->timer.func = stepper_tmc5x_homing_event;
        s->timer.waketime = args[1] + s->homing_report_interval;
        sched_add_timer(&s->timer);
    } else {
        stepper_tmc5x_stop(s);
        s->timer.func = stepper_tmc5x_event;
    }
#if (CONFIG_SIMULATOR == 1)
    printf("stepper_tmc5x_home oid %u clock %u interval %u\n", args[0], args[1], args[2]);
#endif
}
DECL_COMMAND(command_stepper_tmc5x_home,
             "stepper_tmc5x_home oid=%c clock=%u interval=%u cmd=%*s");

void
stepper_tmc5x_home_task(void)
{
    if (!sched_check_wake(&home_wake))
        return;

#if (CONFIG_SIMULATOR == 1)
    printf("stepper_tmc5x_home_task()\n");
#endif

    uint32_t value;
    uint8_t poll_cmd[5] = {REG_RAMP_STAT, 0x0, 0x0, 0x0, 0x0};
    uint8_t oid, ready = 0;
    struct stepper_tmc5x *stepper;
    foreach_oid(oid, stepper, command_stepper_tmc5x_config) {
        if (!stepper->homing_report_interval) continue;
        // Send homing feedback...
        spidev_transfer(stepper->spi, 0, 5, poll_cmd);
        spidev_transfer(stepper->spi, 1, 5, poll_cmd);
        memcpy(&value, &poll_cmd[1], 4);
        value = be32_to_cpu(value);
        // b10 = vzero, b1 = status_stop_r, b0 = status_stop_l
        ready = !!(value & 0b10000000011);
        sendf("stepper_tmc5x_home_status oid=%c value=%u ready=%c",
              oid, value, ready);
#if (CONFIG_SIMULATOR == 1)
        printf("stepper_tmc5x_home_task, oid %u, ready %u\n", oid, ready);
#endif
    }
}
DECL_TASK(stepper_tmc5x_home_task);


/*****************************************
 * SHUTDOWN
 ****************************************/

void
stepper_tmc5x_stop(struct stepper_tmc5x *s)
{
#if (CONFIG_SIMULATOR == 1)
        printf("stepper_tmc5x_stop()\n");
#endif
    sched_del_timer(&s->timer);
    s->next_step_time = 0 - TIMER_ADVANCE;
    s->homing_report_interval = 0;
    while (s->first) {
        struct stepper_tmc5x_move *next = s->first->next;
        move_free(s->first);
        s->first = next;
    }
}

void
stepper_tmc5x_shutdown(void)
{
    uint8_t oid;
    struct stepper_tmc5x *s;
    foreach_oid(oid, s, command_stepper_tmc5x_config) {
        s->first = NULL;
        stepper_tmc5x_stop(s);
    }
}
DECL_SHUTDOWN(stepper_tmc5x_shutdown);


/*****************************************
 * PUBLIC
 ****************************************/

// Return the 'struct stepper_tmc5x' for a given stepper oid
struct stepper_tmc5x *
stepper_tmc5x_oid_lookup(uint8_t oid)
{
#if (CONFIG_SIMULATOR == 1)
    printf("stepper_tmc5x_oid_lookup(oid=%u)\n", oid);
#endif
    return oid_lookup(oid, command_stepper_tmc5x_config);
}

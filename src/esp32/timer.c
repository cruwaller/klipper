#include "board/irq.h"

#include <esp_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/timers.h>
#include <freertos/semphr.h>
#include <soc/timer_group_struct.h>
#include <driver/periph_ctrl.h>
#include <driver/timer.h>
#include <esp_timer.h>

#include "autoconf.h"
#include "command.h" // DECL_SHUTDOWN
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "board/timer_irq.h" // timer_dispatch_many
#include "board/internal.h"
#include "sched.h" // DECL_INIT

/********************************************************************************

  - TIMER GROUP0 is used by FreeRTOS kernel
  - TIMER GROUP1 is used for Klipper
  *    TIMER0 -> event timer
  *    TIMER1 -> watchdog

  - FreeRTOS SW timer for non critical events
  *    1ms period is minimum

 ********************************************************************************/


/********************************************************************************
 *                                   PRIVATE
 ********************************************************************************/
#define USE_HIGH_PRIO_TIMER_TASK 1

#if (USE_HIGH_PRIO_TIMER_TASK)
static SemaphoreHandle_t sem_ptr = NULL;
#endif

uint64_t IRAM_ATTR timer_read_time_isr(void) {
    TIMERG1.hw_timer[TIMER_0].update = 1;
    return (((uint64_t) TIMERG1.hw_timer[TIMER_0].cnt_high << 32) |
            TIMERG1.hw_timer[TIMER_0].cnt_low);
}

/****************************************************************
 * event timer ISR handler
 ****************************************************************/
static void IRAM_ATTR timer_isr(void * arg) {
    int const timer_idx = (int)arg;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t intr_status = TIMERG1.int_st_timers.val;
    TIMERG1.hw_timer[timer_idx].update = 1;

    /*uint64_t timer_counter_value =
        ((uint64_t) TIMERG1.hw_timer[timer_idx].cnt_high) << 32
        | TIMERG1.hw_timer[timer_idx].cnt_low*/;

    /* Clear the interrupt and update the alarm time for the timer with without reload */
    if (intr_status & BIT(timer_idx)) {
        TIMERG1.int_clr_timers.t0 = 1;

#if (!USE_HIGH_PRIO_TIMER_TASK)
        /* Just dispatch all configured timer events */
        uint64_t timer_counter_value = timer_dispatch_many();

        /* Set next ISR timeout */
        TIMERG1.hw_timer[timer_idx].alarm_high = (uint32_t)(timer_counter_value >> 32);
        TIMERG1.hw_timer[timer_idx].alarm_low = (uint32_t)timer_counter_value;
#else
        if (sem_ptr) {
            /* Post sem to trigger high prio task */
            portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(sem_ptr, &xHigherPriorityTaskWoken);
            if (xHigherPriorityTaskWoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
#endif
    }

#if (!USE_HIGH_PRIO_TIMER_TASK)
    /* After the alarm has been triggered we need enable it again,
       so it is triggered the next time */
    TIMERG1.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;
#endif
}

/****************************************************************
 * High prio timer task
 ****************************************************************/
#if (USE_HIGH_PRIO_TIMER_TASK)
static void
timer_dispatch_task(void * arg) {
    (void)arg;
    uint64_t next_alarm;
    uint32_t next_req, diff;
    sem_ptr = xSemaphoreCreateBinary();
    xSemaphoreTake(sem_ptr, 0);
    for (;;) {
        if (xSemaphoreTake(sem_ptr, portMAX_DELAY) != pdTRUE)
            shutdown("Timer SemaphoreTake Failure");
        /* Just dispatch all configured timer events */
        next_req = timer_dispatch_many();
        /* Calc next alarm time */
        timer_get_counter_value(TIMER_GROUP_1, TIMER_0, &next_alarm);
        diff = next_req - (uint32_t)next_alarm;
        next_alarm += unlikely(!diff) ? 0x100000000 : diff;
        /* Set alarm time and enable */
        timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, next_alarm);
        timer_set_alarm(TIMER_GROUP_1, TIMER_0, TIMER_ALARM_EN);
    }
    vSemaphoreDelete(sem_ptr);
    vTaskDelete(NULL);
}
#endif

/****************************************************************
 * watchdog handler
 ****************************************************************/
static void IRAM_ATTR watchdog_isr(void * arg) {
    int const timer_idx = (int)arg;
    uint32_t intr_status = TIMERG1.int_st_timers.val;
    TIMERG1.hw_timer[timer_idx].update = 1;
    TIMERG1.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_DIS;
    if (intr_status & BIT(timer_idx)) {
        TIMERG1.int_clr_timers.t1 = 1;
        timer_pause(TIMER_GROUP_1, timer_idx);
        esp_restart(); // Just call reset!
    }
}

/********************************************************************************
 *                                   PUBLIC
 ********************************************************************************/

// Return the current time (in absolute clock ticks).
uint32_t timer_read_time(void) {
    uint64_t val;
    timer_get_counter_value(TIMER_GROUP_1, TIMER_0, &val);
    return val;
}

// Activate timer dispatch as soon as possible
void timer_kick(void) {
    // Set the next irq time
    uint64_t value;
    timer_get_counter_value(TIMER_GROUP_1, TIMER_0, &value);
    value += timer_from_us(50);
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, value);
#if (USE_HIGH_PRIO_TIMER_TASK)
    timer_set_alarm(TIMER_GROUP_1, TIMER_0, TIMER_ALARM_EN);
#endif
}

void klipper_timer_init(void* arg) {
    (void)arg;
    uint32_t delay = 0;

#if (USE_HIGH_PRIO_TIMER_TASK)
    /* Create a high prio task first */
    xTaskCreatePinnedToCore(
            timer_dispatch_task, "TimerTask",
            4096, NULL, TASK_PRIO_TIMER, NULL, 1);
#endif

    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    // TIMER_BASE_CLK = APB_CLK_FREQ = 80MHz
    config.divider     = (APB_CLK_FREQ / CONFIG_CLOCK_FREQ);
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en  = TIMER_PAUSE;
    config.alarm_en    = TIMER_ALARM_EN;
    config.intr_type   = TIMER_INTR_LEVEL;
    config.auto_reload = TIMER_AUTORELOAD_DIS;
    timer_init(TIMER_GROUP_1, TIMER_0, &config);

    //printf("Timer init ok: clk %u (div %u) = %u (req %u)\n",
    //       TIMER_BASE_CLK, config.divider, (TIMER_BASE_CLK / config.divider), CONFIG_CLOCK_FREQ);
    while(delay++ < 1000000);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_kick();
    timer_enable_intr(TIMER_GROUP_1, TIMER_0);
    timer_isr_register(
            TIMER_GROUP_1, TIMER_0, timer_isr,
            (void *)TIMER_0, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_1, TIMER_0);

    /* Config timer to reset system if no message received from master */
    config.divider = (APB_CLK_FREQ / 1000000); // us timer
    timer_init(TIMER_GROUP_1, TIMER_1, &config);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_1, 30000000 ); // default to 30s
    timer_enable_intr(TIMER_GROUP_1, TIMER_1);
    timer_isr_register(
            TIMER_GROUP_1, TIMER_1, watchdog_isr,
            (void *)TIMER_1, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_1, TIMER_1);
}
DECL_INIT(klipper_timer_init);

void timer_reset(void) {
    // timer reload....
    timer_pause(TIMER_GROUP_1, TIMER_0);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0x00000000ULL);
    timer_start(TIMER_GROUP_1, TIMER_0);
    timer_kick();
}
DECL_SHUTDOWN(timer_reset);


#if (IRQ_STOP_TMR)
/****************************************************************
 * IRQs
 ****************************************************************/
void irq_disable(void)
{
    timer_set_alarm(TIMER_GROUP_1, TIMER_0, TIMER_ALARM_DIS);
}

void irq_enable(void)
{
    timer_set_alarm(TIMER_GROUP_1, TIMER_0, TIMER_ALARM_EN);
}
#endif

/****************************************************************
 * watchdog reset
 ****************************************************************/
void wd_reset(void) {
    timer_pause(TIMER_GROUP_1, TIMER_1);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0x00000000ULL);
    timer_start(TIMER_GROUP_1, TIMER_1);
}

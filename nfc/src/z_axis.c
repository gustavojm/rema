#include "z_axis.h"
#include "mot_pap.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "board.h"

#include "debug.h"
#include "tmr.h"
#include "gpio.h"
#include "kp.h"

#define Z_AXIS_TASK_PRIORITY ( configMAX_PRIORITIES - 1 )
#define Z_AXIS_SUPERVISOR_TASK_PRIORITY ( configMAX_PRIORITIES - 3)

SemaphoreHandle_t z_axis_supervisor_semaphore;

struct mot_pap z_axis;

/**
 * @brief   handles the X axis movement.
 * @param   par     : unused
 * @returns never
 * @note    Receives commands from z_axis_queue
 */
static void z_axis_task(void *par) {
    while (true) {
        mot_pap_task(&z_axis);
    }
}

/**
 * @brief   checks if stalled and if position reached in closed loop.
 * @param   par : unused
 * @returns never
 */
static void z_axis_supervisor_task(void *par) {
    while (true) {
        xSemaphoreTake(z_axis.supervisor_semaphore, portMAX_DELAY);
        mot_pap_supervise(&z_axis);
    }
}

/**
 * @brief   creates the queues, semaphores and endless tasks to handle Y axis movements.
 * @returns nothing
 */
void z_axis_init() {
    z_axis.queue = xQueueCreate(5, sizeof(struct mot_pap_msg*));

    z_axis.name = "z_axis";
    z_axis.type = MOT_PAP_TYPE_STOP;
    z_axis.counts_to_inch_factor = (double) 1 / 1000000;
    z_axis.half_pulses = 0;
    z_axis.pos_act = 0;

    z_axis.gpios.direction =
            (struct gpio_entry ) { 4, 6, SCU_MODE_FUNC0, 2, 5 };                    //DOUT2 P4_6    PIN08   GPIO2[5]   Z_AXIS_DIR
    z_axis.gpios.step = (struct gpio_entry ) { 4, 10, SCU_MODE_FUNC4, 5, 14 };      //DOUT6 P4_10   PIN35   GPIO5[14]  Z_AXIS_STEP

    gpio_init_output(z_axis.gpios.direction);
    gpio_init_output(z_axis.gpios.step);

    z_axis.tmr.started = false;
    z_axis.tmr.lpc_timer = LPC_TIMER3;
    z_axis.tmr.rgu_timer_rst = RGU_TIMER3_RST;
    z_axis.tmr.clk_mx_timer = CLK_MX_TIMER3;
    z_axis.tmr.timer_IRQn = TIMER3_IRQn;

    tmr_init(&z_axis.tmr);

    kp_init(&z_axis.kp, 100,                        //!< Kp
            KP_DIRECT,                              //!< Control type
            z_axis.step_time,                       //!< Update rate (ms)
            -100000,                                //!< Min output
            100000,                                 //!< Max output
            10000                                   //!< Absolute Min output
            );

    z_axis.supervisor_semaphore = xSemaphoreCreateBinary();

    if (z_axis.supervisor_semaphore != NULL) {
        // Create the 'handler' task, which is the task to which interrupt processing is deferred
        xTaskCreate(z_axis_supervisor_task, "Z_AXIS supervisor", 2048,
        NULL, Z_AXIS_SUPERVISOR_TASK_PRIORITY, NULL);
        lDebug(Info, "z_axis: supervisor task created");
    }

    xTaskCreate(z_axis_task, "Z_AXIS", 512, NULL, Z_AXIS_TASK_PRIORITY, NULL);

    lDebug(Info, "z_axis: task created");
}

/**
 * @brief   handle interrupt from 32-bit timer to generate pulses for the stepper motor drivers
 * @returns nothing
 */
void TIMER3_IRQHandler(void) {
    if (tmr_match_pending(&(z_axis.tmr))) {
        mot_pap_isr(&z_axis);
    }
}

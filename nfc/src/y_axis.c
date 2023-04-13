#include "y_axis.h"
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

#define Y_AXIS_TASK_PRIORITY ( configMAX_PRIORITIES - 1 )
#define Y_AXIS_SUPERVISOR_TASK_PRIORITY ( configMAX_PRIORITIES - 3)

SemaphoreHandle_t y_axis_supervisor_semaphore;

struct mot_pap y_axis;

/**
 * @brief   handles the X axis movement.
 * @param   par     : unused
 * @returns never
 * @note    Receives commands from y_axis_queue
 */
static void y_axis_task(void *par) {
    while (true) {
        mot_pap_task(&y_axis);
    }
}

/**
 * @brief   checks if stalled and if position reached in closed loop.
 * @param   par : unused
 * @returns never
 */
static void y_axis_supervisor_task(void *par) {
    while (true) {
        xSemaphoreTake(y_axis.supervisor_semaphore, portMAX_DELAY);
        mot_pap_supervise(&y_axis);
    }
}

/**
 * @brief 	creates the queues, semaphores and endless tasks to handle Y axis movements.
 * @returns	nothing
 */
void y_axis_init() {
    y_axis.queue = xQueueCreate(5, sizeof(struct mot_pap_msg*));

    y_axis.name = "y_axis";
    y_axis.type = MOT_PAP_TYPE_STOP;
    y_axis.reversed = false;
    y_axis.inches_to_counts_factor = 1000;
    y_axis.half_pulses = 0;
    y_axis.pos_act = 0;

    y_axis.gpios.direction =
            (struct gpio_entry ) { 4, 5, SCU_MODE_FUNC0, 2, 5 };	            //DOUT1 P4_5 	PIN10  	GPIO2[5]   Y_AXIS_DIR
    y_axis.gpios.step = (struct gpio_entry ) { 4, 9, SCU_MODE_FUNC4, 5, 13 };   //DOUT5 P4_9 	PIN33  	GPIO5[13]  Y_AXIS_STEP

    gpio_init_output(y_axis.gpios.direction);
    gpio_init_output(y_axis.gpios.step);

    y_axis.tmr.started = false;
    y_axis.tmr.lpc_timer = LPC_TIMER2;
    y_axis.tmr.rgu_timer_rst = RGU_TIMER2_RST;
    y_axis.tmr.clk_mx_timer = CLK_MX_TIMER2;
    y_axis.tmr.timer_IRQn = TIMER2_IRQn;

    tmr_init(&y_axis.tmr);

    kp_init(&y_axis.kp, 100,                        //!< Kp
            KP_DIRECT,                              //!< Control type
            y_axis.step_time,                       //!< Update rate (ms)
            -100000,                                //!< Min output
            100000,                                 //!< Max output
            10000                                   //!< Absolute Min output
            );

    y_axis.supervisor_semaphore = xSemaphoreCreateBinary();

    if (y_axis.supervisor_semaphore != NULL) {
        // Create the 'handler' task, which is the task to which interrupt processing is deferred
        xTaskCreate(y_axis_supervisor_task, "Y_AXIS supervisor", 256,
        NULL, Y_AXIS_SUPERVISOR_TASK_PRIORITY, NULL);
        lDebug(Info, "y_axis: supervisor task created");
    }

    xTaskCreate(y_axis_task, "Y_AXIS", 256, NULL, Y_AXIS_TASK_PRIORITY, NULL);

    lDebug(Info, "y_axis: task created");
}

/**
 * @brief	handle interrupt from 32-bit timer to generate pulses for the stepper motor drivers
 * @returns	nothing
 */
void TIMER2_IRQHandler(void) {
    if (tmr_match_pending(&(y_axis.tmr))) {
        mot_pap_isr(&y_axis);
    }
}

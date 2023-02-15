#include "x_axis.h"
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

#define X_AXIS_TASK_PRIORITY ( configMAX_PRIORITIES - 1 )
#define X_AXIS_SUPERVISOR_TASK_PRIORITY ( configMAX_PRIORITIES - 3)

SemaphoreHandle_t x_axis_supervisor_semaphore;

struct mot_pap x_axis;

/**
 * @brief 	handles the X axis movement.
 * @param 	par		: unused
 * @returns	never
 * @note	Receives commands from x_axis_queue
 */
static void x_axis_task(void *par)
{
	struct mot_pap_msg *msg_rcv;

	while (true) {
		if (xQueueReceive(x_axis.queue, &msg_rcv, portMAX_DELAY) == pdPASS) {
			lDebug(Info, "x_axis: command received");

			x_axis.stalled = false; 		// If a new command was received, assume we are not stalled
			x_axis.stalled_counter = 0;
			x_axis.already_there = false;
			x_axis.step_time = 100;

			//mot_pap_read_corrected_pos(&x_axis);

			switch (msg_rcv->type) {
			case MOT_PAP_TYPE_FREE_RUNNING:
				mot_pap_move_free_run(&x_axis, msg_rcv->free_run_direction,
						msg_rcv->free_run_speed);
				break;

			case MOT_PAP_TYPE_CLOSED_LOOP:
				mot_pap_move_closed_loop(&x_axis, msg_rcv->closed_loop_setpoint);
				break;

			case MOT_PAP_TYPE_STEPS:
				mot_pap_move_steps(msg_rcv->axis, msg_rcv->free_run_direction,
						msg_rcv->free_run_speed, msg_rcv->steps, 200, 50);
				break;

			default:
				mot_pap_stop(&x_axis);
				break;
			}

			vPortFree(msg_rcv);
			msg_rcv = NULL;
		}
	}
}

/**
 * @brief	checks if stalled and if position reached in closed loop.
 * @param 	par	: unused
 * @returns	never
 */
static void x_axis_supervisor_task(void *par)
{
	while (true) {
		xSemaphoreTake(x_axis.supervisor_semaphore, portMAX_DELAY);
		mot_pap_supervise(&x_axis);
	}
}

void x_axis_init() {
	x_axis.queue = xQueueCreate(5, sizeof(struct mot_pap_msg*));

	x_axis.name = "x_axis";
	x_axis.type = MOT_PAP_TYPE_STOP;
	x_axis.counts_to_inch_factor = (double) 1 / 1000000;
	x_axis.half_pulses = 0;
	x_axis.pos_act = 0;

	x_axis.gpios.direction =
			(struct gpio_entry ) { 2, 1, SCU_MODE_FUNC4, 5, 1 };	//DOUT0 P2_1 	PIN81  	GPIO5[1]   X_AXIS_DIR
	x_axis.gpios.step = (struct gpio_entry ) { 4, 8, SCU_MODE_FUNC4, 5, 12 };//DOUT4 P4_8 	PIN15  	GPIO5[12]  X_AXIS_STEP

	gpio_init_output(x_axis.gpios.direction);
	gpio_init_output(x_axis.gpios.step);

	x_axis.tmr.started = false;
	x_axis.tmr.lpc_timer = LPC_TIMER1;
	x_axis.tmr.rgu_timer_rst = RGU_TIMER1_RST;
	x_axis.tmr.clk_mx_timer = CLK_MX_TIMER1;
	x_axis.tmr.timer_IRQn = TIMER1_IRQn;

	tmr_init(&x_axis.tmr);

	kp_init(&x_axis.kp,100,						//!< Kp
			KP_DIRECT,								//!< Control type
			x_axis.step_time,						//!< Update rate (ms)
			-100000,								//!< Min output
			100000,									//!< Max output
			10000									//!< Absolute Min output
			);

	x_axis.supervisor_semaphore = xSemaphoreCreateBinary();

	if (x_axis.supervisor_semaphore != NULL) {
		// Create the 'handler' task, which is the task to which interrupt processing is deferred
		xTaskCreate(x_axis_supervisor_task, "XAxisSupervisor",
		2048,
		NULL, X_AXIS_SUPERVISOR_TASK_PRIORITY, NULL);
		lDebug(Info, "x_axis: supervisor task created");
	}

	xTaskCreate(x_axis_task, "X_AXIS", 512, NULL,
	X_AXIS_TASK_PRIORITY, NULL);

	lDebug(Info, "x_axis: task created");
}

/**
 * @brief	handle interrupt from 32-bit timer to generate pulses for the stepper motor drivers
 * @returns	nothing
 */
void TIMER1_IRQHandler(void) {
	if (tmr_match_pending(&(x_axis.tmr))) {
		mot_pap_isr(&x_axis);
	}
}

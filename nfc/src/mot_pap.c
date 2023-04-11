#include "mot_pap.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

#include "misc.h"
#include "debug.h"
#include "rema.h"
#include "tmr.h"

void mot_pap_task(struct mot_pap *me)
{
    struct mot_pap_msg *msg_rcv;
    if (xQueueReceive(me->queue, &msg_rcv, portMAX_DELAY) == pdPASS) {
        lDebug(Info, "%s: command received", me->name);

        me->stalled = false;         // If a new command was received, assume we are not stalled
        me->stalled_counter = 0;
        me->already_there = false;
        me->step_time = 100;

        //mot_pap_read_corrected_pos(&x_axis);

        switch (msg_rcv->type) {
        case MOT_PAP_TYPE_FREE_RUNNING:
            mot_pap_move_free_run(me, msg_rcv->free_run_direction,
                    msg_rcv->free_run_speed);
            break;

        case MOT_PAP_TYPE_CLOSED_LOOP:
            mot_pap_move_closed_loop(me, msg_rcv->closed_loop_setpoint * me->inches_to_counts_factor);
            break;

        default:
            mot_pap_stop(me);
            break;
        }

        vPortFree(msg_rcv);
        msg_rcv = NULL;
    }
}


/**
 * @brief	returns the direction of movement depending if the error is positive or negative
 * @param 	error : the current position error in closed loop positioning
 * @returns	MOT_PAP_DIRECTION_CW if error is positive
 * @returns	MOT_PAP_DIRECTION_CCW if error is negative
 */
enum mot_pap_direction mot_pap_direction_calculate(int32_t error)
{
	return error < 0 ? MOT_PAP_DIRECTION_CCW : MOT_PAP_DIRECTION_CW;
}

/**
 * @brief	if allowed, starts a free run movement
 * @param 	me			: struct mot_pap pointer
 * @param 	direction	: either MOT_PAP_DIRECTION_CW or MOT_PAP_DIRECTION_CCW
 * @param 	speed		: integer from 0 to 8
 * @returns	nothing
 */
void mot_pap_move_free_run(struct mot_pap *me, enum mot_pap_direction direction,
		int speed)
{
	if (speed < MOT_PAP_MAX_FREQ) {
		me->stalled = false; // If a new command was received, assume we are not stalled
		me->stalled_counter = 0;
		me->already_there = false;

		if ((me->dir != direction) && (me->type != MOT_PAP_TYPE_STOP)) {
			tmr_stop(&(me->tmr));
			vTaskDelay(pdMS_TO_TICKS(MOT_PAP_DIRECTION_CHANGE_DELAY_MS));
		}

		me->type = MOT_PAP_TYPE_FREE_RUNNING;
		me->dir = direction;
		gpio_set_pin_state(me->gpios.direction, me->dir);
		me->requested_freq = speed;

		tmr_stop(&(me->tmr));
		tmr_set_freq(&(me->tmr), me->requested_freq);
		tmr_start(&(me->tmr));
		lDebug(Info, "%s: FREE RUN, speed: %i, direction: %s", me->name,
				me->requested_freq,
				me->dir == MOT_PAP_DIRECTION_CW ? "CW" : "CCW");
	} else {
		lDebug(Warn, "%s: chosen speed out of bounds %i", me->name, speed);
	}
}

/**
 * @brief	if allowed, starts a closed loop movement
 * @param 	me			: struct mot_pap pointer
 * @param 	setpoint	: the resolver value to reach
 * @returns	nothing
 */
void mot_pap_move_closed_loop(struct mot_pap *me, int setpoint)
{
	int32_t error;
	enum mot_pap_direction dir;
	me->stalled = false; // If a new command was received, assume we are not stalled
	me->stalled_counter = 0;

	me->pos_cmd = setpoint;
	lDebug(Info, "%s: CLOSED_LOOP posCmd: %i posAct: %i", me->name, me->pos_cmd,
			me->pos_act);

	//calculate position error
	error = me->pos_cmd - me->pos_act;
	me->already_there = (abs(error) < MOT_PAP_POS_THRESHOLD);

	if (me->already_there) {
		tmr_stop(&(me->tmr));
		lDebug(Info, "%s: already there", me->name);
	} else {
		kp_restart(&(me->kp), me->pos_act);

		int out = kp_run(&(me->kp), me->pos_cmd, me->pos_act);

		dir = mot_pap_direction_calculate(out);
		if ((me->dir != dir) && (me->type != MOT_PAP_TYPE_STOP)) {
			tmr_stop(&(me->tmr));
			vTaskDelay(pdMS_TO_TICKS(MOT_PAP_DIRECTION_CHANGE_DELAY_MS));
		}
		me->type = MOT_PAP_TYPE_CLOSED_LOOP;
		me->dir = dir;
		gpio_set_pin_state(me->gpios.direction, me->dir);
		me->requested_freq = abs(out);
		tmr_stop(&(me->tmr));
		tmr_set_freq(&(me->tmr), me->requested_freq);
		tmr_start(&(me->tmr));
	}
}

/**
 * @brief	if there is a movement in process, stops it
 * @param 	me	: struct mot_pap pointer
 * @returns	nothing
 */
void mot_pap_stop(struct mot_pap *me)
{
	me->type = MOT_PAP_TYPE_STOP;
	tmr_stop(&(me->tmr));
	lDebug(Info, "%s: STOP", me->name);
}

/**
 * @brief 	supervise motor movement for stall or position reached in closed loop
 * @param 	me			: struct mot_pap pointer
 * @returns nothing
 * @note	to be called by the deferred interrupt task handler
 */
void mot_pap_supervise(struct mot_pap *me)
{
	while (true) {

		if (xSemaphoreTake(me->supervisor_semaphore,
				portMAX_DELAY) == pdPASS) {
			if (rema_stall_control_get()) {
				if (abs(
						(int) (me->pos_act - me->last_pos)) < MOT_PAP_STALL_THRESHOLD) {

					me->stalled_counter++;
					if (me->stalled_counter >= MOT_PAP_STALL_MAX_COUNT) {
						me->stalled = true;
						tmr_stop(&(me->tmr));
						lDebug(Warn, "%s: stalled", me->name);
						rema_control_enabled_set(false);
						goto end;
					}
				} else {
					me->stalled_counter = 0;
				}
			}

			if (me->already_there) {
				lDebug(Info, "%s: position reached", me->name);
				goto end;
			}

			if (me->type == MOT_PAP_TYPE_CLOSED_LOOP) {
				int out = kp_run(&(me->kp), me->pos_cmd, me->pos_act);
				lDebug(Info, "Control output = %i: ", out);

				enum mot_pap_direction dir = mot_pap_direction_calculate(out);
				if ((me->dir != dir) && (me->type != MOT_PAP_TYPE_STOP)) {
					tmr_stop(&(me->tmr));
					vTaskDelay(
							pdMS_TO_TICKS(MOT_PAP_DIRECTION_CHANGE_DELAY_MS));
				}
				me->dir = dir;
				gpio_set_pin_state(me->gpios.direction, me->dir);
				me->requested_freq = abs(out);
				tmr_stop(&(me->tmr));
				tmr_set_freq(&(me->tmr), me->requested_freq);
				tmr_start(&(me->tmr));
			}

		}
	}
	end: ;
}

/**
 * @brief 	function called by the timer ISR to generate the output pulses
 * @param 	me : struct mot_pap pointer
 */
void mot_pap_isr(struct mot_pap *me)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	int error;
	if (me->type == MOT_PAP_TYPE_CLOSED_LOOP) {
		error = me->pos_cmd - me->pos_act;
		me->already_there = (abs((int) error) < 2);
	}

	if (me->already_there) {
		me->type = MOT_PAP_TYPE_STOP;
		tmr_stop(&(me->tmr));
		xSemaphoreGiveFromISR(me->supervisor_semaphore,
				&xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		goto cont;
	}

	++me->half_steps_curr;

	gpio_toggle(me->gpios.step);

	int ticks_now = xTaskGetTickCount();
	if ((ticks_now - me->ticks_last_time) > pdMS_TO_TICKS(me->step_time)) {

		me->ticks_last_time = ticks_now;
		xSemaphoreGiveFromISR(me->supervisor_semaphore,
				&xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}

	cont: me->last_pos = me->pos_act;
}

/**
 * @brief 	updates the current position from encoder
 * @param 	me : struct mot_pap pointer
 */

void mot_pap_update_position(struct mot_pap *me) {
	if (me->dir == MOT_PAP_DIRECTION_CW) {
		me->pos_act ++;
	} else {
		me->pos_act --;
	}
}

/**
 * @brief	sets axis offset
 * @param 	offset		: RDC position for 0 degrees
 * @returns	nothing
 */
void mot_pap_set_offset(struct mot_pap *me, int offset)
{
	me->offset = offset;
}

void mot_pap_set_position(struct mot_pap *me, double pos)
{
	me->pos_act = pos * me->inches_to_counts_factor ;
}


/**
 * @brief	returns status of the X axis task.
 * @returns copy of status structure of the task
 */
struct mot_pap* mot_pap_get_status(struct mot_pap *me)
{
	mot_pap_read_corrected_pos(me);
	return me;
}

JSON_Value* mot_pap_json(struct mot_pap *me)
{
	JSON_Value *ans = json_value_init_object();
	json_object_set_number(json_value_get_object(ans), "pos_cmd", me->pos_cmd);
	json_object_set_number(json_value_get_object(ans), "posAct", me->pos_act / me->inches_to_counts_factor);
	json_object_set_boolean(json_value_get_object(ans), "stalled", me->stalled);
	json_object_set_number(json_value_get_object(ans), "offset", me->offset);
	return ans;
}


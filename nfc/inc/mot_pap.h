#ifndef MOT_PAP_H_
#define MOT_PAP_H_

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "tmr.h"
#include "parson.h"
#include "gpio.h"
#include "kp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOT_PAP_MAX_FREQ						125000
#define MOT_PAP_MIN_FREQ						100
#define MOT_PAP_CLOSED_LOOP_FREQ_MULTIPLIER  	( MOT_PAP_MAX_FREQ / 100 )
#define MOT_PAP_MAX_SPEED_FREE_RUN				8
#define MOT_PAP_COMPUMOTOR_MAX_FREQ				300000
#define MOT_PAP_DIRECTION_CHANGE_DELAY_MS		0

#define MOT_PAP_SUPERVISOR_RATE    				625	//2 means one step
#define MOT_PAP_POS_PROXIMITY_THRESHOLD			100
#define MOT_PAP_POS_THRESHOLD 					6
#define MOT_PAP_STALL_THRESHOLD 				3
#define MOT_PAP_STALL_MAX_COUNT					40

enum mot_pap_direction {
	MOT_PAP_DIRECTION_CW, MOT_PAP_DIRECTION_CCW,
};

enum mot_pap_type {
	MOT_PAP_TYPE_FREE_RUNNING,
	MOT_PAP_TYPE_CLOSED_LOOP,
	MOT_PAP_TYPE_STOP,
	MOT_PAP_TYPE_STEPS
};

/**
 * @struct 	mot_pap_msg
 * @brief	messages to axis tasks.
 */
struct mot_pap_msg {
	enum mot_pap_type type;
	enum mot_pap_direction free_run_direction;
	int free_run_speed;
	float closed_loop_setpoint;
	int steps;
	struct mot_pap *axis;
};

/**
 * @struct 	mot_pap_gpios
 * @brief	pointers to functions to handle GPIO lines of this stepper motor.
 */
struct mot_pap_gpios {
	struct gpio_entry direction;
	struct gpio_entry step;
};

/**
 * @struct 	mot_pap
 * @brief	structure for axis motors.
 */
struct mot_pap {
	char *name;

	enum mot_pap_type type;
	enum mot_pap_direction dir;

	bool already_there;
	bool stalled;
	volatile bool stop;

	volatile int pos_act;
	int pos_cmd;

	int requested_freq;

	int step_time;
	int half_steps_requested;
	int half_steps_curr;
	int half_steps_to_middle;

	int stalled_counter;
	int half_pulses;	// counts steps from the last call to supervisor task
	int offset;

	int ticks_last_time;
	int last_pos;
	int inches_to_counts_factor;

	struct mot_pap_gpios gpios;
	struct tmr tmr;
	QueueHandle_t queue;
	SemaphoreHandle_t supervisor_semaphore;
	struct kp kp;
};

void mot_pap_task(struct mot_pap *axis);

void mot_pap_init();

void mot_pap_supervise(struct mot_pap *me);

uint16_t mot_pap_offset_correction(uint16_t pos, uint16_t offset,
		uint8_t resolution);

void mot_pap_read_corrected_pos(struct mot_pap *me);

void mot_pap_isr_helper_task();

void mot_pap_supervisor_task();

void mot_pap_move_free_run(struct mot_pap *me, enum mot_pap_direction direction,
		int speed);

void mot_pap_move_closed_loop(struct mot_pap *status, int setpoint);

void mot_pap_move_steps(struct mot_pap *me, enum mot_pap_direction direction,
		int speed, int steps, int step_time, int step_amplitude_divider);

void mot_pap_stop(struct mot_pap *me);

void mot_pap_isr(struct mot_pap *me);

void mot_pap_update_position(struct mot_pap *me);

void mot_pap_set_offset(struct mot_pap *me, int offset);

void mot_pap_set_position(struct mot_pap *me, double pos);

uint32_t mot_pap_read_on_condition(void);

JSON_Value* mot_pap_json(struct mot_pap *me);

#ifdef __cplusplus
}
#endif

#endif /* MOT_PAP_H_ */

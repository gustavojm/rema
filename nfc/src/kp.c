#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "board.h"
#include "kp.h"
#include "limits.h"
#include "debug.h"

void kp_init(struct kp *this, int kp,
		enum kp_controller_direction controller_dir,
		double sample_period_ms, int min_output, int max_output, int min_abs_output)
{
	kp_set_output_limits(this, min_output, max_output, min_abs_output);

	this->sample_period_ms = sample_period_ms;

	kp_set_controller_direction(this, controller_dir);

	// Set tunings with provided constants
	kp_set_tunings(this, kp);
	this->prev_input = 0;
	this->prev_output = 0;

	this->p_term = 0.0;
	this->num_times_ran = 0;
}

void kp_restart(struct kp *this, int input) {
	this->prev_input = input;
	this->prev_error = 0;
	this->num_times_ran = 1;
}


int kp_run(struct kp *this, int setpoint, int input) {
#define RAMP_RATE	0.01		//Change the setpoint by at most 0.1 per iteration

//	if (this->num_times_ran == 1) {
//		this->setpoint_jump = abs(setpoint - this->prev_setpoint);
//	}
//
//	if (setpoint > 0 && setpoint > (this->prev_setpoint + (this->setpoint_jump * RAMP_RATE))) {
//		setpoint = this->prev_setpoint + (this->setpoint_jump * RAMP_RATE);
//	} else if (setpoint < 0 && setpoint < (this->prev_setpoint - (this->setpoint_jump * RAMP_RATE))){
//		setpoint = this->prev_setpoint - (this->setpoint_jump * RAMP_RATE);
//	}

	int error = (setpoint - input);

	// PROPORTIONAL CALCS
	this->p_term = this->zp * error;

	this->output = this->p_term;

	// Limit output
	if (this->output > this->out_max)
		this->output = this->out_max;
	else if (this->output < this->out_min)
		this->output = this->out_min;

	// Remember input value to next call
	this->prev_input = input;
	// Remember last output for next call
	this->prev_output = this->output;
	this->prev_error = error;

	// Increment the Run() counter, after checking to make sure it hasn't reached
	// max value.
	if (this->num_times_ran < INT_MAX)
		this->num_times_ran++;

	float attenuation = (this->num_times_ran < 10) ? this->num_times_ran * 0.1 : 1;
	int out = this->output * attenuation;
	if (out >= 0 && out < this->out_abs_min) {
		return this->out_abs_min;
	}
	if (out < 0 && out > -this->out_abs_min) {
		return -this->out_abs_min;
	}
	return out;
}

//! @brief		Sets the KP tunings.
//! @warning	Make sure samplePeriodMs is set before calling this funciton.
void kp_set_tunings(struct kp *this, int kp) {
	if (kp < 0)
		return;

	this->kp = kp;

	// Calculate time-step-scaled KP terms
	this->zp = kp;

	if (this->controller_dir == KP_REVERSE) {
		this->zp = (0 - this->zp);

	}

	lDebug(Info, "ZP: %f", this->zp);
}

int kp_get_kp(struct kp *this) {
	return this->kp;
}


int kp_get_zp(struct kp *this) {
	return this->zp;
}


void kp_set_sample_period(struct kp *this, unsigned int new_sample_period_ms) {
	if (new_sample_period_ms > 0) {
		int ratio = (int) new_sample_period_ms / (double) this->sample_period_ms;
		this->sample_period_ms = new_sample_period_ms;
	}
}

void kp_set_output_limits(struct kp *this, int min, int max, int min_abs) {
	this->out_min = min;
	this->out_max = max;
	this->out_abs_min = min_abs;
}

void kp_set_controller_direction(struct kp *this, enum kp_controller_direction controller_dir) {
	if (controller_dir != this->controller_dir) {
		// Invert control constants
		this->zp = (0 - this->zp);
	}
	this->controller_dir = controller_dir;
}

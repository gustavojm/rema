#ifndef KP_H_
#define KP_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//! @brief		Enumerates the controller direction modes
enum kp_controller_direction {
	KP_DIRECT,			//!< Direct drive (+error gives +output)
	KP_REVERSE			//!< Reverse driver (+error gives -output)
};

//! @brief 		Error proportional -kp- control for frequency output.
struct kp {
	//! @brief 		The set-point the error proportional -kp- control is trying to make the output converge to.
	int setpoint;

	//! @brief		The control output.
	//! @details	This is updated when kp_run() is called.
	int output;

	//! @brief		Time-step scaled proportional constant for quick calculation (equal to actualKp)
	float zp;

	//! @brief		Actual (non-scaled) proportional constant
	int kp;

	//! @brief		Actual (non-scaled) proportional constant
	int prev_input;

	//! @brief		The change in input between the current and previous value
	int input_change;

	//! @brief 		The output value calculated the previous time Pid_Run() was called.
	//! @details	Used in ACCUMULATE_OUTPUT mode.
	int prev_output;
	int prev_error;

	//! @brief		The sample period (in milliseconds) between successive Pid_Run() calls.
	//! @details	The constants with the z prefix are scaled according to this value.
	int sample_period_ms;

	float p_term;		//!< The proportional term that is summed as part of the output (calculated in Pid_Run())
	int out_min;	//!< The minimum output value. Anything lower will be limited to this floor.
	int out_max;	//!< The maximum output value. Anything higher will be limited to this ceiling.
	int out_abs_min;	//!< The absolute minimum output value. Anything lower will be limited to this floor.

	//! @brief		Counts the number of times that Run() has be called. Used to stop
	//!				derivative control from influencing the output on the first call.
	//! @details	Safely stops counting once it reaches 2^32-1 (rather than overflowing).
	int num_times_ran;

	//! @brief		The controller direction (FORWARD or REVERSE).
	enum kp_controller_direction controller_dir;
};

//! @brief 		Init function
//! @details   	The parameters specified here are those for for which we can't set up
//!    			reliable defaults, so we need to have the user set them.
void kp_init(struct kp *this, int ki,
		enum kp_controller_direction controller_dir,
		double sample_period_ms, int min_output, int max_output, int min_abs_output);

void kp_restart(struct kp *this, int input);

//! @brief 		Computes new KP values
//! @details 	Call once per sampleTimeMs. Output is stored in the kpData structure.
int kp_run(struct kp *this, float setpoint, float input);

void kp_set_output_limits(struct kp *this, int min, int max, int min_abs_output);

//! @details	The KP will either be connected to a direct acting process (+error leads to +output, aka inputs are positive)
//!				or a reverse acting process (+error leads to -output, aka inputs are negative)
void kp_set_controller_direction(struct kp *this, enum kp_controller_direction controller_dir);

//! @brief		Changes the sample time
void kp_set_sample_period(struct kp *this, unsigned int new_sample_period_ms);

//! @brief		This function allows the controller's dynamic performance to be adjusted.
//! @details	It's called automatically from the init function, but tunings can also
//! 			be adjusted on the fly during normal operation
void kp_set_tunings(struct kp *this, int kp);

//! @brief		Returns the actual (not time-scaled) proportional constant.
int kp_get_kp(struct kp *this);

//! @brief		Returns the time-scaled (dependent on sample period) proportional constant.
int kp_get_zp(struct kp *this);


#ifdef __cplusplus
}
#endif

#endif /* KP_H_ */

#ifndef REMA_H_
#define REMA_H_

#include <relay.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


extern volatile bool control_enabled;
extern volatile bool stall_detection;

static inline void rema_control_enabled_set(bool status) {
	control_enabled = status;
	relay_main_pwr(status);
}

static inline bool rema_control_enabled_get() {
	return control_enabled;
}

static inline void rema_stall_control_set(bool status) {
	stall_detection = status;
}

static inline bool rema_stall_control_get() {
	return stall_detection;
}


#ifdef __cplusplus
}
#endif

#endif /* REMA_H_ */


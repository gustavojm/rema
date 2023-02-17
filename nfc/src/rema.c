#include <stdbool.h>
#include "rema.h"

static bool control_enabled = false;
static bool stall_detection = true;

inline void rema_control_enabled_set(bool status) {
	control_enabled = status;
	relay_main_pwr(status);
}

inline bool rema_control_enabled_get() {
	return control_enabled;
}

inline void rema_stall_control_set(bool status) {
	stall_detection = status;
}

inline bool rema_stall_control_get() {
	return stall_detection;
}


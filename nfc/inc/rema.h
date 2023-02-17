#ifndef REMA_H_
#define REMA_H_

#include <relay.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


void rema_control_enabled_set(bool status);

bool rema_control_enabled_get();

void rema_stall_control_set(bool status);

bool rema_stall_control_get();


#ifdef __cplusplus
}
#endif

#endif /* REMA_H_ */


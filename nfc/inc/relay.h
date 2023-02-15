#ifndef RELAY_H_
#define RELAY_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void relay_init();

void relay_main_pwr(bool state);

bool relay_main_pwr_state();

void relay_spare_led(bool state);

#ifdef __cplusplus
}
#endif

#endif /* RELAY_H_ */

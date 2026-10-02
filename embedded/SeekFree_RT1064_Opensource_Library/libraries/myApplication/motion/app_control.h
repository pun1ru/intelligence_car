#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdint.h>
#include "app_attitude.h"
#include "app_state.h"

uint8_t app_control_step(const app_state_t *state,
                         const app_attitude_t *attitude);

#endif

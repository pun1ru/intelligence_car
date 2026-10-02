#ifndef STATE_TASK_H
#define STATE_TASK_H

#include <stdint.h>
#include "app_state.h"

void state_task(void *pvParameters);
uint8_t state_task_get_snapshot(app_state_t *state);
void state_task_request_protection(void);

#endif

#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdint.h>
#include "app_control.h"

uint8_t control_task_get_snapshot(app_control_telemetry_t *telemetry);

void control_task(void *pvParameters);

#endif

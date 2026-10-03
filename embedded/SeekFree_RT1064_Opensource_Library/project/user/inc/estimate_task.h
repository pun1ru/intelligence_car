#ifndef ESTIMATE_TASK_H
#define ESTIMATE_TASK_H

#include <stdint.h>

#define ESTIMATE_REQUEST_CALIBRATE 1U
#define ESTIMATE_REQUEST_RESET     2U
#define ESTIMATE_REQUEST_CANCEL    4U
#define ESTIMATE_REQUEST_GYRO_BIAS 8U

void estimate_task(void *pvParameters);
void estimate_task_request(uint32_t request);

#endif

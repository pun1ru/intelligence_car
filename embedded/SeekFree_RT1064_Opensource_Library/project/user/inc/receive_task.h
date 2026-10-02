#ifndef RECEIVE_TASK_H_
#define RECEIVE_TASK_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "app_attitude.h"
#include "app_sensor.h"
#include "semphr.h"

extern SemaphoreHandle_t g_imu_data_ready_sem;

void receive_task(void *pvParameters);

uint8_t receive_data_init(void);
uint8_t receive_imu_snapshot(app_imu_sample_t *sample);
uint8_t receive_attitude_snapshot(app_attitude_t *attitude);
uint8_t receive_display_snapshot(app_attitude_t *attitude,
                                 app_encoder_sample_t *encoder);
void receive_publish_attitude(const app_attitude_t *attitude);

/* Blocking calibration; call from a task other than receive_task. */
void imu_gyro_noise_calibrate(void);
void imu_accel_noise_calibrate(void);

#endif

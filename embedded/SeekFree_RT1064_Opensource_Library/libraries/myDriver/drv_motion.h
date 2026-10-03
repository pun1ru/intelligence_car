#ifndef DRV_MOTION_H
#define DRV_MOTION_H

#include <stdint.h>

void drv_motion_init(uint32_t key_period_ms);
void drv_motion_set_signed(int32_t left_pwm, int32_t right_pwm);
void drv_motion_get_signed(int16_t *left_pwm, int16_t *right_pwm);
void drv_motion_read_encoder(int16_t *left_count, int16_t *right_count);
/* Returns counts accumulated since the previous delta read, including timer wrap. */
void drv_motion_read_encoder_delta(int16_t *left_count, int16_t *right_count);

#endif

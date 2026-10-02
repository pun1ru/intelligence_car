#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include <stdint.h>
#include "app_sensor.h"

typedef struct
{
    int32_t delta_count;
    int8_t direction;
    float angle_rad;
    float angular_speed_rad_s;
    float speed_m_s;
    float distance_m;
} app_wheel_motion_t;

typedef struct
{
    app_wheel_motion_t left;
    app_wheel_motion_t right;
} app_encoder_motion_t;

typedef struct
{
    app_encoder_motion_t motion;
    app_encoder_sample_t previous;
    uint8_t initialized;
} app_encoder_tracker_t;

void app_encoder_reset(app_encoder_motion_t *motion);
uint8_t app_encoder_convert(const app_encoder_sample_t *sample, float dt_s,
                            app_encoder_motion_t *motion);
void app_encoder_tracker_reset(app_encoder_tracker_t *tracker);
uint8_t app_encoder_tracker_step(app_encoder_tracker_t *tracker,
                                 const app_encoder_sample_t *absolute,
                                 float dt_s, app_encoder_motion_t *motion);

#endif

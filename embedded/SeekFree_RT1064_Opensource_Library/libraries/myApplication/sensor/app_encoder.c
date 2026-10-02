#include "app_encoder.h"

#include <stddef.h>
#include <math.h>
#include "general_define.h"

void app_encoder_reset(app_encoder_motion_t *motion)
{
    if (motion != NULL)
    {
        *motion = (app_encoder_motion_t){0};
    }
}

static void update_wheel(app_wheel_motion_t *wheel, int16_t count,
                         int32_t sign, float radians_per_count, float dt_s)
{
    float delta_angle;

    wheel->delta_count = (int32_t)count * sign;
    wheel->direction = (wheel->delta_count > 0) ? 1 :
                       ((wheel->delta_count < 0) ? -1 : 0);
    delta_angle = (float)wheel->delta_count * radians_per_count;
    wheel->angle_rad += delta_angle;
    wheel->angular_speed_rad_s = delta_angle / dt_s;
    wheel->speed_m_s = wheel->angular_speed_rad_s * WHEEL_RADIUS_M;
    wheel->distance_m = wheel->angle_rad * WHEEL_RADIUS_M;
}

uint8_t app_encoder_convert(const app_encoder_sample_t *sample, float dt_s,
                            app_encoder_motion_t *motion)
{
    float radians_per_count;

    if ((sample == NULL) || (motion == NULL) || (dt_s <= 0.0f) ||
        !isfinite(dt_s) ||
        (WHEEL_ENCODER_COUNTS_PER_REV <= 0.0f) || (WHEEL_RADIUS_M <= 0.0f) ||
        ((WHEEL_LEFT_COUNT_SIGN != 1) && (WHEEL_LEFT_COUNT_SIGN != -1)) ||
        ((WHEEL_RIGHT_COUNT_SIGN != 1) && (WHEEL_RIGHT_COUNT_SIGN != -1)))
    {
        return 0U;
    }

    radians_per_count = WHEEL_TWO_PI_RAD / WHEEL_ENCODER_COUNTS_PER_REV;
    update_wheel(&motion->left, sample->left_count, WHEEL_LEFT_COUNT_SIGN,
                 radians_per_count, dt_s);
    update_wheel(&motion->right, sample->right_count, WHEEL_RIGHT_COUNT_SIGN,
                 radians_per_count, dt_s);
    return 1U;
}

static int16_t counter_delta(int16_t current, int16_t previous)
{
    int32_t delta = (int32_t)current - (int32_t)previous;

    if (delta > INT16_MAX)
    {
        delta -= (int32_t)UINT16_MAX + 1;
    }
    else if (delta < INT16_MIN)
    {
        delta += (int32_t)UINT16_MAX + 1;
    }
    return (int16_t)delta;
}

void app_encoder_tracker_reset(app_encoder_tracker_t *tracker)
{
    if (tracker != NULL)
    {
        *tracker = (app_encoder_tracker_t){0};
    }
}

uint8_t app_encoder_tracker_step(app_encoder_tracker_t *tracker,
                                 const app_encoder_sample_t *absolute,
                                 float dt_s, app_encoder_motion_t *motion)
{
    app_encoder_sample_t delta;

    if ((tracker == NULL) || (absolute == NULL) || (motion == NULL))
    {
        return 0U;
    }
    if (tracker->initialized == 0U)
    {
        tracker->previous = *absolute;
        tracker->initialized = 1U;
        return 0U;
    }
    delta.left_count = counter_delta(absolute->left_count,
                                     tracker->previous.left_count);
    delta.right_count = counter_delta(absolute->right_count,
                                      tracker->previous.right_count);
    if (app_encoder_convert(&delta, dt_s, &tracker->motion) == 0U)
    {
        return 0U;
    }
    tracker->previous = *absolute;
    *motion = tracker->motion;
    return 1U;
}

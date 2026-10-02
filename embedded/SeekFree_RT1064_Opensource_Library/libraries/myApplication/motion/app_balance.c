#include "app_balance.h"

#include <math.h>
#include <stddef.h>
#include "drv_motion.h"
#include "general_define.h"

#if (BALANCE_SPEED_PERIOD_MS < BALANCE_ANGLE_PERIOD_MS) || \
    ((BALANCE_SPEED_PERIOD_MS % BALANCE_ANGLE_PERIOD_MS) != 0U)
#error Balance speed loop period must be a multiple of the angle loop period.
#endif

static float clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static int32_t pwm_value(float value, float polarity, float limit)
{
    return (int32_t)clamp(value * polarity, limit);
}

void app_balance_init(app_balance_t *control)
{
    if (control == NULL)
    {
        return;
    }
    control_pid_init(&control->wheel_left, WHEEL_SPEED_KP, WHEEL_SPEED_KI,
                     WHEEL_SPEED_KD, WHEEL_SPEED_PWM_LIMIT);
    control_pid_init(&control->wheel_right, WHEEL_SPEED_KP, WHEEL_SPEED_KI,
                     WHEEL_SPEED_KD, WHEEL_SPEED_PWM_LIMIT);
    control_pid_init(&control->angle, BALANCE_ANGLE_KP, BALANCE_ANGLE_KI,
                     BALANCE_ANGLE_KD, BALANCE_ANGLE_PWM_LIMIT);
    control_pid_init(&control->speed, BALANCE_SPEED_KP, BALANCE_SPEED_KI,
                     BALANCE_SPEED_KD, BALANCE_TARGET_TILT_MAX_DEG);
    control->target_tilt_deg = BALANCE_ANGLE_TARGET_PITCH_DEG;
    control->speed_elapsed_ms = 0U;
}

void app_balance_reset(app_balance_t *control)
{
    if (control == NULL)
    {
        return;
    }
    control_pid_reset(&control->wheel_left);
    control_pid_reset(&control->wheel_right);
    control_pid_reset(&control->angle);
    control_pid_reset(&control->speed);
    control->target_tilt_deg = BALANCE_ANGLE_TARGET_PITCH_DEG;
    control->speed_elapsed_ms = 0U;
}

void app_balance_wheel_speed_step(app_balance_t *control,
                                  float left_target_m_s, float right_target_m_s,
                                  float left_speed_m_s, float right_speed_m_s,
                                  app_balance_output_t *output)
{
    float dt_s = (float)WHEEL_SPEED_LOOP_PERIOD_MS / 1000.0f;

    if ((control == NULL) || (output == NULL))
    {
        return;
    }
    output->left_pwm = pwm_value(control_pid_step(&control->wheel_left,
        left_target_m_s, left_speed_m_s, 0.0f, dt_s),
        WHEEL_SPEED_PWM_POLARITY, WHEEL_SPEED_PWM_LIMIT);
    output->right_pwm = pwm_value(control_pid_step(&control->wheel_right,
        right_target_m_s, right_speed_m_s, 0.0f, dt_s),
        WHEEL_SPEED_PWM_POLARITY, WHEEL_SPEED_PWM_LIMIT);
    output->target_tilt_deg = BALANCE_UPRIGHT_PITCH_DEG;
}

void app_balance_angle_step(app_balance_t *control, uint8_t enabled,
                            float target_pitch_deg, float pitch_deg,
                            float pitch_rate_dps,
                            app_balance_output_t *output)
{
    float tilt;
    float pwm;

    if ((control == NULL) || (output == NULL))
    {
        return;
    }
    output->left_pwm = 0;
    output->right_pwm = 0;
    output->target_tilt_deg = BALANCE_UPRIGHT_PITCH_DEG;
    if ((enabled == 0U) || !isfinite(target_pitch_deg) ||
        !isfinite(pitch_deg) || !isfinite(pitch_rate_dps) ||
        (fabsf(pitch_deg - BALANCE_UPRIGHT_PITCH_DEG) >
         VEHICLE_FALL_PITCH_MAX_DEG))
    {
        app_balance_reset(control);
        return;
    }

    tilt = clamp(target_pitch_deg - BALANCE_UPRIGHT_PITCH_DEG,
                 BALANCE_TARGET_TILT_MAX_DEG);
    output->target_tilt_deg = BALANCE_UPRIGHT_PITCH_DEG + tilt;
    pwm = control_pid_step(&control->angle, output->target_tilt_deg,
                           pitch_deg, pitch_rate_dps,
                           (float)BALANCE_ANGLE_PERIOD_MS / 1000.0f);
    output->left_pwm = pwm_value(pwm, BALANCE_ANGLE_PWM_POLARITY,
                                 BALANCE_ANGLE_PWM_LIMIT);
    output->right_pwm = output->left_pwm;
}

void app_balance_step(app_balance_t *control, uint8_t enabled,
                      float target_speed_m_s, float pitch_deg,
                      float pitch_rate_dps, float left_speed_m_s,
                      float right_speed_m_s, app_balance_output_t *output)
{
    float average_speed;

    if ((control == NULL) || (output == NULL))
    {
        return;
    }
    output->left_pwm = 0;
    output->right_pwm = 0;
    output->target_tilt_deg = BALANCE_ANGLE_TARGET_PITCH_DEG;

    if ((enabled == 0U) || !isfinite(target_speed_m_s) ||
        !isfinite(pitch_deg) || !isfinite(pitch_rate_dps) ||
        !isfinite(left_speed_m_s) || !isfinite(right_speed_m_s) ||
        (fabsf(pitch_deg - BALANCE_UPRIGHT_PITCH_DEG) >
         VEHICLE_FALL_PITCH_MAX_DEG))
    {
        app_balance_reset(control);
        return;
    }

    control->speed_elapsed_ms += BALANCE_ANGLE_PERIOD_MS;
    if (control->speed_elapsed_ms >= BALANCE_SPEED_PERIOD_MS)
    {
        average_speed = (left_speed_m_s + right_speed_m_s) * 0.5f;
        control->target_tilt_deg = BALANCE_ANGLE_TARGET_PITCH_DEG +
            BALANCE_SPEED_TO_TILT_SIGN * control_pid_step(&control->speed,
                clamp(target_speed_m_s, BALANCE_SPEED_TARGET_MAX_MPS),
                average_speed, 0.0f,
                (float)control->speed_elapsed_ms / 1000.0f);
        control->speed_elapsed_ms = 0U;
    }

    app_balance_angle_step(control, 1U, control->target_tilt_deg,
                           pitch_deg, pitch_rate_dps, output);
}

void app_balance_apply(const app_balance_output_t *output)
{
    if (output != NULL)
    {
        drv_motion_set_signed(output->left_pwm, output->right_pwm);
    }
}

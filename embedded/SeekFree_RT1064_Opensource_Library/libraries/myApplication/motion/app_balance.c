#include "app_balance.h"

#include <math.h>
#include <stddef.h>
#include "drv_motion.h"
#include "general_define.h"

static uint8_t update_gain(control_pid_t *pid, uint8_t parameter, float value)
{
    if ((pid == NULL) || !isfinite(value) || (value < 0.0f) ||
        (value > SERIAL_PID_MAX_VALUE))
    {
        return 0U;
    }
    if (parameter == SERIAL_PID_GAIN_KP)
    {
        pid->kp = value;
    }
    else if (parameter == SERIAL_PID_GAIN_KI)
    {
        pid->ki = value;
    }
    else if (parameter == SERIAL_PID_GAIN_KD)
    {
        pid->kd = value;
    }
    else
    {
        return 0U;
    }
    control_pid_reset(pid);
    return 1U;
}

uint8_t app_balance_update_pid(app_balance_t *control, uint8_t controller,
                               uint8_t parameter, float value)
{
    if (control == NULL)
    {
        return 0U;
    }
    if (controller == SERIAL_PID_CONTROLLER_WHEEL)
    {
        return (uint8_t)(update_gain(&control->wheel_left, parameter, value) &&
                         update_gain(&control->wheel_right, parameter, value));
    }
    if (controller == SERIAL_PID_CONTROLLER_ANGLE)
    {
        return update_gain(&control->angle, parameter, value);
    }
    if (controller == SERIAL_PID_CONTROLLER_SPEED)
    {
        return update_gain(&control->speed, parameter, value);
    }
    if (controller == SERIAL_PID_CONTROLLER_STEERING)
    {
        return update_gain(&control->steering, parameter, value);
    }
    return 0U;
}

void app_balance_set_wheel_feedforward(app_balance_t *control, float value)
{
    if ((control != NULL) && isfinite(value) &&
        (value >= 0.0f) && (value <= SERIAL_FEEDFORWARD_MAX_PWM_PER_MPS))
    {
        control->wheel_feedforward_gain = value;
    }
}

void app_balance_set_steering_feedforward(app_balance_t *control, float value)
{
    if ((control != NULL) && isfinite(value) &&
        (value >= 0.0f) &&
        (value <= SERIAL_STEERING_FEEDFORWARD_MAX_PWM_PER_DEG))
    {
        control->steering_feedforward_gain = value;
    }
}

void app_balance_set_steering_enabled(app_balance_t *control, uint8_t enabled)
{
    if (control != NULL)
    {
        control->steering_enabled = (enabled != 0U) ? 1U : 0U;
        if (control->steering_enabled == 0U)
        {
            control_pid_reset(&control->steering);
        }
    }
}

void app_balance_set_pitch_target(app_balance_t *control, float target_pitch_deg)
{
    if ((control != NULL) && isfinite(target_pitch_deg))
    {
        /* Keep the speed-loop output between its slower update periods. */
        if (control->base_pitch_deg != target_pitch_deg)
        {
            control->base_pitch_deg = target_pitch_deg;
            control->target_tilt_deg = target_pitch_deg;
            control->speed_elapsed_ms = 0U;
            control_pid_reset(&control->speed);
        }
    }
}

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

static float wrap_angle_deg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
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
    control_pid_init(&control->steering, BALANCE_STEERING_KP,
                     BALANCE_STEERING_KI, BALANCE_STEERING_KD,
                     BALANCE_STEERING_PWM_LIMIT);
    control_pid_set_derivative_filter(&control->wheel_left,
                                      CONTROL_PID_D_FILTER_ALPHA);
    control_pid_set_derivative_filter(&control->wheel_right,
                                      CONTROL_PID_D_FILTER_ALPHA);
    control_pid_set_derivative_filter(&control->angle,
                                      CONTROL_PID_D_FILTER_ALPHA);
    control_pid_set_derivative_filter(&control->speed,
                                      CONTROL_PID_D_FILTER_ALPHA);
    control_pid_set_derivative_filter(&control->steering,
                                      CONTROL_PID_D_FILTER_ALPHA);
    control->base_pitch_deg = BALANCE_ANGLE_TARGET_PITCH_DEG;
    control->target_tilt_deg = control->base_pitch_deg;
    control->wheel_feedforward_gain = WHEEL_SPEED_FEEDFORWARD_PWM_PER_MPS;
    control->steering_feedforward_gain =
        BALANCE_STEERING_FEEDFORWARD_PWM_PER_DEG;
    control->speed_elapsed_ms = 0U;
    control->steering_enabled = BALANCE_STEERING_ENABLE_DEFAULT;
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
    control_pid_reset(&control->steering);
    control->target_tilt_deg = control->base_pitch_deg;
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
    output->left_pwm += (int32_t)(control->wheel_feedforward_gain *
        (left_target_m_s - left_speed_m_s) * WHEEL_SPEED_PWM_POLARITY);
    output->right_pwm = pwm_value(control_pid_step(&control->wheel_right,
        right_target_m_s, right_speed_m_s, 0.0f, dt_s),
        WHEEL_SPEED_PWM_POLARITY, WHEEL_SPEED_PWM_LIMIT);
    output->right_pwm += (int32_t)(control->wheel_feedforward_gain *
        (right_target_m_s - right_speed_m_s) * WHEEL_SPEED_PWM_POLARITY);
    output->left_pwm = (int32_t)clamp((float)output->left_pwm,
                                      WHEEL_SPEED_PWM_LIMIT);
    output->right_pwm = (int32_t)clamp((float)output->right_pwm,
                                       WHEEL_SPEED_PWM_LIMIT);
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
                      float right_speed_m_s, float target_yaw_deg,
                      float yaw_deg, float yaw_rate_dps,
                      app_balance_output_t *output)
{
    float average_speed;
    float yaw_error;
    float steering_pwm;

    if ((control == NULL) || (output == NULL))
    {
        return;
    }
    output->left_pwm = 0;
    output->right_pwm = 0;
    output->target_tilt_deg = control->base_pitch_deg;

    if ((enabled == 0U) || !isfinite(target_speed_m_s) ||
        !isfinite(pitch_deg) || !isfinite(pitch_rate_dps) ||
        !isfinite(left_speed_m_s) || !isfinite(right_speed_m_s) ||
        !isfinite(target_yaw_deg) || !isfinite(yaw_deg) ||
        !isfinite(yaw_rate_dps) ||
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
        control->target_tilt_deg = control->base_pitch_deg +
            BALANCE_SPEED_TO_TILT_SIGN * control_pid_step_limits(
                &control->speed,
                clamp(target_speed_m_s, BALANCE_SPEED_TARGET_MAX_MPS),
                average_speed, 0.0f,
                (float)control->speed_elapsed_ms / 1000.0f,
                BALANCE_TARGET_TILT_FORWARD_MAX_DEG,
                BALANCE_TARGET_TILT_BACKWARD_MAX_DEG);
        control->speed_elapsed_ms = 0U;
    }

    app_balance_angle_step(control, 1U, control->target_tilt_deg,
                           pitch_deg, pitch_rate_dps, output);
    if (control->steering_enabled != 0U)
    {
        yaw_error = wrap_angle_deg(target_yaw_deg - yaw_deg);
        steering_pwm = control_pid_step(&control->steering,
            yaw_deg + yaw_error, yaw_deg, yaw_rate_dps,
            (float)BALANCE_ANGLE_PERIOD_MS / 1000.0f);
        steering_pwm += control->steering_feedforward_gain * yaw_error;
        steering_pwm *= BALANCE_STEERING_PWM_POLARITY;
        /* Differential steering: equal and opposite torque around the axle.
         * With zero forward speed this produces opposite wheel commands, so
         * the vehicle turns about its center instead of pivoting on one wheel.
         */
        output->left_pwm = (int32_t)clamp(
            (float)output->left_pwm - steering_pwm,
            BALANCE_ANGLE_PWM_LIMIT);
        output->right_pwm = (int32_t)clamp(
            (float)output->right_pwm + steering_pwm,
            BALANCE_ANGLE_PWM_LIMIT);
    }
    else
    {
        control_pid_reset(&control->steering);
    }
}

void app_balance_apply(const app_balance_output_t *output)
{
    if (output != NULL)
    {
        drv_motion_set_signed(output->left_pwm, output->right_pwm);
    }
}

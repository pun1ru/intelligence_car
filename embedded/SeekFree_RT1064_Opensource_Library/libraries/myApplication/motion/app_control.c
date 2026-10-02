#include "app_control.h"

#include <math.h>
#include "app_balance.h"
#include "app_encoder.h"
#include "drv_io.h"
#include "drv_motion.h"
#include "general_define.h"

#if (CONTROL_TASK_PERIOD_MS != BALANCE_ANGLE_PERIOD_MS) || \
    ((WHEEL_SPEED_LOOP_PERIOD_MS % CONTROL_TASK_PERIOD_MS) != 0U)
#error Control task must match the angle loop and divide the wheel speed loop period.
#endif

static app_balance_t wheel_test_control;
static app_balance_t angle_control;
static uint8_t wheel_test_initialized;
static uint8_t balance_fault_latched;
static uint32_t wheel_elapsed_ms;
static uint16_t test_left_duty;
static uint16_t test_right_duty;

static uint16_t test_duty(int32_t pwm)
{
    if (pwm <= 0)
    {
        return 0U;
    }
    if (pwm > (int32_t)MOTOR_TEST_MAX_PWM_DUTY)
    {
        return MOTOR_TEST_MAX_PWM_DUTY;
    }
    return (uint16_t)pwm;
}

uint8_t app_control_step(const app_state_t *state,
                         const app_attitude_t *attitude)
{
    app_encoder_sample_t counts;
    app_encoder_motion_t motion = {0};
    app_balance_output_t output;
    uint8_t sample_ready = 0U;
    uint8_t test_enabled;
    uint8_t left_pressed;
    uint8_t right_pressed;

    if (wheel_test_initialized == 0U)
    {
        app_balance_init(&wheel_test_control);
        app_balance_init(&angle_control);
        wheel_test_initialized = 1U;
    }

    wheel_elapsed_ms += CONTROL_TASK_PERIOD_MS;
    if (wheel_elapsed_ms >= WHEEL_SPEED_LOOP_PERIOD_MS)
    {
        drv_motion_read_encoder_delta(&counts.left_count, &counts.right_count);
        wheel_elapsed_ms = 0U;
        sample_ready = 1U;
    }
    test_enabled = drv_io_calibration_switches_on();
    if (test_enabled != 0U)
    {
        app_balance_reset(&angle_control);
        left_pressed = drv_io_button_pressed(DRV_IO_BUTTON_C15);
        right_pressed = drv_io_button_pressed(DRV_IO_BUTTON_C14);
        if (left_pressed == 0U)
        {
            control_pid_reset(&wheel_test_control.wheel_left);
            test_left_duty = 0U;
        }
        if (right_pressed == 0U)
        {
            control_pid_reset(&wheel_test_control.wheel_right);
            test_right_duty = 0U;
        }
        if ((left_pressed == 0U) && (right_pressed == 0U))
        {
            app_balance_reset(&wheel_test_control);
        }
        else if ((sample_ready != 0U) &&
                 (app_encoder_convert(&counts,
                    (float)WHEEL_SPEED_LOOP_PERIOD_MS / 1000.0f,
                    &motion) != 0U))
        {
            app_balance_wheel_speed_step(&wheel_test_control,
                (left_pressed != 0U) ? MOTOR_TEST_TARGET_SPEED_MPS : 0.0f,
                (right_pressed != 0U) ? MOTOR_TEST_TARGET_SPEED_MPS : 0.0f,
                (left_pressed != 0U) ? fabsf(motion.left.speed_m_s) : 0.0f,
                (right_pressed != 0U) ? fabsf(motion.right.speed_m_s) : 0.0f,
                &output);
            test_left_duty = (left_pressed != 0U) ?
                test_duty(output.left_pwm) : 0U;
            test_right_duty = (right_pressed != 0U) ?
                test_duty(output.right_pwm) : 0U;
        }
        else if (sample_ready != 0U)
        {
            app_balance_reset(&wheel_test_control);
            test_left_duty = 0U;
            test_right_duty = 0U;
        }
        drv_motion_set_signed((int32_t)test_left_duty,
                              (int32_t)test_right_duty);
        return 0U;
    }

    app_balance_reset(&wheel_test_control);
    test_left_duty = 0U;
    test_right_duty = 0U;
    if ((state != NULL) && (state->vehicle_mode == APP_VEHICLE_PROTECT))
    {
        balance_fault_latched = 0U;
    }
    if ((drv_io_calibration_switches_off() == 0U) || (state == NULL) ||
        (state->control_mode == APP_STATE_CALIBRATION) ||
        (app_state_balance_enabled(state) == 0U))
    {
        app_balance_reset(&angle_control);
        drv_motion_set_signed(0, 0);
        return 0U;
    }
    if ((balance_fault_latched != 0U) || (attitude == NULL) ||
        !isfinite(attitude->pitch) ||
        !isfinite(attitude->pitch_rate_dps) ||
        (fabsf(attitude->pitch - BALANCE_UPRIGHT_PITCH_DEG) >
         VEHICLE_FALL_PITCH_MAX_DEG))
    {
        balance_fault_latched = 1U;
        app_balance_reset(&angle_control);
        drv_motion_set_signed(0, 0);
        return 1U;
    }

    app_balance_angle_step(&angle_control, 1U, BALANCE_UPRIGHT_PITCH_DEG,
                           attitude->pitch, attitude->pitch_rate_dps, &output);
    app_balance_apply(&output);
    return 0U;
}

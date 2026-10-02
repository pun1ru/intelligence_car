#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "app_balance.h"
#include "app_control.h"
#include "app_encoder.h"
#include "app_serial.h"
#include "app_state.h"
#include "drv_io.h"
#include "general_define.h"

static int32_t applied_left;
static int32_t applied_right;
static uint8_t test_switches_on;
static uint8_t test_c14_pressed;
static uint8_t test_c15_pressed;
static int16_t test_left_delta;
static int16_t test_right_delta;

void drv_motion_set_signed(int32_t left_pwm, int32_t right_pwm)
{
    applied_left = left_pwm;
    applied_right = right_pwm;
}

uint8_t drv_io_button_pressed(drv_io_button_t button)
{
    if (button == DRV_IO_BUTTON_C14)
    {
        return test_c14_pressed;
    }
    if (button == DRV_IO_BUTTON_C15)
    {
        return test_c15_pressed;
    }
    return 0U;
}

uint8_t drv_io_calibration_switches_on(void) { return test_switches_on; }
uint8_t drv_io_calibration_switches_off(void) { return 1U; }

void drv_motion_read_encoder_delta(int16_t *left_count, int16_t *right_count)
{
    *left_count = test_left_delta;
    *right_count = test_right_delta;
    test_left_delta = 0;
    test_right_delta = 0;
}

void drv_io_uart_init(void) {}
uint8_t drv_io_uart_try_read(uint8_t *byte)
{
    (void)byte;
    return 0U;
}
void drv_io_uart_write(const uint8_t *data, size_t length)
{
    (void)data;
    (void)length;
}

static uint8_t parse_frame2(app_serial_parser_t *parser, uint8_t type,
                             uint16_t value, uint16_t value2, uint8_t corrupt,
                             app_serial_command_t *command)
{
    uint8_t frame[SERIAL_FRAME_LENGTH] =
    {
        SERIAL_FRAME_SYNC_0, SERIAL_FRAME_SYNC_1, type,
        (uint8_t)value, (uint8_t)(value >> 8U),
        (uint8_t)value2, (uint8_t)(value2 >> 8U), 0U
    };
    uint8_t index;
    uint8_t parsed = 0U;

    for (index = 2U; index < SERIAL_FRAME_LENGTH - 1U; index++)
    {
        frame[SERIAL_FRAME_LENGTH - 1U] ^= frame[index];
    }
    frame[SERIAL_FRAME_LENGTH - 1U] ^= corrupt;
    for (index = 0U; index < SERIAL_FRAME_LENGTH; index++)
    {
        parsed = app_serial_feed_byte(parser, frame[index], command);
    }
    return parsed;
}

static uint8_t parse_frame(app_serial_parser_t *parser, uint8_t type,
                            uint16_t value, uint8_t corrupt,
                            app_serial_command_t *command)
{
    return parse_frame2(parser, type, value, 0U, corrupt, command);
}

static void step_buttons(app_state_t *state, app_state_buttons_t *buttons,
                         float pitch, uint32_t count)
{
    uint32_t index;
    for (index = 0U; index < count; index++)
    {
        app_state_step(state, buttons, 1U,
                       BALANCE_UPRIGHT_PITCH_DEG + pitch, 10U, NULL);
    }
}

static void test_encoder(void)
{
    app_encoder_sample_t sample = {25, -25};
    app_encoder_motion_t motion;

    assert(fabsf(WHEEL_DIAMETER_M - 0.064f) < 0.000001f);
    assert(fabsf(WHEEL_TRACK_M - 0.200f) < 0.000001f);
    assert(fabsf(WHEEL_ENCODER_COUNTS_PER_REV - 2321.0667f) < 0.001f);
    app_encoder_reset(&motion);
    assert(app_encoder_convert(&sample, 0.1f, &motion) == 1U);
    assert(motion.left.delta_count == 25);
    assert(motion.left.direction == 1);
    assert(fabsf(motion.left.angle_rad -
                 25.0f * WHEEL_TWO_PI_RAD /
                 WHEEL_ENCODER_COUNTS_PER_REV) < 0.0001f);
    assert(fabsf(motion.left.angular_speed_rad_s -
                 motion.left.angle_rad / 0.1f) < 0.0001f);
    assert(fabsf(motion.left.speed_m_s -
                 motion.left.angular_speed_rad_s * WHEEL_RADIUS_M) < 0.0001f);
    assert(motion.right.direction == 1);
    assert(app_encoder_convert(&sample, 0.0f, &motion) == 0U);
    assert(fabsf(motion.left.angle_rad -
                 25.0f * WHEEL_TWO_PI_RAD /
                 WHEEL_ENCODER_COUNTS_PER_REV) < 0.0001f);
}

static void test_encoder_counter_wrap(void)
{
    app_encoder_tracker_t tracker;
    app_encoder_motion_t motion;
    app_encoder_sample_t counter = {32760, -32760};

    app_encoder_tracker_reset(&tracker);
    assert(app_encoder_tracker_step(&tracker, &counter, 0.1f, &motion) == 0U);
    counter.left_count = -32760;
    counter.right_count = 32760;
    assert(app_encoder_tracker_step(&tracker, &counter, 0.1f, &motion) == 1U);
    assert(motion.left.delta_count == 16);
    assert(motion.right.delta_count == 16);
    assert(fabsf(motion.left.angular_speed_rad_s -
                 motion.right.angular_speed_rad_s) < 0.0001f);
    assert(fabsf(motion.left.speed_m_s -
                 motion.left.angular_speed_rad_s * WHEEL_RADIUS_M) < 0.0001f);
}

static void test_motor_probe(void)
{
    int32_t starting_duty;

    assert(WHEEL_LEFT_PWM_FORWARD_LEVEL == 0U);
    assert(WHEEL_RIGHT_PWM_FORWARD_LEVEL == 0U);

    test_c14_pressed = 1U;
    test_c15_pressed = 1U;
    app_control_step(NULL, NULL);
    app_control_step(NULL, NULL);
    assert(applied_left == 0 && applied_right == 0);

    test_switches_on = 1U;
    test_c14_pressed = 0U;
    app_control_step(NULL, NULL);
    app_control_step(NULL, NULL);
    starting_duty = applied_left;
    assert(starting_duty > 0U && starting_duty <= MOTOR_TEST_MAX_PWM_DUTY);
    assert(applied_right == 0);

    test_left_delta = 116;
    app_control_step(NULL, NULL);
    app_control_step(NULL, NULL);
    assert(applied_left < starting_duty);
    assert(applied_right == 0);

    test_c15_pressed = 0U;
    test_c14_pressed = 1U;
    app_control_step(NULL, NULL);
    app_control_step(NULL, NULL);
    assert(applied_left == 0 && applied_right > 0);
    assert(applied_right <= MOTOR_TEST_MAX_PWM_DUTY);

    test_right_delta = -116;
    app_control_step(NULL, NULL);
    app_control_step(NULL, NULL);
    assert(applied_right < starting_duty);

    test_c15_pressed = 1U;
    app_control_step(NULL, NULL);
    app_control_step(NULL, NULL);
    assert(applied_left > 0 && applied_right > 0);
    test_c14_pressed = 0U;
    test_c15_pressed = 0U;
    app_control_step(NULL, NULL);
    assert(applied_left == 0 && applied_right == 0);

    test_c15_pressed = 1U;
    app_control_step(NULL, NULL);
    app_control_step(NULL, NULL);
    assert(applied_left == starting_duty && applied_right == 0);
    test_switches_on = 0U;
    app_control_step(NULL, NULL);
    assert(applied_left == 0 && applied_right == 0);
}

static void test_angle_control_protection(void)
{
    app_state_t state = {0};
    app_attitude_t attitude = {0};

    state.control_mode = APP_STATE_BUTTON_DEBUG;
    state.vehicle_mode = APP_VEHICLE_BALANCE;
    attitude.pitch = BALANCE_UPRIGHT_PITCH_DEG;
    assert(app_control_step(&state, &attitude) == 0U);
    assert(applied_left == 0 && applied_right == 0);

    attitude.pitch = BALANCE_UPRIGHT_PITCH_DEG + 1.0f;
    assert(app_control_step(&state, &attitude) == 0U);
    assert(applied_left == 300 && applied_right == 300);

    attitude.pitch = BALANCE_UPRIGHT_PITCH_DEG - 1.0f;
    assert(app_control_step(&state, &attitude) == 0U);
    assert(applied_left == -300 && applied_right == -300);

    attitude.pitch = BALANCE_UPRIGHT_PITCH_DEG;
    attitude.pitch_rate_dps = 10.0f;
    assert(app_control_step(&state, &attitude) == 0U);
    assert(applied_left == 80 && applied_right == 80);
    attitude.pitch_rate_dps = 0.0f;

    attitude.pitch = BALANCE_UPRIGHT_PITCH_DEG + 46.0f;
    assert(app_control_step(&state, &attitude) == 1U);
    assert(applied_left == 0 && applied_right == 0);

    attitude.pitch = BALANCE_UPRIGHT_PITCH_DEG;
    assert(app_control_step(&state, &attitude) == 1U);
    assert(applied_left == 0 && applied_right == 0);

    state.vehicle_mode = APP_VEHICLE_PROTECT;
    assert(app_control_step(&state, &attitude) == 0U);
    assert(applied_left == 0 && applied_right == 0);

    state.vehicle_mode = APP_VEHICLE_BALANCE;
    attitude.pitch = BALANCE_UPRIGHT_PITCH_DEG + 1.0f;
    assert(app_control_step(&state, &attitude) == 0U);
    assert(applied_left == 300 && applied_right == 300);
    assert(app_control_step(&state, NULL) == 1U);
    assert(applied_left == 0 && applied_right == 0);
    state.vehicle_mode = APP_VEHICLE_PROTECT;
    assert(app_control_step(&state, &attitude) == 0U);
}

static void test_balance(void)
{
    app_balance_t control;
    app_balance_output_t output;
    control_pid_t integral_pid;
    uint32_t index;

    control_pid_init(&integral_pid, 0.0f, 10.0f, 0.0f, 5.0f);
    assert(control_pid_step(&integral_pid, 1.0f, 0.0f, 0.0f, 1.0f) == 5.0f);
    app_balance_init(&control);
    app_balance_wheel_speed_step(&control, 0.5f, 0.5f, 0.0f, 0.0f, &output);
    assert(output.left_pwm == 2006 && output.right_pwm == 2006);
    app_balance_apply(&output);
    assert(applied_left == 2006 && applied_right == 2006);
    app_balance_angle_step(&control, 1U, BALANCE_UPRIGHT_PITCH_DEG + 2.0f,
                           BALANCE_UPRIGHT_PITCH_DEG, 0.0f, &output);
    assert(output.left_pwm == -600 && output.right_pwm == -600);
    app_balance_angle_step(&control, 1U, BALANCE_UPRIGHT_PITCH_DEG,
                           BALANCE_UPRIGHT_PITCH_DEG + 30.0f, 0.0f, &output);
    assert(output.left_pwm == 3000 && output.right_pwm == 3000);
    app_balance_reset(&control);

    for (index = 0U; index < 3U; index++)
    {
        app_balance_step(&control, 1U, 0.5f, BALANCE_UPRIGHT_PITCH_DEG, 0.0f,
                         0.0f, 0.0f, &output);
        assert(output.target_tilt_deg == BALANCE_UPRIGHT_PITCH_DEG);
    }
    app_balance_step(&control, 1U, 0.5f, BALANCE_UPRIGHT_PITCH_DEG, 0.0f,
                     0.0f, 0.0f, &output);
    assert(fabsf(output.target_tilt_deg -
                 (BALANCE_UPRIGHT_PITCH_DEG + 2.0f)) < 0.0001f);
    assert(output.left_pwm == -600 && output.right_pwm == -600);
    app_balance_step(&control, 1U, 0.0f,
                     BALANCE_UPRIGHT_PITCH_DEG + 46.0f, 0.0f,
                     0.0f, 0.0f, &output);
    assert(output.left_pwm == 0 && output.right_pwm == 0);
    assert(control.speed_elapsed_ms == 0U);
}

static void test_state_and_serial(void)
{
    app_state_t state;
    app_state_buttons_t buttons = {0};
    app_serial_parser_t parser;
    app_serial_command_t command;
    uint32_t index;

    app_state_init(&state);
    buttons.switches_off = 1U;
    app_serial_parser_reset(&parser);
    assert(state.control_mode == APP_STATE_BUTTON_DEBUG);
    assert(state.vehicle_mode == APP_VEHICLE_PROTECT);

    buttons.c14_pressed = 1U;
    step_buttons(&state, &buttons, 0.0f, 4U);
    assert(state.vehicle_mode == APP_VEHICLE_BALANCE);
    step_buttons(&state, &buttons, 45.0f, 1U);
    assert(state.vehicle_mode == APP_VEHICLE_BALANCE);
    step_buttons(&state, &buttons, 46.0f, 1U);
    assert(state.vehicle_mode == APP_VEHICLE_PROTECT);
    step_buttons(&state, &buttons, 0.0f, 4U);
    assert(state.vehicle_mode == APP_VEHICLE_PROTECT);
    buttons.c14_pressed = 0U;
    step_buttons(&state, &buttons, 0.0f, 4U);
    buttons.c14_pressed = 1U;
    step_buttons(&state, &buttons, 0.0f, 4U);
    assert(state.vehicle_mode == APP_VEHICLE_BALANCE);

    buttons.c15_pressed = 1U;
    step_buttons(&state, &buttons, 0.0f, 4U);
    assert(state.vehicle_mode == APP_VEHICLE_PROTECT);
    assert(parse_frame(&parser, SERIAL_COMMAND_STATE,
                       SERIAL_STATE_BALANCE, 0U, &command) == 1U);
    app_state_step(&state, &buttons, 1U, BALANCE_UPRIGHT_PITCH_DEG,
                   10U, &command);
    assert(state.vehicle_mode == APP_VEHICLE_PROTECT);
    buttons.c14_pressed = 0U;
    buttons.c15_pressed = 0U;
    step_buttons(&state, &buttons, 0.0f, 4U);

    assert(parse_frame(&parser, SERIAL_COMMAND_STATE,
                       SERIAL_STATE_BALANCE, 1U, &command) == 0U);
    assert(parse_frame(&parser, SERIAL_COMMAND_STATE,
                       SERIAL_STATE_BALANCE, 0U, &command) == 1U);
    app_state_step(&state, &buttons, 1U,
                   BALANCE_UPRIGHT_PITCH_DEG + 11.0f, 10U, &command);
    assert(state.vehicle_mode == APP_VEHICLE_PROTECT);
    app_state_step(&state, &buttons, 1U, BALANCE_UPRIGHT_PITCH_DEG,
                   10U, &command);
    assert(state.control_mode == APP_STATE_SERIAL_DEBUG);
    assert(state.vehicle_mode == APP_VEHICLE_BALANCE);
    assert(parse_frame(&parser, SERIAL_COMMAND_SPEED, 500U, 0U, &command) == 1U);
    app_state_step(&state, &buttons, 1U, BALANCE_UPRIGHT_PITCH_DEG,
                   10U, &command);
    assert(fabsf(state.target_speed_m_s - 0.5f) < 0.0001f);
    assert(parse_frame(&parser, SERIAL_COMMAND_YAW, 9000U, 0U, &command) == 1U);
    app_state_step(&state, &buttons, 1U, BALANCE_UPRIGHT_PITCH_DEG,
                   10U, &command);
    assert(fabsf(state.target_yaw_deg - 90.0f) < 0.0001f);
    assert(state.vehicle_mode == APP_VEHICLE_BALANCE);
    assert(parse_frame2(&parser, SERIAL_COMMAND_MOTION, 250U,
                        (uint16_t)-4500, 0U, &command) == 1U);
    app_state_step(&state, &buttons, 1U, BALANCE_UPRIGHT_PITCH_DEG,
                   10U, &command);
    assert(fabsf(state.target_speed_m_s - 0.25f) < 0.0001f);
    assert(fabsf(state.target_yaw_deg + 45.0f) < 0.0001f);
    assert(parse_frame2(&parser, SERIAL_COMMAND_SPEED, 250U, 1U,
                        0U, &command) == 0U);

    for (index = 0U; index < VEHICLE_SERIAL_TIMEOUT_MS / 10U; index++)
    {
        app_state_step(&state, &buttons, 1U, BALANCE_UPRIGHT_PITCH_DEG,
                       10U, NULL);
    }
    assert(state.control_mode == APP_STATE_BUTTON_DEBUG);
    assert(state.vehicle_mode == APP_VEHICLE_PROTECT);
    assert(state.target_speed_m_s == 0.0f);
    assert(state.target_yaw_deg == 0.0f);

    buttons.c13_pressed = 1U;
    step_buttons(&state, &buttons, 0.0f, 4U);
    assert(state.vehicle_mode == APP_VEHICLE_NAVIGATION);
    assert(app_state_balance_enabled(&state) == 1U);
    app_state_force_protection(&state);
    assert(state.vehicle_mode == APP_VEHICLE_PROTECT);
    assert(app_state_balance_enabled(&state) == 0U);
}

static void test_calibration_switches(void)
{
    app_state_t state;
    app_state_buttons_t buttons = {0};
    app_serial_parser_t parser;
    app_serial_command_t command;

    app_state_init(&state);
    app_serial_parser_reset(&parser);
    buttons.switches_off = 1U;
    buttons.c14_pressed = 1U;
    step_buttons(&state, &buttons, 0.0f, 4U);
    assert(state.vehicle_mode == APP_VEHICLE_BALANCE);

    buttons.switches_off = 0U;
    step_buttons(&state, &buttons, 0.0f, 1U);
    assert(state.vehicle_mode == APP_VEHICLE_PROTECT);
    assert(app_state_balance_enabled(&state) == 0U);

    buttons.switches_on = 1U;
    buttons.c14_pressed = 0U;
    step_buttons(&state, &buttons, 0.0f, 1U);
    assert(state.control_mode == APP_STATE_CALIBRATION);
    buttons.c12_pressed = 1U;
    step_buttons(&state, &buttons, 0.0f, 4U);
    assert(state.calibrate_requested == 1U);
    step_buttons(&state, &buttons, 0.0f, 1U);
    assert(state.calibrate_requested == 0U);
    buttons.c13_pressed = 1U;
    step_buttons(&state, &buttons, 0.0f, 4U);
    assert(state.reset_ekf_requested == 1U);
    assert(state.vehicle_mode == APP_VEHICLE_PROTECT);
    assert(parse_frame2(&parser, SERIAL_COMMAND_MOTION, 250U,
                        1000U, 0U, &command) == 1U);
    app_state_step(&state, &buttons, 1U, BALANCE_UPRIGHT_PITCH_DEG,
                   10U, &command);
    assert(state.control_mode == APP_STATE_CALIBRATION);
    assert(state.target_speed_m_s == 0.0f);
    assert(state.target_yaw_deg == 0.0f);

    buttons.switches_on = 0U;
    buttons.switches_off = 1U;
    step_buttons(&state, &buttons, 0.0f, 1U);
    assert(state.control_mode == APP_STATE_BUTTON_DEBUG);
    assert(state.vehicle_mode == APP_VEHICLE_PROTECT);
    assert(state.reset_ekf_requested == 0U);
}

int main(void)
{
    test_encoder();
    test_encoder_counter_wrap();
    test_motor_probe();
    test_angle_control_protection();
    test_balance();
    test_state_and_serial();
    test_calibration_switches();
    puts("control_state_test passed");
    return 0;
}

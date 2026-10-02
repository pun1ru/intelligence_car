#include "drv_motion.h"

#include <stddef.h>

#include "general_define.h"
#include "zf_common_headfile.h"

static int16_t last_left_count;
static int16_t last_right_count;

static int16_t encoder_delta(int16_t current, int16_t previous)
{
    int32_t difference = (int32_t)current - (int32_t)previous;

    if (difference > INT16_MAX)
    {
        difference -= (int32_t)UINT16_MAX + 1;
    }
    else if (difference < INT16_MIN)
    {
        difference += (int32_t)UINT16_MAX + 1;
    }
    return (int16_t)difference;
}

static void set_motor(int32_t pwm, pwm_channel_enum channel, gpio_pin_enum direction,
                      uint8_t forward_level)
{
    uint32_t duty;
    uint8_t forward = (pwm >= 0) ? 1U : 0U;

    duty = (pwm >= 0) ? (uint32_t)pwm : (uint32_t)(-(int64_t)pwm);
    if (duty > PWM_DUTY_MAX)
    {
        duty = PWM_DUTY_MAX;
    }
    if (duty == 0U)
    {
        pwm_set_duty(channel, 0U);
        return;
    }

    gpio_set_level(direction, (forward == forward_level) ? GPIO_HIGH : GPIO_LOW);
    pwm_set_duty(channel, duty);
}

void drv_motion_init(uint32_t key_period_ms)
{
    key_init(key_period_ms);

    gpio_init(MOTOR_LEFT_DIRECTION_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_DIRECTION_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    pwm_init(MOTOR_LEFT_PWM_CHANNEL, MOTOR_PWM_FREQUENCY_HZ, 0U);
    pwm_init(MOTOR_RIGHT_PWM_CHANNEL, MOTOR_PWM_FREQUENCY_HZ, 0U);

    encoder_quad_init(ENCODER_LEFT_INDEX, ENCODER_LEFT_CHANNEL_1,
                      ENCODER_LEFT_CHANNEL_2);
    encoder_quad_init(ENCODER_RIGHT_INDEX, ENCODER_RIGHT_CHANNEL_1,
                      ENCODER_RIGHT_CHANNEL_2);
    encoder_clear_count(ENCODER_LEFT_INDEX);
    encoder_clear_count(ENCODER_RIGHT_INDEX);
    last_left_count = 0;
    last_right_count = 0;
}

void drv_motion_set_signed(int32_t left_pwm, int32_t right_pwm)
{
    set_motor(left_pwm, MOTOR_LEFT_PWM_CHANNEL, MOTOR_LEFT_DIRECTION_PIN,
              WHEEL_LEFT_PWM_FORWARD_LEVEL);
    set_motor(right_pwm, MOTOR_RIGHT_PWM_CHANNEL, MOTOR_RIGHT_DIRECTION_PIN,
              WHEEL_RIGHT_PWM_FORWARD_LEVEL);
}

void drv_motion_read_encoder(int16_t *left_count, int16_t *right_count)
{
    if ((left_count == NULL) || (right_count == NULL))
    {
        return;
    }

    *left_count = encoder_get_count(ENCODER_LEFT_INDEX);
    *right_count = encoder_get_count(ENCODER_RIGHT_INDEX);
}

void drv_motion_read_encoder_delta(int16_t *left_count, int16_t *right_count)
{
    if ((left_count == NULL) || (right_count == NULL))
    {
        return;
    }

    int16_t left_now = encoder_get_count(ENCODER_LEFT_INDEX);
    int16_t right_now = encoder_get_count(ENCODER_RIGHT_INDEX);

    *left_count = encoder_delta(left_now, last_left_count);
    *right_count = encoder_delta(right_now, last_right_count);
    last_left_count = left_now;
    last_right_count = right_now;
}

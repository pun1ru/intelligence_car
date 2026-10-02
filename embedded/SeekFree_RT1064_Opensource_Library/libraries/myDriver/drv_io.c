#include "drv_io.h"

#include "general_define.h"
#include "zf_common_headfile.h"

uint8_t drv_io_button_pressed(drv_io_button_t button)
{
    gpio_pin_enum pin;

    switch (button)
    {
        case DRV_IO_BUTTON_C12: pin = VEHICLE_BUTTON_CALIBRATE_PIN; break;
        case DRV_IO_BUTTON_C13: pin = VEHICLE_BUTTON_NAV_PIN; break;
        case DRV_IO_BUTTON_C14: pin = VEHICLE_BUTTON_ARM_PIN; break;
        case DRV_IO_BUTTON_C15: pin = VEHICLE_BUTTON_PROTECT_PIN; break;
        default: return 0U;
    }
    return (gpio_get_level(pin) == GPIO_LOW) ? 1U : 0U;
}

uint8_t drv_io_calibration_switches_on(void)
{
    return ((gpio_get_level(VEHICLE_MODE_SWITCH_1_PIN) == VEHICLE_SWITCH_ON_LEVEL) &&
            (gpio_get_level(VEHICLE_MODE_SWITCH_2_PIN) == VEHICLE_SWITCH_ON_LEVEL)) ? 1U : 0U;
}

uint8_t drv_io_calibration_switches_off(void)
{
    return ((gpio_get_level(VEHICLE_MODE_SWITCH_1_PIN) != VEHICLE_SWITCH_ON_LEVEL) &&
            (gpio_get_level(VEHICLE_MODE_SWITCH_2_PIN) != VEHICLE_SWITCH_ON_LEVEL)) ? 1U : 0U;
}

void drv_io_platform_init(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    debug_init();
    gpio_init(VEHICLE_BUTTON_CALIBRATE_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(VEHICLE_MODE_SWITCH_1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(VEHICLE_MODE_SWITCH_2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
}

void drv_io_uart_init(void)
{
    uart_init(VEHICLE_UART_INDEX, VEHICLE_UART_BAUD,
              VEHICLE_UART_TX_PIN, VEHICLE_UART_RX_PIN);
}

uint8_t drv_io_uart_try_read(uint8_t *byte)
{
    return (byte != NULL) ? uart_query_byte(VEHICLE_UART_INDEX, byte) : 0U;
}

void drv_io_uart_write(const uint8_t *data, size_t length)
{
    if ((data != NULL) && (length > 0U))
    {
        uart_write_buffer(VEHICLE_UART_INDEX, data, (uint32)length);
    }
}

void drv_io_display_init(void)
{
    tft180_init();
    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_RED, RGB565_WHITE);
    tft180_show_string(0, 0, "IMU / WHEELS");
    tft180_show_string(0, 16, "Roll:");
    tft180_show_string(0, 32, "Pitch:");
    tft180_show_string(0, 48, "Yaw:");
    tft180_show_string(0, 64, "L rad/s:");
    tft180_show_string(0, 80, "L m/s:");
    tft180_show_string(0, 96, "R rad/s:");
    tft180_show_string(0, 112, "R m/s:");
    tft180_show_string(0, 128, "Gx dps:");
    tft180_show_string(64, 64, "--");
    tft180_show_string(64, 80, "--");
    tft180_show_string(64, 96, "--");
    tft180_show_string(64, 112, "--");
}

void drv_io_display_angles(float roll, float pitch, float yaw,
                           float pitch_rate_dps)
{
    tft180_show_float(48, 16, roll, 3, 1);
    tft180_show_float(48, 32, pitch, 3, 1);
    tft180_show_float(48, 48, yaw, 3, 1);
    tft180_show_float(64, 128, pitch_rate_dps, 3, 1);
}

void drv_io_display_wheels(float left_omega_rad_s, float left_speed_m_s,
                           float right_omega_rad_s, float right_speed_m_s)
{
    tft180_show_float(64, 64, left_omega_rad_s, 3, 2);
    tft180_show_float(64, 80, left_speed_m_s, 3, 3);
    tft180_show_float(64, 96, right_omega_rad_s, 3, 2);
    tft180_show_float(64, 112, right_speed_m_s, 3, 3);
}

void drv_io_buzzer_init(void)
{
    gpio_init(B11, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

void drv_io_buzzer_set(uint8_t active)
{
    gpio_set_level(B11, (active != 0U) ? GPIO_HIGH : GPIO_LOW);
}

void drv_io_delay_ms(uint32_t delay_ms)
{
    system_delay_ms(delay_ms);
}

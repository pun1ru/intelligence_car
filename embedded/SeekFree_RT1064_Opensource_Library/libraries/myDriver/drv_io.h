#ifndef DRV_IO_H
#define DRV_IO_H

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    DRV_IO_BUTTON_C12,
    DRV_IO_BUTTON_C13,
    DRV_IO_BUTTON_C14,
    DRV_IO_BUTTON_C15
} drv_io_button_t;

void drv_io_platform_init(void);
void drv_io_uart_init(void);
void drv_io_uart_write(const uint8_t *data, size_t length);
uint8_t drv_io_uart_try_read(uint8_t *byte);
void drv_io_display_init(void);
void drv_io_display_angles(float roll, float pitch, float yaw);
void drv_io_display_encoders(int16_t left_count, int16_t right_count);
void drv_io_buzzer_init(void);
void drv_io_buzzer_set(uint8_t active);
void drv_io_delay_ms(uint32_t delay_ms);
uint8_t drv_io_button_pressed(drv_io_button_t button);
uint8_t drv_io_calibration_switches_on(void);
uint8_t drv_io_calibration_switches_off(void);

#endif

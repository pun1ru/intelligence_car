#include "app_buzzer.h"

#include <stdint.h>
#include "drv_io.h"

static volatile uint8_t fast_running;

void app_buzzer_init(void)
{
    drv_io_buzzer_init();
}

void app_buzzer_stop(void)
{
    fast_running = 0U;
    drv_io_buzzer_set(0U);
}

void app_buzzer_play(app_buzzer_mode_t mode)
{
    uint8_t count;

    app_buzzer_init();
    if (mode == APP_BUZZER_FAST_CONTINUOUS)
    {
        fast_running = 1U;
        while (fast_running != 0U)
        {
            drv_io_buzzer_set(1U);
            drv_io_delay_ms(50U);
            drv_io_buzzer_set(0U);
            drv_io_delay_ms(50U);
        }
        return;
    }

    count = (uint8_t)mode;
    if (count > 4U)
    {
        count = 1U;
    }
    while (count-- > 0U)
    {
        drv_io_buzzer_set(1U);
        drv_io_delay_ms(200U);
        drv_io_buzzer_set(0U);
        if (count > 0U)
        {
            drv_io_delay_ms(100U);
        }
    }
}

#include "app_system.h"

#include "app_buzzer.h"
#include "app_serial.h"
#include "app_sensor.h"
#include "drv_io.h"
#include "drv_motion.h"

void app_system_platform_init(void)
{
    drv_io_platform_init();
}

void app_system_devices_init(uint32_t control_period_ms)
{
    app_buzzer_play(APP_BUZZER_ONCE);

    app_serial_init();
    app_sensor_init();
    drv_motion_init(control_period_ms);
}

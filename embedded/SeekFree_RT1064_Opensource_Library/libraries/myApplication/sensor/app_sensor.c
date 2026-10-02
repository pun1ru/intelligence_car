#include "app_sensor.h"

#include <stddef.h>

#include "drv_imu.h"
#include "drv_motion.h"

static uint8_t imu_ready;
static uint32_t sample_count;

void app_sensor_init(void)
{
    imu_ready = (drv_imu_init() == 0U) ? 1U : 0U;
    sample_count = 0U;
}

uint8_t app_sensor_imu_ready(void)
{
    return imu_ready;
}

uint8_t app_sensor_read_imu(app_imu_sample_t *sample)
{
    drv_imu_sample_t reading;

    if ((sample == NULL) || (imu_ready == 0U))
    {
        return 0U;
    }

    drv_imu_read(&reading);
    sample->acc_x_g = reading.acc_x_g;
    sample->acc_y_g = reading.acc_y_g;
    sample->acc_z_g = reading.acc_z_g;
    sample->gyro_x_dps = reading.gyro_x_dps;
    sample->gyro_y_dps = reading.gyro_y_dps;
    sample->gyro_z_dps = reading.gyro_z_dps;
    sample->sample_count = ++sample_count;
    return 1U;
}

void app_sensor_read_encoder(app_encoder_sample_t *sample)
{
    if (sample != NULL)
    {
        drv_motion_read_encoder(&sample->left_count, &sample->right_count);
    }
}

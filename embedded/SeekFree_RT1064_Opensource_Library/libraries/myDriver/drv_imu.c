#include "drv_imu.h"

#include <stddef.h>

#include "zf_common_headfile.h"

uint8_t drv_imu_init(void)
{
    return imu660rc_init(IMU660RC_QUARTERNION_DISABLE);
}

void drv_imu_read(drv_imu_sample_t *sample)
{
    if (sample == NULL)
    {
        return;
    }

    imu660rc_get_acc();
    imu660rc_get_gyro();

    sample->acc_x = imu660rc_acc_x;
    sample->acc_y = imu660rc_acc_y;
    sample->acc_z = imu660rc_acc_z;
    sample->gyro_x = imu660rc_gyro_x;
    sample->gyro_y = imu660rc_gyro_y;
    sample->gyro_z = imu660rc_gyro_z;
    sample->acc_x_g = imu660rc_acc_transition(imu660rc_acc_x);
    sample->acc_y_g = imu660rc_acc_transition(imu660rc_acc_y);
    sample->acc_z_g = imu660rc_acc_transition(imu660rc_acc_z);
    sample->gyro_x_dps = imu660rc_gyro_transition(imu660rc_gyro_x);
    sample->gyro_y_dps = imu660rc_gyro_transition(imu660rc_gyro_y);
    sample->gyro_z_dps = imu660rc_gyro_transition(imu660rc_gyro_z);
}

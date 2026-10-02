#include "app_display.h"

#include <stddef.h>
#include "app_encoder.h"
#include "drv_io.h"

static app_encoder_tracker_t wheel_tracker;

void app_display_init(void)
{
    drv_io_display_init();
    app_encoder_tracker_reset(&wheel_tracker);
}

void app_display_update(const app_attitude_t *attitude,
                        const app_encoder_sample_t *encoder, float dt_s)
{
    app_encoder_motion_t motion;

    if ((attitude == NULL) || (encoder == NULL))
    {
        return;
    }

    drv_io_display_angles(attitude->roll, attitude->pitch, attitude->yaw,
                          attitude->pitch_rate_dps);
    if (app_encoder_tracker_step(&wheel_tracker, encoder, dt_s, &motion) != 0U)
    {
        drv_io_display_wheels(motion.left.angular_speed_rad_s,
                              motion.left.speed_m_s,
                              motion.right.angular_speed_rad_s,
                              motion.right.speed_m_s);
    }
}

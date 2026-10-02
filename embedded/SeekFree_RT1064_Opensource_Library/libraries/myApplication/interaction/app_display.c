#include "app_display.h"

#include <stddef.h>
#include "drv_io.h"

void app_display_init(void)
{
    drv_io_display_init();
}

void app_display_update(const app_attitude_t *attitude,
                        const app_encoder_sample_t *encoder)
{
    if ((attitude == NULL) || (encoder == NULL))
    {
        return;
    }

    drv_io_display_angles(attitude->roll, attitude->pitch, attitude->yaw);
    drv_io_display_encoders(encoder->left_count, encoder->right_count);
}

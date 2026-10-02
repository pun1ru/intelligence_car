#include "app_control.h"

#include "drv_motion.h"
void app_control_step(void)
{
    drv_motion_set_signed(0, 0);
}

#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include "app_attitude.h"
#include "app_pose.h"
#include "app_sensor.h"

void app_display_init(void);
/* pose 为 NULL 或 valid 为 0 时，屏幕上的位姿一行显示 "--"。
 * 位姿来自 decision_task，传 NULL 不影响其余读数。 */
void app_display_update(const app_attitude_t *attitude,
                        const app_encoder_sample_t *encoder, float dt_s,
                        const app_pose_t *pose);

#endif

#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include "app_attitude.h"
#include "app_sensor.h"

void app_display_init(void);
void app_display_update(const app_attitude_t *attitude,
                        const app_encoder_sample_t *encoder);

#endif

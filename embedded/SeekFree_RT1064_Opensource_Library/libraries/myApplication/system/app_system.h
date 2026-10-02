#ifndef APP_SYSTEM_H
#define APP_SYSTEM_H

#include <stdint.h>

void app_system_platform_init(void);
void app_system_devices_init(uint32_t control_period_ms);

#endif

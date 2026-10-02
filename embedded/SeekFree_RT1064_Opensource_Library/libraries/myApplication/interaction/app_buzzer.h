#ifndef APP_BUZZER_H
#define APP_BUZZER_H

typedef enum
{
    APP_BUZZER_ONCE = 1,
    APP_BUZZER_TWICE,
    APP_BUZZER_THREE,
    APP_BUZZER_FOUR,
    APP_BUZZER_FAST_CONTINUOUS
} app_buzzer_mode_t;

void app_buzzer_init(void);
void app_buzzer_play(app_buzzer_mode_t mode);
void app_buzzer_stop(void);

#endif

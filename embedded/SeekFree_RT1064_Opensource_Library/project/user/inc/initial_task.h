#ifndef INITIAL_TASK_H_
#define INITIAL_TASK_H_

typedef enum
{
    BUZZER_MODE_ONCE = 1,
    BUZZER_MODE_TWICE,
    BUZZER_MODE_THREE,
    BUZZER_MODE_FOUR,
    BUZZER_MODE_FAST_CONTINUOUS
} buzzer_mode_t;

void buzzer_init(void);
void buzzer_play(buzzer_mode_t mode);
void buzzer_stop(void);
void buzzer_beep_once(void);
void buzzer_beep_twice(void);
void buzzer_beep_three(void);
void buzzer_beep_four(void);
void buzzer_beep_fast(void);
void initial_task(void *pvParameters);
void state_task(void *pvParameters);
void decision_task(void *pvParameters);
void estimate_task(void *pvParameters);
void control_task(void *pvParameters);
void receive_task(void *pvParameters);
void send_task(void *pvParameters);
#endif 

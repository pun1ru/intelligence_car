#ifndef INITIAL_TASK_H_
#define INITIAL_TASK_H_
void initial_task(void *pvParameters);
void state_task(void *pvParameters);
void decision_task(void *pvParameters);
void estimate_task(void *pvParameters);
void control_task(void *pvParameters);
void receive_task(void *pvParameters);
void send_task(void *pvParameters);
#endif 

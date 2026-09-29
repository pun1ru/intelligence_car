#include "zf_common_headfile.h"
#include "general_define.h"
#include "general_include.h"
#include "receive_task.h"


TaskHandle_t stateTaskHandle;
TaskHandle_t decisionTaskHandle;
TaskHandle_t estimateTaskHnadle;
TaskHandle_t controlTaskHandle;
TaskHandle_t receiveTaskHandel;
TaskHandle_t sendTaskHandle;

static void buzzer_beep(void)
{
    gpio_init(B11, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_set_level(B11, GPIO_HIGH);
    system_delay_ms(200U);
    gpio_set_level(B11, GPIO_LOW);
}



void initial_task(void *pvParameters)
{
    uint32_t metric_start = task_metrics_begin();
    uint8 imu_init_status;

    (void)pvParameters;
    buzzer_beep();
    imu_init_status = imu660rc_init(IMU660RC_QUARTERNION_DISABLE);
    g_imu_receive_data.imu_ready = (imu_init_status == 0U) ? 1U : 0U;

    //进入临界区
    taskENTER_CRITICAL();

    //外设初始化，串口-uart，imu-spi，电机pwm，蜂鸣器
    //任务创建初始化
    xTaskCreate(state_task,"state_task",STATE_TASK_STACK_SIZE,NULL,STATE_TASK_PRIORITY,&stateTaskHandle);
    // Task creation
    xTaskCreate(state_task,"state_task",STATE_TASK_STACK_SIZE,NULL,STATE_TASK_PRIORITY,&stateTaskHandle);
    xTaskCreate(decision_task,"decision_task",DECISION_TASK_STACK_SIZE,NULL,DECISION_TASK_PRIORITY,&decisionTaskHandle);
    xTaskCreate(estimate_task,"estimate_task",ESTIMATE_TASK_STACK_SIZE,NULL,ESTIMATE_TASK_PRIORITY,&estimateTaskHnadle);
    xTaskCreate(control_task,"control_task",CONTROL_TASK_STACK_SIZE,NULL,CONTROL_TASK_PRIORITY,&controlTaskHandle);   
    xTaskCreate(receive_task,"receive_task",RECEIVE_TASK_STACK_SIZE,NULL,RECEIVE_TASK_PRIORITY,&receiveTaskHandel);
    xTaskCreate(send_task,"send_task",SEND_TASK_STACK_SIZE,NULL,SEND_TASK_PRIORITY,&sendTaskHandle);
    //删除任务并退出临界区
    taskEXIT_CRITICAL();
    task_metrics_end(TASK_METRIC_INITIAL, metric_start);
    vTaskDelete(NULL);

}

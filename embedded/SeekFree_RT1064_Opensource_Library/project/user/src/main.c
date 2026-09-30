
#include "zf_common_headfile.h"
#include "FreeRTOS.h"
#include "task.h"
#include "general_define.h"
#include "initial_task.h"
#include "task_metrics.h"



int main(void)
{   //逐飞封装好的初始化
    clock_init(SYSTEM_CLOCK_600M);  
    debug_init();
    //初始化任务性能统计模块                   
    task_metrics_init();

    xTaskCreate(initial_task, "initial", 512U, NULL, 3U, NULL);
    //进入任务调度
    vTaskStartScheduler();

    while(1)
    {



    }
}




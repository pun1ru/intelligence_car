#ifndef DECISION_TASK_H
#define DECISION_TASK_H

#include <stdint.h>
#include "app_pose.h"

void decision_task(void *pvParameters);

/* Latest navigation pose, written by decision_task.  Returns 0 when no fresh
 * pose is available, in which case pose is left untouched. */
uint8_t decision_task_get_pose(app_pose_t *pose);

#endif

#ifndef APP_NAV_PATH_H
#define APP_NAV_PATH_H

#include <stdint.h>
#include "pure_pursuit.h"

/*
 * app_nav_path：导航路径表（现场唯一需要改的文件）。
 *
 * 路径 = 一串"要穿过的锥桶间隙中点"，按通过顺序排列。坐标相对发车点：
 * 进入导航状态时的位置为 (0,0)，车头方向为 0 度，+X 向前、+Y 向左，单位 m
 * （见 docs/VEHICLE_CONVENTIONS.md）。填表方法写在 app_nav_path.c 里。
 *
 * 比赛前一日公布锥桶布局后：改表 → 重新编译 → 烧录。因为正式发车那次禁用无线，
 * 路径必须在没有无线的情况下就位。
 */

/* 返回路径点个数。表格未填写时返回 0，调用方应把"没有路径"当作不可行驶。 */
uint16_t app_nav_path_count(void);

/* 返回路径表首地址；返回的指针在整个运行期间有效，内容只读。 */
const pure_pursuit_point_t *app_nav_path_points(void);

#endif

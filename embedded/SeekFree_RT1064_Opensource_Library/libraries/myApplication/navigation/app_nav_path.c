#include "app_nav_path.h"

#include <stddef.h>

/*
 * 导航路径表（改这里，只改这里）
 *
 * 怎么填：锥桶布局公布后，在场地量出每个要穿过的"间隙中点"相对发车点的
 * 前后距离 X 和左右偏移 Y，按通过顺序写进下面的表。
 *   - 原点 = 进入导航状态时车的位置，车头方向 = 0 度，+X 向前、+Y 向左；
 *   - 第一个点建议放在车正前方 0.3~0.5 m，让车先走直线再进桶；
 *   - 最后一个点放回发车线附近，pure_pursuit 用 goal_radius_m 判断"跑完"；
 *   - 点越密贴线越紧，但太密会让前视点频繁跳变，间隔 0.3~0.6 m 比较合适。
 *
 * 填好后把 APP_NAV_PATH_DEFINED 改成 1。没有填之前 count 返回 0，
 * 调用方必须把"没有路径"当作不可行驶处理。
 */
/* 当前表里是**示例路径（一条直线接 S 形）**，只用于在台架上验证
 * "路径 → 目标速度/目标航向"这段逻辑。**比赛前必须用实测锥桶间隙坐标整体替换**，
 * 否则车会照着这条假路径走。 */
#define APP_NAV_PATH_DEFINED 1

static const pure_pursuit_point_t nav_path_points[] =
{
    {0.50f,  0.00f},   /* 示例：先直行 0.5 m */
    {1.20f, -0.25f},   /* 示例：第 1 个间隙，偏右 */
    {1.90f,  0.25f},   /* 示例：第 2 个间隙，偏左 */
    {2.60f, -0.25f},   /* 示例：第 3 个间隙，偏右 */
    {3.30f,  0.00f}    /* 示例：终点 */
};

/*
 * 填写示例（把上面的数组整体替换成这种形式，然后把宏改成 1）：
 *
 *   static const pure_pursuit_point_t nav_path_points[] =
 *   {
 *       {0.40f,  0.00f},   // 发车后先直行 0.4 m
 *       {1.10f, -0.35f},   // 第 1 个锥桶间隙（偏右）
 *       {1.80f,  0.30f},   // 第 2 个锥桶间隙（偏左）
 *       {2.50f, -0.25f},   // 第 3 个锥桶间隙
 *       {0.20f,  0.00f},   // 回到发车线附近
 *   };
 */

uint16_t app_nav_path_count(void)
{
#if (APP_NAV_PATH_DEFINED != 0)
    return (uint16_t)(sizeof(nav_path_points) / sizeof(nav_path_points[0]));
#else
    return 0U;
#endif
}

const pure_pursuit_point_t *app_nav_path_points(void)
{
    return nav_path_points;
}

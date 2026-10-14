/*
 * odometry 的主机侧单元测试：该模块只依赖 C 标准库，不用接板子就能跑。
 * 里程与航向是最容易算错、又最难在车上看出来的部分（例如航向跨 ±180 度
 * 时应该往哪边转、非法输入会不会污染状态），所以专门用测试把行为锁住。
 *
 *   gcc -std=c99 -Wall -Wextra -o odometry_test odometry_test.c \
 *       ../../embedded/SeekFree_RT1064_Opensource_Library/libraries/myAlgorithm/odometry/odometry.c \
 *       -I../../embedded/SeekFree_RT1064_Opensource_Library/libraries/myAlgorithm/odometry \
 *       -lm
 *
 * 全部通过时返回 0，并打印 "25 checks, 0 failure(s)"。
 */

#include <math.h>
#include <stdio.h>

#include "odometry.h"

#define TEST_TRACK_M 0.200f
#define TEST_TOLERANCE 1.0e-4f

static int failures;
static int checks;

static void check_close(const char *name, float actual, float expected)
{
    checks++;
    if (fabsf(actual - expected) > TEST_TOLERANCE)
    {
        printf("FAIL %s: expected %.6f, got %.6f\n", name, expected, actual);
        failures++;
    }
}

static void check_int(const char *name, int actual, int expected)
{
    checks++;
    if (actual != expected)
    {
        printf("FAIL %s: expected %d, got %d\n", name, expected, actual);
        failures++;
    }
}

static float deg_to_rad(float deg)
{
    return deg * ODOMETRY_PI_F / 180.0f;
}

/* 直线行驶：两轮等量前进，应当只增加 x 和累计里程，y 与航向保持 0。 */
static void test_straight_line(void)
{
    odometry_t odometry;

    odometry_init(&odometry, TEST_TRACK_M);
    odometry_reset(&odometry, 0.0f);
    check_int("straight/step", odometry_step(&odometry, 0.01f, 0.010f, 0.010f,
                                             0.0f), 1);
    check_close("straight/x", odometry.x_m, 0.010f);
    check_close("straight/y", odometry.y_m, 0.0f);
    check_close("straight/heading", odometry.heading_rad, 0.0f);
    check_close("straight/path", odometry.path_length_m, 0.010f);
    check_close("straight/yaw_rate", odometry.yaw_rate_rad_s, 0.0f);
}

/* 左转：右轮比左轮走得远，航向应当为正，位置往 +Y（左）偏。 */
static void test_left_turn(void)
{
    odometry_t odometry;

    /* 10 ms 内右轮比左轮多走 2 mm，200 mm 轮距 → 车体角速度 +1 rad/s、
     * 航向变化 0.01 rad。三个量互相印证，所以一起断言。 */
    odometry_init(&odometry, TEST_TRACK_M);
    odometry_reset(&odometry, 0.0f);
    (void)odometry_step(&odometry, 0.01f, 0.010f, 0.012f, deg_to_rad(0.573f));
    check_close("turn/yaw_rate", odometry.yaw_rate_rad_s, 1.0f);
    check_close("turn/heading", odometry.heading_rad, deg_to_rad(0.573f));
    check_close("turn/y", odometry.y_m, 0.011f * sinf(deg_to_rad(0.573f)));
    check_close("turn/x", odometry.x_m, 0.011f * cosf(deg_to_rad(0.573f)));
}

/* 原点偏置：原点航向不同时，相对航向只应反映"差值"，与绝对 yaw 无关。 */
static void test_origin_offsets_heading(void)
{
    odometry_t odometry;

    odometry_init(&odometry, TEST_TRACK_M);
    odometry_reset(&odometry, deg_to_rad(30.0f));
    (void)odometry_step(&odometry, 0.01f, 0.010f, 0.010f, deg_to_rad(30.0f));
    check_close("origin/heading", odometry.heading_rad, 0.0f);

    odometry_reset(&odometry, deg_to_rad(30.0f));
    (void)odometry_step(&odometry, 0.01f, 0.010f, 0.010f, deg_to_rad(40.0f));
    check_close("origin/relative", odometry.heading_rad, deg_to_rad(10.0f));
}

/* 航向回绕：跨 ±180 度边界时必须走短边，否则转向会绕一大圈。 */
static void test_heading_wraps(void)
{
    odometry_t odometry;

    /* 原点在 170 度，当前 -170 度：实际只相差 20 度。
     * 若实现成走长边，就会得到 -340 度这类错误结果。 */
    odometry_init(&odometry, TEST_TRACK_M);
    odometry_reset(&odometry, deg_to_rad(170.0f));
    (void)odometry_step(&odometry, 0.01f, 0.010f, 0.010f, deg_to_rad(-170.0f));
    check_close("wrap/heading", odometry.heading_rad, deg_to_rad(20.0f));
}

/* 非法输入：空指针、非正 dt、NaN/Inf、无效轮距都必须被拒绝，且不污染已有位姿。 */
static void test_rejects_invalid_input(void)
{
    odometry_t odometry;

    /* 还没定原点时不允许积分，否则等于拿一个没有意义的参考站在算。 */
    odometry_init(&odometry, TEST_TRACK_M);
    check_int("invalid/no_origin", odometry_step(&odometry, 0.01f, 0.010f,
                                                 0.010f, 0.0f), 0);

    odometry_reset(&odometry, 0.0f);
    check_int("invalid/null", odometry_step(NULL, 0.01f, 0.01f, 0.01f, 0.0f), 0);
    check_int("invalid/dt_zero", odometry_step(&odometry, 0.0f, 0.010f,
                                               0.010f, 0.0f), 0);
    check_int("invalid/dt_negative", odometry_step(&odometry, -0.01f, 0.010f,
                                                   0.010f, 0.0f), 0);
    check_int("invalid/yaw_nan", odometry_step(&odometry, 0.01f, 0.010f,
                                               0.010f, NAN), 0);
    check_int("invalid/delta_inf", odometry_step(&odometry, 0.01f, INFINITY,
                                                 0.010f, 0.0f), 0);
    check_close("invalid/keeps_pose", odometry.x_m, 0.0f);

    /* 轮距非法时 reset 必须被整体拒绝，不能只写一半状态，
     * 否则之后会拿错误的几何关系算位姿。 */
    odometry_init(&odometry, 0.0f);
    odometry_reset(&odometry, 0.0f);
    check_int("invalid/track_zero", odometry_step(&odometry, 0.01f, 0.010f,
                                                  0.010f, 0.0f), 0);
    odometry_init(&odometry, NAN);
    odometry_reset(&odometry, 0.0f);
    check_int("invalid/track_nan", odometry_step(&odometry, 0.01f, 0.010f,
                                                 0.010f, 0.0f), 0);

    odometry_init(&odometry, TEST_TRACK_M);
    odometry_reset(&odometry, NAN);
    check_int("invalid/yaw_nan_origin", odometry.origin_set, 0);
}

/* 倒车：两轮同时后退，应当只让 x 和累计里程变负。 */
static void test_reverse_travel(void)
{
    odometry_t odometry;

    odometry_init(&odometry, TEST_TRACK_M);
    odometry_reset(&odometry, 0.0f);
    (void)odometry_step(&odometry, 0.01f, -0.010f, -0.010f, 0.0f);
    check_close("reverse/x", odometry.x_m, -0.010f);
    check_close("reverse/path", odometry.path_length_m, -0.010f);
}

int main(void)
{
    /* 每项独立运行：共享的只有计数器，任何一项失败都会反映在退出码里，
     * 方便接进脚本或 CI。 */
    test_straight_line();
    test_left_turn();
    test_origin_offsets_heading();
    test_heading_wraps();
    test_rejects_invalid_input();
    test_reverse_travel();

    printf("%d checks, %d failure(s)\n", checks, failures);
    return (failures == 0) ? 0 : 1;
}

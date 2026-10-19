/*
 * pure_pursuit 的主机侧单元测试：该模块只做几何计算，不依赖硬件，
 * 所以路径跟踪的逻辑可以在电脑上先验证清楚，再上车调参。
 * 重点覆盖容易出错的地方：横向偏差的左右符号、前视点只前进不后退、
 * 急弯减速、终点判定、以及非法输入不能算出"有效指令"。
 *
 *   gcc -std=c99 -Wall -Wextra -o pure_pursuit_test pure_pursuit_test.c \
 *       ../../embedded/SeekFree_RT1064_Opensource_Library/libraries/myAlgorithm/path_tracking/pure_pursuit.c \
 *       -I../../embedded/SeekFree_RT1064_Opensource_Library/libraries/myAlgorithm/path_tracking \
 *       -lm
 */

#include <math.h>
#include <stdio.h>
#include "pure_pursuit.h"

static int failures;
static int checks;

static void check_close(const char *name, float actual, float expected)
{
    checks++;
    if (fabsf(actual - expected) > 1.0e-3f)
    {
        printf("FAIL %s: expected %.4f, got %.4f\n", name, expected, actual);
        failures++;
    }
}

static void check_between(const char *name, float actual, float low, float high)
{
    checks++;
    if (!(actual >= low && actual <= high))
    {
        printf("FAIL %s: expected in [%.4f, %.4f], got %.4f\n", name, low, high,
               actual);
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

static pure_pursuit_config_t test_config(void)
{
    pure_pursuit_config_t config;

    config.lookahead_m = 0.50f;
    config.speed_max_m_s = 0.80f;
    config.speed_min_m_s = 0.15f;
    config.slow_radius_m = 0.50f;
    config.goal_radius_m = 0.10f;
    return config;
}

/* 沿 +X 的直线路径 */
static const pure_pursuit_point_t straight_path[] =
{
    {0.0f, 0.0f}, {1.0f, 0.0f}, {2.0f, 0.0f}, {3.0f, 0.0f}
};

/* 沿 +X、但整条路径偏 +Y（左侧）0.5 m，用来检查横向偏差的符号 */
static const pure_pursuit_point_t offset_path[] =
{
    {0.0f, 0.5f}, {1.0f, 0.5f}, {2.0f, 0.5f}
};

/* 先直行 0.3 m 再急转 +Y，用来检查曲率减速 */
static const pure_pursuit_point_t sharp_turn_path[] =
{
    {0.3f, 0.0f}, {0.3f, 0.5f}, {0.3f, 1.0f}
};

/* 直线：车头已经朝前，期望航向接近 0、速度取上限、横向偏差接近 0 */
static void test_straight_ahead(void)
{
    pure_pursuit_config_t config = test_config();
    pure_pursuit_state_t state;
    pure_pursuit_output_t output;

    pure_pursuit_init(&state);
    check_int("straight/valid", pure_pursuit_step(&config, &state, straight_path,
        4U, 0.2f, 0.0f, 0.0f, &output), 1);
    check_close("straight/heading", output.target_heading_deg, 0.0f);
    check_close("straight/yaw_error", output.yaw_error_deg, 0.0f);
    check_close("straight/speed", output.target_speed_m_s, 0.8f);
    check_close("straight/cross_track", output.cross_track_m, 0.0f);
    /* 沿路径从最近点往前走 0.5 m 正好到第 1 个点 (1,0) */
    check_int("straight/target_index", (int)output.target_index, 1);
    check_int("straight/finished", output.finished, 0);
}

/* 车在路径右侧时应当往左打，横向偏差为负 */
static void test_offset_right_of_path(void)
{
    pure_pursuit_config_t config = test_config();
    pure_pursuit_state_t state;
    pure_pursuit_output_t output;

    pure_pursuit_init(&state);
    (void)pure_pursuit_step(&config, &state, offset_path, 3U, 0.2f, 0.0f, 0.0f,
                            &output);
    check_between("right/yaw_error", output.yaw_error_deg, 10.0f, 90.0f);
    check_close("right/cross_track", output.cross_track_m, -0.5f);
}

/* 车在路径左侧时应当往右打，横向偏差为正 */
static void test_offset_left_of_path(void)
{
    pure_pursuit_config_t config = test_config();
    pure_pursuit_state_t state;
    pure_pursuit_output_t output;

    pure_pursuit_init(&state);
    (void)pure_pursuit_step(&config, &state, offset_path, 3U, 0.2f, 1.0f, 0.0f,
                            &output);
    check_between("left/yaw_error", output.yaw_error_deg, -90.0f, -10.0f);
    check_close("left/cross_track", output.cross_track_m, 0.5f);
}

/* 前方急弯：速度要低于上限，且不低于下限（平衡车不能停车） */
static void test_sharp_turn_slows_down(void)
{
    pure_pursuit_config_t config = test_config();
    pure_pursuit_state_t state;
    pure_pursuit_output_t output;

    pure_pursuit_init(&state);
    (void)pure_pursuit_step(&config, &state, sharp_turn_path, 3U, 0.0f, 0.0f,
                            0.0f, &output);
    check_between("turn/speed", output.target_speed_m_s, 0.15f, 0.75f);
    check_between("turn/yaw_error", output.yaw_error_deg, 30.0f, 90.0f);
}

/* 到达最后一个点附近即判定跑完 */
static void test_goal_detection(void)
{
    pure_pursuit_config_t config = test_config();
    pure_pursuit_state_t state;
    pure_pursuit_output_t output;

    pure_pursuit_init(&state);
    (void)pure_pursuit_step(&config, &state, straight_path, 4U, 3.02f, 0.0f,
                            0.0f, &output);
    check_int("goal/finished", output.finished, 1);

    pure_pursuit_init(&state);
    (void)pure_pursuit_step(&config, &state, straight_path, 4U, 2.50f, 0.0f,
                            0.0f, &output);
    check_int("goal/not_yet", output.finished, 0);
}

/* 前视点只能前进：路径来回交叉时不能退回上一段 */
static void test_index_never_goes_back(void)
{
    pure_pursuit_config_t config = test_config();
    pure_pursuit_state_t state;
    pure_pursuit_output_t output;

    pure_pursuit_init(&state);
    (void)pure_pursuit_step(&config, &state, straight_path, 4U, 0.2f, 0.0f,
                            0.0f, &output);
    (void)pure_pursuit_step(&config, &state, straight_path, 4U, 2.20f, 0.0f,
                            0.0f, &output);
    check_int("monotonic/forward", (int)state.index, 2);
    /* 位姿突然回到起点（丢帧、重定位）也不许把序号退回去 */
    (void)pure_pursuit_step(&config, &state, straight_path, 4U, 0.20f, 0.0f,
                            0.0f, &output);
    check_int("monotonic/no_back", (int)state.index, 2);
}

/* 单点路径：只有目标点，没有方向线段，不能算出 NaN 或横向偏差乱跳 */
static void test_single_point_path(void)
{
    pure_pursuit_config_t config = test_config();
    pure_pursuit_state_t state;
    pure_pursuit_output_t output;
    const pure_pursuit_point_t one_point[] = {{1.0f, 0.0f}};

    pure_pursuit_init(&state);
    check_int("single/valid", pure_pursuit_step(&config, &state, one_point, 1U,
        0.0f, 0.0f, 0.0f, &output), 1);
    check_close("single/heading", output.target_heading_deg, 0.0f);
    check_int("single/index", (int)output.target_index, 0);
}

/* 车正好压在最后一个点上：前视点延伸到末段方向之外，期望航向应等于末段方向
 * （0°），而不是原地保持当前 7.5° 或吃 atan2(0,0) 的噪声。 */
static void test_on_top_of_target(void)
{
    pure_pursuit_config_t config = test_config();
    pure_pursuit_state_t state;
    pure_pursuit_output_t output;
    const pure_pursuit_point_t two_points[] = {{1.0f, 0.0f}, {2.0f, 0.0f}};

    pure_pursuit_init(&state);
    (void)pure_pursuit_step(&config, &state, two_points, 2U, 2.0f, 0.0f, 7.5f,
                            &output);
    check_close("ontop/heading", output.target_heading_deg, 0.0f);
}

/* 快到终点时前视点要沿末段方向延伸：
 * 车在 (2.7, 0.2) 偏在直线右侧 0.2 m，如果目标退化成终点 (3,0)，期望航向是
 * -33.7°（斜插）；延伸 0.5 m 后目标是 (3.5,0)，航向约 -14°，车会顺着末段
 * 方向慢慢并回中线，而不是到点再回正。 */
static void test_lookahead_extrapolates_past_end(void)
{
    pure_pursuit_config_t config = test_config();
    pure_pursuit_state_t state;
    pure_pursuit_output_t output;

    pure_pursuit_init(&state);
    (void)pure_pursuit_step(&config, &state, straight_path, 4U, 2.70f, 0.20f,
                            0.0f, &output);
    check_between("extrapolate/yaw_error", output.yaw_error_deg, -20.0f, -8.0f);
    check_int("extrapolate/target_index", (int)output.target_index, 3);
}

/* 前视点要在段内插值，而不是退到下一个路径点：
 * 路径点每 1 m 一个，车在 (0.2,0.2)、前视 0.5 m，前视点应是 (0.5,0)，
 * 期望航向 atan2(-0.2,0.3)≈-33.7°；退到下一个点 (1,0) 则只有 -14°，
 * 直线段点稀时就会表现为"朝着很远的点抄近路"。 */
static void test_lookahead_interpolates_within_segment(void)
{
    pure_pursuit_config_t config = test_config();
    pure_pursuit_state_t state;
    pure_pursuit_output_t output;

    pure_pursuit_init(&state);
    (void)pure_pursuit_step(&config, &state, straight_path, 4U, 0.20f, 0.20f,
                            0.0f, &output);
    check_between("interpolate/yaw_error", output.yaw_error_deg, -40.0f, -28.0f);
}

/* 非法输入一律返回 0，并且不改写输出 */
static void test_rejects_invalid_input(void)
{
    pure_pursuit_config_t config = test_config();
    pure_pursuit_config_t bad_config = test_config();
    pure_pursuit_state_t state;
    pure_pursuit_output_t output;

    output.target_speed_m_s = -123.0f;
    pure_pursuit_init(&state);
    check_int("invalid/null_path", pure_pursuit_step(&config, &state, NULL, 4U,
        0.0f, 0.0f, 0.0f, &output), 0);
    check_int("invalid/zero_count", pure_pursuit_step(&config, &state,
        straight_path, 0U, 0.0f, 0.0f, 0.0f, &output), 0);
    check_int("invalid/null_state", pure_pursuit_step(&config, NULL,
        straight_path, 4U, 0.0f, 0.0f, 0.0f, &output), 0);
    check_int("invalid/null_output", pure_pursuit_step(&config, &state,
        straight_path, 4U, 0.0f, 0.0f, 0.0f, NULL), 0);
    check_int("invalid/nan_pose", pure_pursuit_step(&config, &state,
        straight_path, 4U, NAN, 0.0f, 0.0f, &output), 0);
    bad_config.lookahead_m = 0.0f;
    check_int("invalid/lookahead_zero", pure_pursuit_step(&bad_config, &state,
        straight_path, 4U, 0.0f, 0.0f, 0.0f, &output), 0);
    bad_config = test_config();
    bad_config.slow_radius_m = -1.0f;
    check_int("invalid/slow_radius", pure_pursuit_step(&bad_config, &state,
        straight_path, 4U, 0.0f, 0.0f, 0.0f, &output), 0);
    check_close("invalid/output_untouched", output.target_speed_m_s, -123.0f);
}

int main(void)
{
    test_straight_ahead();
    test_offset_right_of_path();
    test_offset_left_of_path();
    test_sharp_turn_slows_down();
    test_goal_detection();
    test_index_never_goes_back();
    test_single_point_path();
    test_on_top_of_target();
    test_lookahead_extrapolates_past_end();
    test_lookahead_interpolates_within_segment();
    test_rejects_invalid_input();

    printf("%d checks, %d failure(s)\n", checks, failures);
    return (failures == 0) ? 0 : 1;
}

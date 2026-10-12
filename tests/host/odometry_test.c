/*
 * Host test for the odometry algorithm.  The module only depends on the C
 * standard library, so it runs on the PC without the board:
 *
 *   gcc -std=c99 -Wall -Wextra -o odometry_test odometry_test.c \
 *       ../../embedded/SeekFree_RT1064_Opensource_Library/libraries/myAlgorithm/odometry/odometry.c \
 *       -I../../embedded/SeekFree_RT1064_Opensource_Library/libraries/myAlgorithm/odometry \
 *       -lm
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

static void test_left_turn(void)
{
    odometry_t odometry;

    /* The right wheel leads by 2 mm over 10 ms on a 200 mm track, which is
     * +1 rad/s and a 0.01 rad heading change. */
    odometry_init(&odometry, TEST_TRACK_M);
    odometry_reset(&odometry, 0.0f);
    (void)odometry_step(&odometry, 0.01f, 0.010f, 0.012f, deg_to_rad(0.573f));
    check_close("turn/yaw_rate", odometry.yaw_rate_rad_s, 1.0f);
    check_close("turn/heading", odometry.heading_rad, deg_to_rad(0.573f));
    check_close("turn/y", odometry.y_m, 0.011f * sinf(deg_to_rad(0.573f)));
    check_close("turn/x", odometry.x_m, 0.011f * cosf(deg_to_rad(0.573f)));
}

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

static void test_heading_wraps(void)
{
    odometry_t odometry;

    /* Crossing the +/-180 degree boundary must take the short way round. */
    odometry_init(&odometry, TEST_TRACK_M);
    odometry_reset(&odometry, deg_to_rad(170.0f));
    (void)odometry_step(&odometry, 0.01f, 0.010f, 0.010f, deg_to_rad(-170.0f));
    check_close("wrap/heading", odometry.heading_rad, deg_to_rad(20.0f));
}

static void test_rejects_invalid_input(void)
{
    odometry_t odometry;

    /* No origin yet: integration must not run. */
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

    /* A rejected reset must leave the origin unset rather than half applied. */
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
    test_straight_line();
    test_left_turn();
    test_origin_offsets_heading();
    test_heading_wraps();
    test_rejects_invalid_input();
    test_reverse_travel();

    printf("%d checks, %d failure(s)\n", checks, failures);
    return (failures == 0) ? 0 : 1;
}

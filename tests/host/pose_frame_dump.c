/*
 * 打印车端位姿帧编码器实际产出的字节，用来和上位机 Python 解码器对表。
 * 这类"两边各写一遍"的协议最容易出布局不一致的问题（字段顺序、缩放、
 * CRC 字节序），所以这里直接调用固件里的 app_serial_encode_pose 生成样本，
 * 结果再作为固定向量写进 upper/test_protocol.py。
 *
 *   gcc -std=c99 -Wall -Wextra -o pose_frame_dump pose_frame_dump.c \
 *       ../../embedded/SeekFree_RT1064_Opensource_Library/libraries/myApplication/communication/app_serial.c \
 *       -I../../embedded/SeekFree_RT1064_Opensource_Library/project/user/inc \
 *       -I../../embedded/SeekFree_RT1064_Opensource_Library/libraries/myApplication/communication \
 *       -I../../embedded/SeekFree_RT1064_Opensource_Library/libraries/myDriver -lm
 *
 * 下面三个 drv_io_uart_* 是空实现：app_serial.c 里发送串口的部分需要它们
 * 才能链接，而本测试只调用"只填帧、不发送"的 app_serial_encode_pose。
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "app_serial.h"

void drv_io_uart_init(void)
{
}

void drv_io_uart_write(const uint8_t *data, size_t length)
{
    (void)data;
    (void)length;
}

uint8_t drv_io_uart_try_read(uint8_t *byte)
{
    (void)byte;
    return 0U;
}

int main(void)
{
    app_serial_pose_t pose = {0};
    uint8_t frame[SERIAL_NAV_POSE_LENGTH];
    uint8_t index;

    /* Values chosen to exercise negative, zero and boundary-adjacent scales. */
    pose.valid = SERIAL_POSE_VALID_DATA | SERIAL_POSE_VALID_NAVIGATION |
                 SERIAL_POSE_VALID_TRACKING;
    pose.x_m = -1.234f;
    pose.y_m = 0.567f;
    pose.heading_deg = 90.0f;
    pose.path_length_m = 2.345f;
    pose.yaw_rate_rad_s = -0.125f;
    pose.nav_target_speed_m_s = 0.45f;
    pose.nav_target_yaw_deg = -12.34f;
    pose.nav_cross_track_m = -0.08f;
    pose.nav_target_index = 3U;

    if (app_serial_encode_pose(&pose, 3U, frame) == 0U)
    {
        printf("encode failed\n");
        return 1;
    }
    for (index = 0U; index < SERIAL_NAV_POSE_LENGTH; index++)
    {
        printf("%02x", frame[index]);
    }
    printf("\n");
    return 0;
}

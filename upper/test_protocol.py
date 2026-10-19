import struct
import unittest

from protocol import (
    BALANCE, FEEDFORWARD, MOTION, NAV_POSE_LENGTH, NAV_POSE_TYPE, PID_BASE,
    POSE_VALID_DATA, POSE_VALID_NAVIGATION, POSE_VALID_TRACKING, STATE,
    STEERING_FEEDFORWARD,
    SUPPORT, TELEMETRY_LENGTH, NavPoseTelemetry, SequenceTracker, TelemetryParser,
    command_frame, crc16, decode_nav_pose, decode_telemetry, feedforward_frame,
    pid_frame, pose_reset_frame, steering_feedforward_frame,
)


def sample_frame() -> bytes:
    frame = bytearray(TELEMETRY_LENGTH)
    frame[:6] = bytes((0x5A, 0xA5, 1, 0x80, 7, 0x0F))
    struct.pack_into("<I", frame, 6, 12345)
    frame[10:12] = bytes((1, 1))
    struct.pack_into("<h", frame, 14, -9000)
    struct.pack_into("<h", frame, 32, 250)
    struct.pack_into("<h", frame, 40, 1234)
    struct.pack_into("<h", frame, 42, -1234)
    struct.pack_into("<h", frame, 44, 250)
    struct.pack_into("<H", frame, 50, crc16(frame[:50]))
    return bytes(frame)


def sample_pose_frame() -> bytes:
    frame = bytearray(NAV_POSE_LENGTH)
    frame[:6] = bytes((0x5A, 0xA5, 1, NAV_POSE_TYPE, 3,
                       POSE_VALID_DATA | POSE_VALID_NAVIGATION
                       | POSE_VALID_TRACKING))
    struct.pack_into("<8h", frame, 6, -1234, 567, 9000, 2345, -125,
                     450, -1234, -80)
    frame[22] = 3
    struct.pack_into("<H", frame, 23, crc16(frame[:23]))
    return bytes(frame)


# 固定向量：由车端编码器本人产出（tests/host/pose_frame_dump.c 调用
# app_serial_encode_pose 打印出来的字节）。这样"C 端怎么填帧"和
# "Python 怎么解帧"就不会各改各的而没人发现。
VEHICLE_POSE_FRAME = bytes.fromhex(
    "5aa5018203072efb37022823290983ffc2012efbb0ff03ac53")


class ProtocolTest(unittest.TestCase):
    def test_crc_and_commands(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)
        self.assertEqual(command_frame(STATE, BALANCE), bytes.fromhex("a5 5a 01 01 00 00 00 00"))
        self.assertEqual(len(command_frame(STATE, SUPPORT)), 8)
        self.assertEqual(command_frame(MOTION, 250, -4500), bytes.fromhex("a5 5a 04 fa 00 6c ee 7c"))
        with self.assertRaises(ValueError):
            command_frame(STATE, 9)
        frame = pid_frame(1, 0, 300.0)
        self.assertEqual(frame[2], PID_BASE + 3)
        self.assertEqual(struct.unpack_from("<f", frame, 3)[0], 300.0)
        self.assertEqual(frame[7], 0x86)
        wheel_ff = feedforward_frame(12.25)
        self.assertEqual(wheel_ff[2], FEEDFORWARD)
        # 位姿复位命令：type=0x09，两个值必须为 0（车端会拒绝带值的帧）
        self.assertEqual(pose_reset_frame(),
                         bytes.fromhex("a5 5a 09 00 00 00 00 09"))
        self.assertEqual(struct.unpack_from("<h", wheel_ff, 3)[0], 12)
        steering_ff = steering_feedforward_frame(4.25)
        self.assertEqual(steering_ff[2], STEERING_FEEDFORWARD)
        self.assertEqual(struct.unpack_from("<h", steering_ff, 3)[0], 4)

    def test_telemetry_round_trip_and_resync(self):
        frame = sample_frame()
        decoded = decode_telemetry(frame)
        self.assertEqual(decoded.uptime_ms, 12345)
        self.assertEqual(decoded.pitch_deg, -90)
        self.assertEqual(decoded.left_speed_m_s, 0.25)
        self.assertEqual((decoded.left_pwm, decoded.right_pwm), (1234, -1234))
        corrupted = bytearray(frame)
        corrupted[14] ^= 1
        parser = TelemetryParser()
        self.assertEqual(parser.feed(b"noise" + bytes(corrupted) + frame[:19]), [])
        self.assertEqual(parser.feed(frame[19:] + frame)[0], decoded)
        self.assertEqual(parser.bad_frames, 1)

    def test_reject_version_and_crc(self):
        frame = bytearray(sample_frame())
        frame[2] = 2
        with self.assertRaises(ValueError):
            decode_telemetry(frame)
        frame = bytearray(sample_frame())
        frame[16] ^= 1
        with self.assertRaises(ValueError):
            decode_telemetry(frame)

    def test_nav_pose_round_trip_and_interleaving(self):
        # 先确认解码结果和写入的物理量一致（缩放、符号、字节序）
        pose = sample_pose_frame()
        decoded = decode_nav_pose(pose)
        self.assertEqual((decoded.x_m, decoded.y_m), (-1.234, 0.567))
        self.assertEqual(decoded.heading_deg, 90.0)
        self.assertEqual(decoded.path_length_m, 2.345)
        self.assertEqual(decoded.yaw_rate_rad_s, -0.125)
        self.assertTrue(decoded.nav_active)
        self.assertEqual(decoded.nav_target_speed_m_s, 0.45)
        self.assertEqual(decoded.nav_target_yaw_deg, -12.34)
        self.assertEqual(decoded.nav_cross_track_m, -0.08)
        self.assertEqual(decoded.nav_target_index, 3)
        self.assertEqual(decoded.valid, POSE_VALID_DATA | POSE_VALID_NAVIGATION
                         | POSE_VALID_TRACKING)

        # 车端是把三种帧连着发出来的，所以解析器必须能在一段字节流里按类型
        # 各自切帧——这是实际链路上最容易出错的地方。
        parser = TelemetryParser()
        frames = parser.feed(sample_frame() + pose + sample_frame())
        self.assertEqual(len(frames), 3)
        self.assertIsInstance(frames[1], NavPoseTelemetry)
        self.assertEqual(frames[1], decoded)
        self.assertEqual(parser.bad_frames, 0)

        # 数据位被改动（CRC 对不上）时应当丢弃这一帧并重新同步，
        # 而不是把它当成遥测帧，也不能因此丢掉后面的好帧。
        damaged = bytearray(pose)
        damaged[8] ^= 1
        parser = TelemetryParser()
        self.assertEqual(parser.feed(bytes(damaged) + pose), [decoded])
        self.assertEqual(parser.bad_frames, 1)

    def test_vehicle_encoder_matches_decoder(self):
        # 用固件产出的真实字节验证解码，防止两边布局悄悄跑偏
        decoded = decode_nav_pose(VEHICLE_POSE_FRAME)
        self.assertEqual(decoded.x_m, -1.234)
        self.assertEqual(decoded.y_m, 0.567)
        self.assertEqual(decoded.heading_deg, 90.0)
        self.assertEqual(decoded.path_length_m, 2.345)
        self.assertEqual(decoded.yaw_rate_rad_s, -0.125)
        self.assertTrue(decoded.nav_active)
        self.assertEqual(decoded.nav_target_speed_m_s, 0.45)
        self.assertEqual(decoded.nav_target_yaw_deg, -12.34)
        self.assertEqual(decoded.nav_cross_track_m, -0.08)
        self.assertEqual(decoded.nav_target_index, 3)
        self.assertEqual(decoded.valid, POSE_VALID_DATA | POSE_VALID_NAVIGATION
                         | POSE_VALID_TRACKING)
        self.assertEqual(sample_pose_frame(), VEHICLE_POSE_FRAME)

    def test_sequence_tracker_counts_missing_frames(self):
        tracker = SequenceTracker()
        # 三种帧共用一个序号，正常时序号连续，不应报丢帧
        for sequence in (0, 1, 2, 3, 4, 5, 6):
            self.assertEqual(tracker.update(sequence), 0)
        self.assertEqual(tracker.lost, 0)
        # 少一帧（序号 7 没收到）：下一帧 8 应当报丢失 1
        self.assertEqual(tracker.update(8), 1)
        self.assertEqual(tracker.lost, 1)
        # 一次丢三帧
        self.assertEqual(tracker.update(12), 3)
        self.assertEqual(tracker.lost, 4)
        # 序号回绕：255 之后是 0，不算丢帧
        tracker.reset()
        self.assertEqual(tracker.update(255), 0)
        # 回绕处丢一帧（255 之后应当是 0，实际收到 1）
        self.assertEqual(tracker.update(1), 1)
        self.assertEqual(tracker.lost, 1)
        # 间隔过大说明刚重新同步或换了连接，不当作丢帧
        self.assertEqual(tracker.update(200), 0)
        self.assertEqual(tracker.lost, 1)


if __name__ == "__main__":
    unittest.main()

import struct
import unittest

from protocol import (
    BALANCE, FEEDFORWARD, MOTION, PID_BASE, STATE, STEERING_FEEDFORWARD,
    SUPPORT, TELEMETRY_LENGTH, TelemetryParser, command_frame, crc16,
    decode_telemetry, feedforward_frame, pid_frame,
    steering_feedforward_frame,
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


if __name__ == "__main__":
    unittest.main()

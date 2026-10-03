"""UART8 wire format shared by the vehicle monitor and controller."""

from dataclasses import dataclass
import struct
import math

COMMAND_SYNC = b"\xa5\x5a"
TELEMETRY_SYNC = b"\x5a\xa5"
TELEMETRY_LENGTH = 52
VERSION = 1
TYPE_TELEMETRY = 0x80
HEARTBEAT = 0
STATE = 1
SPEED = 2
YAW = 3
MOTION = 4
PID_BASE = 0x50
PID_LAST = 0x58
PROTECT = 0
BALANCE = 1
NAVIGATION = 2
SUPPORT = 3
VALID_STATE = 1
VALID_ATTITUDE = 2
VALID_IMU = 4
VALID_MOTION = 8
VALID_CONTROL = 16


def pid_frame(controller: int, gain: int, value: float) -> bytes:
    if controller not in (0, 1, 2) or gain not in (0, 1, 2):
        raise ValueError("unknown PID controller or gain")
    if not math.isfinite(value) or not 0.0 <= value <= 10000.0:
        raise ValueError("PID gain must be between 0 and 10000")
    command = PID_BASE + controller * 3 + gain
    body = bytes((command,)) + struct.pack("<f", value)
    checksum = 0
    for byte in body:
        checksum ^= byte
    return COMMAND_SYNC + body + bytes((checksum,))


def command_frame(command: int, value: int = 0, value2: int = 0) -> bytes:
    if command not in (HEARTBEAT, STATE, SPEED, YAW, MOTION):
        raise ValueError("unknown command")
    if command != MOTION and value2 != 0:
        raise ValueError("reserved value must be zero")
    if command == STATE and value not in (PROTECT, BALANCE, NAVIGATION, SUPPORT):
        raise ValueError("invalid state")
    if not (-32768 <= value <= 32767 and -32768 <= value2 <= 32767):
        raise ValueError("command value outside int16")
    body = struct.pack("<Bhh", command, value, value2)
    checksum = 0
    for byte in body:
        checksum ^= byte
    return COMMAND_SYNC + body + bytes((checksum,))


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Telemetry:
    sequence: int
    valid: int
    uptime_ms: int
    control_mode: int
    vehicle_mode: int
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    pitch_rate_dps: float
    gyro_x_dps: float
    gyro_y_dps: float
    gyro_z_dps: float
    acc_x_g: float
    acc_y_g: float
    acc_z_g: float
    left_speed_m_s: float
    right_speed_m_s: float
    left_delta_count: int
    right_delta_count: int
    left_pwm: int
    right_pwm: int
    target_speed_m_s: float
    target_yaw_deg: float
    target_pitch_deg: float


def decode_telemetry(frame: bytes) -> Telemetry:
    if len(frame) != TELEMETRY_LENGTH or frame[:2] != TELEMETRY_SYNC:
        raise ValueError("invalid frame boundary")
    if frame[2:4] != bytes((VERSION, TYPE_TELEMETRY)):
        raise ValueError("unsupported telemetry version or type")
    if crc16(frame[:-2]) != struct.unpack_from("<H", frame, 50)[0]:
        raise ValueError("CRC mismatch")
    values = struct.unpack_from("<19h", frame, 12)
    return Telemetry(
        sequence=frame[4], valid=frame[5],
        uptime_ms=struct.unpack_from("<I", frame, 6)[0],
        control_mode=frame[10], vehicle_mode=frame[11],
        roll_deg=values[0] / 100, pitch_deg=values[1] / 100,
        yaw_deg=values[2] / 100, pitch_rate_dps=values[3] / 100,
        gyro_x_dps=values[4] / 100, gyro_y_dps=values[5] / 100,
        gyro_z_dps=values[6] / 100,
        acc_x_g=values[7] / 1000, acc_y_g=values[8] / 1000,
        acc_z_g=values[9] / 1000,
        left_speed_m_s=values[10] / 1000,
        right_speed_m_s=values[11] / 1000,
        left_delta_count=values[12], right_delta_count=values[13],
        left_pwm=values[14], right_pwm=values[15],
        target_speed_m_s=values[16] / 1000,
        target_yaw_deg=values[17] / 100,
        target_pitch_deg=values[18] / 100,
    )


class TelemetryParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data: bytes) -> list[Telemetry]:
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 2:
            start = self.buffer.find(TELEMETRY_SYNC)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == TELEMETRY_SYNC[0] else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < TELEMETRY_LENGTH:
                break
            try:
                frame = decode_telemetry(self.buffer[:TELEMETRY_LENGTH])
            except ValueError:
                self.bad_frames += 1
                del self.buffer[0]
                continue
            frames.append(frame)
            del self.buffer[:TELEMETRY_LENGTH]
        return frames

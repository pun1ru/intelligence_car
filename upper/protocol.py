"""车端与上位机之间的 UART8 线上格式（下行命令帧 + 三种上行帧）。

上行帧以 "5A A5" 开头，第 3 字节是类型：
    0x80 车辆遥测 52 字节、0x81 校准 24 字节、0x82 导航位姿 18 字节。
车端每 100 ms 依次发这三帧，所以解析器必须能在一段字节流里按类型切帧；
解析失败时按单字节跳过重新同步，见 TelemetryParser。
"""

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
FEEDFORWARD = 5
CALIBRATE = 6
STEERING = 7
STEERING_FEEDFORWARD = 8
PID_BASE = 0x50
PID_LAST = 0x5B
PID_STEERING = 3
PROTECT = 0
BALANCE = 1
NAVIGATION = 2
SUPPORT = 3
VALID_STATE = 1
VALID_ATTITUDE = 2
VALID_IMU = 4
VALID_MOTION = 8
VALID_CONTROL = 16
# Vehicle-side lift/airborne diagnostic flag.  The fixed telemetry frame keeps
# this in the valid bit field so older firmware remains wire-compatible.
VALID_LIFT = 32
CALIBRATION_LENGTH = 24
CALIBRATION_TYPE = 0x81
# 导航位姿单独占一种帧类型，这样 52 字节定长遥测帧不用改动，旧固件与
# 旧上位机仍然兼容（对应车端 general_define.h 的 SERIAL_NAV_POSE_*）。
NAV_POSE_TYPE = 0x82
NAV_POSE_LENGTH = 25
# 位姿帧 valid 字节的位：bit0 数值可信、bit1 发送时车辆处于导航状态
POSE_VALID_DATA = 1
POSE_VALID_NAVIGATION = 2
# bit2：本帧带了有效的路径跟踪指令
POSE_VALID_TRACKING = 4


def pid_frame(controller: int, gain: int, value: float) -> bytes:
    if controller not in (0, 1, 2, 3) or gain not in (0, 1, 2):
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
    if command not in (HEARTBEAT, STATE, SPEED, YAW, MOTION,
                       FEEDFORWARD, CALIBRATE, STEERING, STEERING_FEEDFORWARD):
        raise ValueError("unknown command")
    if command != MOTION and value2 != 0:
        raise ValueError("reserved value must be zero")
    if command == STATE and value not in (PROTECT, BALANCE, NAVIGATION, SUPPORT):
        raise ValueError("invalid state")
    if command == CALIBRATE and (value != 0 or value2 != 0):
        raise ValueError("calibration command takes no value")
    if command == STEERING and (value not in (0, 1) or value2 != 0):
        raise ValueError("invalid steering enable")
    if command == FEEDFORWARD and (value < 0 or value2 != 0):
        raise ValueError("invalid feedforward value")
    if command == STEERING_FEEDFORWARD and (value < 0 or value2 != 0):
        raise ValueError("invalid steering feedforward value")
    if not (-32768 <= value <= 32767 and -32768 <= value2 <= 32767):
        raise ValueError("command value outside int16")
    body = struct.pack("<Bhh", command, value, value2)
    checksum = 0
    for byte in body:
        checksum ^= byte
    return COMMAND_SYNC + body + bytes((checksum,))


def feedforward_frame(value: float) -> bytes:
    if not math.isfinite(value) or not 0.0 <= value <= 10000.0:
        raise ValueError("wheel feedforward gain must be between 0 and 10000")
    return command_frame(FEEDFORWARD, round(value))


def calibration_frame() -> bytes:
    return command_frame(CALIBRATE)


def steering_frame(enabled: bool) -> bytes:
    return command_frame(STEERING, 1 if enabled else 0)


def steering_feedforward_frame(value: float) -> bytes:
    if not math.isfinite(value) or not 0.0 <= value <= 1000.0:
        raise ValueError("steering feedforward gain must be between 0 and 1000")
    return command_frame(STEERING_FEEDFORWARD, round(value))


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


@dataclass(frozen=True)
class CalibrationTelemetry:
    sequence: int
    active: bool
    complete: bool
    sample_count: int
    gyro_bias_x_dps: float
    gyro_bias_y_dps: float
    gyro_bias_z_dps: float


@dataclass(frozen=True)
class NavPoseTelemetry:
    """0x82 帧：车端算出的位姿，字段语义见 docs/VEHICLE_CONVENTIONS.md。"""
    sequence: int
    valid: int             # POSE_VALID_* 位的或
    x_m: float             # 相对原点前后位移，+X 向前
    y_m: float             # 相对原点左右位移，+Y 向左
    heading_deg: float     # 相对原点航向偏角，[-180, 180]
    path_length_m: float   # 累计行程，用来单独核对里程标定
    yaw_rate_rad_s: float  # 编码器推算角速度，可与遥测帧的 IMU yaw 速率对比
    nav_active: bool       # bit2 为 1 时，下面四项才是当前有效的跟踪指令
    nav_target_speed_m_s: float  # 路径跟踪的目标速度
    nav_target_yaw_deg: float    # 路径跟踪的目标航向（IMU 坐标系）
    nav_cross_track_m: float     # 到路径的横向偏差，正=在路径左侧
    nav_target_index: int        # 当前前视点在路径表中的序号


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


def decode_calibration(frame: bytes) -> CalibrationTelemetry:
    if len(frame) != CALIBRATION_LENGTH or frame[:2] != TELEMETRY_SYNC:
        raise ValueError("invalid calibration frame boundary")
    if frame[2:4] != bytes((VERSION, CALIBRATION_TYPE)):
        raise ValueError("unsupported calibration version or type")
    if crc16(frame[:-2]) != struct.unpack_from("<H", frame, 22)[0]:
        raise ValueError("calibration CRC mismatch")
    flags = frame[5]
    return CalibrationTelemetry(
        sequence=frame[4], active=bool(flags & 1), complete=bool(flags & 2),
        sample_count=struct.unpack_from("<I", frame, 6)[0],
        gyro_bias_x_dps=struct.unpack_from("<h", frame, 10)[0] / 100,
        gyro_bias_y_dps=struct.unpack_from("<h", frame, 12)[0] / 100,
        gyro_bias_z_dps=struct.unpack_from("<h", frame, 14)[0] / 100,
    )


def decode_nav_pose(frame: bytes) -> NavPoseTelemetry:
    """解码 0x82 帧。校验边界、版本/类型与 CRC，任一项不符就抛 ValueError，
    由 TelemetryParser 记一次坏帧并继续找下一帧。"""
    if len(frame) != NAV_POSE_LENGTH or frame[:2] != TELEMETRY_SYNC:
        raise ValueError("invalid navigation pose frame boundary")
    if frame[2:4] != bytes((VERSION, NAV_POSE_TYPE)):
        raise ValueError("unsupported navigation pose version or type")
    if crc16(frame[:-2]) != struct.unpack_from("<H", frame, 23)[0]:
        raise ValueError("navigation pose CRC mismatch")
    (x_mm, y_mm, heading, path_mm, yaw_rate,
     nav_speed, nav_yaw, nav_cross) = struct.unpack_from("<8h", frame, 6)
    return NavPoseTelemetry(
        sequence=frame[4], valid=frame[5],
        x_m=x_mm / 1000, y_m=y_mm / 1000, heading_deg=heading / 100,
        path_length_m=path_mm / 1000, yaw_rate_rad_s=yaw_rate / 1000,
        nav_active=bool(frame[5] & POSE_VALID_TRACKING),
        nav_target_speed_m_s=nav_speed / 1000,
        nav_target_yaw_deg=nav_yaw / 100,
        nav_cross_track_m=nav_cross / 1000,
        nav_target_index=frame[22],
    )


# 帧类型 -> (线上长度, 解码函数)。遇到未知类型时退回按遥测帧长度处理，
# 这样它会在 CRC 校验处失败，解析器于是按单字节继续找下一个帧头。
FRAME_FORMATS = {
    TYPE_TELEMETRY: (TELEMETRY_LENGTH, decode_telemetry),
    CALIBRATION_TYPE: (CALIBRATION_LENGTH, decode_calibration),
    NAV_POSE_TYPE: (NAV_POSE_LENGTH, decode_nav_pose),
}


class SequenceTracker:
    """统计上行帧的丢帧数。

    三种上行帧（遥测、校准、位姿）共用同一个序号，车端每 100 ms 依次发一帧，
    所以序号在整条流上是连续的：n、n+1、n+2、n+3…… 因此必须对**每一帧**都
    调用 update()。只对遥测帧计数会把中间的校准帧和位姿帧当成丢帧，链路完好
    也会一直报丢失。
    """

    def __init__(self) -> None:
        self.last_sequence: int | None = None
        self.lost = 0

    def reset(self) -> None:
        self.last_sequence = None
        self.lost = 0

    def update(self, sequence: int) -> int:
        """喂入一帧的序号，返回这一帧之前缺了几帧（0 表示连续）。"""
        gap = 0
        if self.last_sequence is not None:
            gap = (sequence - self.last_sequence - 1) & 0xFF
            # 间隔大于半圈的差值无法判断是我们丢帧还是刚重新同步，忽略
            if gap < 128:
                self.lost += gap
            else:
                gap = 0
        self.last_sequence = sequence
        return gap


class TelemetryParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad_frames = 0

    def feed(self, data: bytes) -> list[Telemetry | CalibrationTelemetry | NavPoseTelemetry]:
        """喂入串口收到的任意长度字节，返回本次解析出的完整帧列表。

        串口数据是流式的，可能一帧都没凑齐、也可能一次收到好几帧，
        所以这里维护一个缓冲区：先找到帧头，再按类型取够长度才解码。
        """
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 2:
            # 丢弃帧头之前的杂字节；若结尾恰好是帧头前半字节则保留，等下批数据
            start = self.buffer.find(TELEMETRY_SYNC)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == TELEMETRY_SYNC[0] else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 4:
                break
            frame_length, decoder = FRAME_FORMATS.get(
                self.buffer[3], (TELEMETRY_LENGTH, decode_telemetry))
            if len(self.buffer) < frame_length:
                break
            try:
                frame = decoder(bytes(self.buffer[:frame_length]))
            except ValueError:
                # 校验失败：只前进一个字节，避免整段错位后一直丢帧
                self.bad_frames += 1
                del self.buffer[0]
                continue
            frames.append(frame)
            del self.buffer[:frame_length]
        return frames

"""Wireless UART vehicle console. Run with: python upper/main.py"""

import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports

from protocol import (
    BALANCE, HEARTBEAT, MOTION, NAVIGATION, PROTECT, STATE, SUPPORT, VALID_ATTITUDE,
    VALID_CONTROL, VALID_IMU, VALID_MOTION, VALID_STATE, Telemetry, TelemetryParser,
    command_frame, pid_frame,
)

BAUD = 115200
POLL_MS = 50
HEARTBEAT_MS = 100
STALE_S = 0.5
MODES = ("按钮调试", "串口调试", "校准")
STATES = ("保护", "平衡", "导航", "支撑平衡")


class Console(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("平衡车无线控制台")
        self.geometry("850x670")
        self.minsize(750, 600)
        self.port: serial.Serial | None = None
        self.parser = TelemetryParser()
        self.last_frame_at = 0.0
        self.last_heartbeat_at = 0.0
        self.last_sequence: int | None = None
        self.last_protect_at = 0.0
        self.lost_frames = 0
        self.status = tk.StringVar(value="未连接")
        self.port_name = tk.StringVar()
        self.speed = tk.StringVar(value="0.000")
        self.yaw = tk.StringVar(value="0.00")
        self.pid_controller = tk.StringVar(value="Angle loop")
        self.pid_gain = tk.StringVar(value="Kp")
        self.pid_value = tk.StringVar(value="600.0")
        self.pid_values = {
            ("Wheel speed loop", "Kp"): 5000.0,
            ("Wheel speed loop", "Ki"): 0.0,
            ("Wheel speed loop", "Kd"): 100.0,
            ("Angle loop", "Kp"): 600.0,
            ("Angle loop", "Ki"): 0.0,
            ("Angle loop", "Kd"): 30.0,
            ("Speed loop", "Kp"): 14.0,
            ("Speed loop", "Ki"): 3.0,
            ("Speed loop", "Kd"): 0.0,
        }
        self.metrics: dict[str, tk.StringVar] = {}
        self._build()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.after(POLL_MS, self.poll)

    def _build(self) -> None:
        root = ttk.Frame(self, padding=14)
        root.pack(fill="both", expand=True)
        root.columnconfigure(0, weight=1)

        connection = ttk.LabelFrame(root, text="连接", padding=10)
        connection.grid(row=0, column=0, sticky="ew")
        self.ports = ttk.Combobox(connection, textvariable=self.port_name, width=28)
        self.ports.grid(row=0, column=0, padx=(0, 8))
        ttk.Button(connection, text="刷新串口", command=self.refresh_ports).grid(row=0, column=1, padx=4)
        self.connect_button = ttk.Button(connection, text="连接", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=4)
        ttk.Label(connection, textvariable=self.status).grid(row=0, column=3, padx=16)

        state = ttk.LabelFrame(root, text="车辆状态", padding=10)
        state.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        for col in range(4):
            state.columnconfigure(col, weight=1)
        for col, (key, label) in enumerate((
            ("mode", "控制模式"), ("state", "机体状态"),
            ("uptime", "运行时间"), ("quality", "遥测质量"),
        )):
            ttk.Label(state, text=label).grid(row=0, column=col, sticky="w")
            self.metrics[key] = tk.StringVar(value="--")
            ttk.Label(state, textvariable=self.metrics[key], font=("Segoe UI", 13, "bold"))\
                .grid(row=1, column=col, sticky="w", pady=(3, 0))

        control = ttk.LabelFrame(root, text="控制", padding=10)
        control.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        ttk.Button(control, text="进入平衡", command=lambda: self.send_state(BALANCE)).grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(control, text="进入导航", command=lambda: self.send_state(NAVIGATION)).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(control, text="急停 / 保护", command=self.emergency_stop).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(control, text="进入支撑平衡", command=lambda: self.send_state(SUPPORT)).grid(row=0, column=3, padx=4, pady=4)
        ttk.Label(control, text="速度目标 (m/s)").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(control, textvariable=self.speed, width=12).grid(row=1, column=1, sticky="w", pady=(10, 0))
        ttk.Label(control, text="Yaw 目标 (deg)").grid(row=2, column=0, sticky="w")
        ttk.Entry(control, textvariable=self.yaw, width=12).grid(row=2, column=1, sticky="w")
        ttk.Button(control, text="发送运动目标", command=self.send_motion).grid(row=1, column=2, rowspan=2, padx=4)
        ttk.Button(control, text="速度归零", command=self.stop_motion).grid(row=1, column=3, rowspan=2, padx=4)
        ttk.Label(control, text="Yaw 目标目前仅记录，车端尚未实现转向闭环。")\
            .grid(row=3, column=0, columnspan=4, sticky="w", pady=(8, 0))

        pid = ttk.LabelFrame(root, text="Online PID tuning", padding=10)
        pid.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        ttk.Label(pid, text="Loop").grid(row=0, column=0, sticky="w")
        self.pid_controller_box = ttk.Combobox(
            pid, textvariable=self.pid_controller, state="readonly", width=20,
            values=("Wheel speed loop", "Angle loop", "Speed loop"))
        self.pid_controller_box.grid(row=0, column=1, padx=5)
        ttk.Label(pid, text="Gain").grid(row=0, column=2, sticky="w")
        self.pid_gain_box = ttk.Combobox(
            pid, textvariable=self.pid_gain, state="readonly", width=8,
            values=("Kp", "Ki", "Kd"))
        self.pid_gain_box.grid(row=0, column=3, padx=5)
        ttk.Label(pid, text="Value").grid(row=0, column=4, sticky="w")
        ttk.Entry(pid, textvariable=self.pid_value, width=12).grid(row=0, column=5, padx=5)
        ttk.Button(pid, text="Apply PID (protect)", command=self.send_pid).grid(row=0, column=6, padx=8)
        ttk.Label(pid, text="Every applied PID value first enters protection and clears motor output.")\
            .grid(row=1, column=0, columnspan=7, sticky="w", pady=(7, 0))
        self.pid_controller_box.bind("<<ComboboxSelected>>", self.update_pid_value)
        self.pid_gain_box.bind("<<ComboboxSelected>>", self.update_pid_value)

        readings = ttk.Frame(root)
        readings.grid(row=4, column=0, sticky="nsew", pady=(10, 0))
        readings.columnconfigure(0, weight=1)
        readings.columnconfigure(1, weight=1)
        root.rowconfigure(4, weight=1)
        self._readout(readings, 0, "姿态与 IMU", (
            ("roll", "Roll", "deg"), ("pitch", "Pitch", "deg"),
            ("yaw_actual", "Yaw", "deg"), ("pitch_rate", "Pitch 角速度", "deg/s"),
            ("gx", "Gyro X", "deg/s"), ("gy", "Gyro Y", "deg/s"),
            ("gz", "Gyro Z", "deg/s"), ("ax", "Accel X", "g"),
            ("ay", "Accel Y", "g"), ("az", "Accel Z", "g"),
        ))
        self._readout(readings, 1, "运动与输出", (
            ("left_speed", "左轮速度", "m/s"), ("right_speed", "右轮速度", "m/s"),
            ("left_count", "左轮增量", "count/10ms"),
            ("right_count", "右轮增量", "count/10ms"),
            ("left_pwm", "左电机 PWM", "/10000"),
            ("right_pwm", "右电机 PWM", "/10000"),
            ("target_speed", "当前目标速度", "m/s"),
            ("target_yaw", "当前目标 Yaw", "deg"),
            ("target_pitch", "当前目标 Pitch", "deg"),
        ))
        self.notice = tk.StringVar(value="连接后查看实时数据。")
        ttk.Label(root, textvariable=self.notice).grid(row=5, column=0, sticky="w", pady=(8, 0))

    def _readout(self, parent, column, title, rows) -> None:
        box = ttk.LabelFrame(parent, text=title, padding=10)
        box.grid(row=0, column=column, sticky="nsew", padx=(0, 6) if column == 0 else (6, 0))
        box.columnconfigure(1, weight=1)
        for index, (key, label, unit) in enumerate(rows):
            ttk.Label(box, text=label).grid(row=index, column=0, sticky="w", pady=3)
            variable = tk.StringVar(value="--")
            self.metrics[key] = variable
            ttk.Label(box, textvariable=variable, font=("Consolas", 11))\
                .grid(row=index, column=1, sticky="e", padx=8)
            ttk.Label(box, text=unit).grid(row=index, column=2, sticky="w")

    def refresh_ports(self) -> None:
        ports = [item.device for item in list_ports.comports()]
        self.ports["values"] = ports
        if ports and self.port_name.get() not in ports:
            self.port_name.set(ports[0])

    def toggle_connection(self) -> None:
        if self.port is not None:
            self.disconnect()
            return
        try:
            self.port = serial.Serial(self.port_name.get(), BAUD, timeout=0, write_timeout=0.2)
        except (serial.SerialException, ValueError) as exc:
            messagebox.showerror("串口连接失败", str(exc))
            return
        self.parser = TelemetryParser()
        self.last_sequence = None
        self.lost_frames = 0
        self.last_frame_at = 0.0
        self.last_heartbeat_at = 0.0
        self.last_protect_at = 0.0
        self.connect_button.configure(text="断开")
        self.status.set("等待遥测")

    def disconnect(self) -> None:
        if self.port is not None:
            try:
                self.port.write(command_frame(STATE, PROTECT))
            except serial.SerialException:
                pass
            self.port.close()
            self.port = None
        self.connect_button.configure(text="连接")
        self.status.set("未连接")
        self.notice.set("已断开；车辆应进入保护状态。")

    def send(self, command: int, value: int = 0, value2: int = 0) -> None:
        if self.port is None:
            self.notice.set("先连接串口。")
            return
        try:
            self.port.write(command_frame(command, value, value2))
        except (serial.SerialException, serial.SerialTimeoutException) as exc:
            self.notice.set(f"发送失败: {exc}")
            self.disconnect()

    def send_state(self, target: int) -> None:
        if target != PROTECT and not self.telemetry_online():
            self.notice.set("遥测未在线，无法确认车体姿态。")
            return
        self.send(STATE, target)
        self.notice.set("状态请求已发送；以车端回传的状态为准。")

    def emergency_stop(self) -> None:
        self.speed.set("0.000")
        self.send(STATE, PROTECT)
        self.notice.set("已请求保护；等待车端状态回传。")

    def send_motion(self) -> None:
        if not self.telemetry_online():
            self.notice.set("遥测未在线，运动目标未发送。")
            return
        try:
            speed = float(self.speed.get())
            yaw = float(self.yaw.get())
            if not (-1.0 <= speed <= 1.0 and -180.0 <= yaw <= 180.0):
                raise ValueError("速度范围 ±1 m/s，Yaw 范围 ±180 deg")
            self.send(MOTION, round(speed * 1000), round(yaw * 100))
            self.notice.set("运动目标已发送；检查车端目标值回传。")
        except ValueError as exc:
            messagebox.showerror("目标值无效", str(exc))

    def stop_motion(self) -> None:
        self.speed.set("0.000")
        if self.telemetry_online():
            self.send_motion()
        else:
            self.emergency_stop()

    def update_pid_value(self, _event=None) -> None:
        key = (self.pid_controller.get(), self.pid_gain.get())
        if key in self.pid_values:
            self.pid_value.set(f"{self.pid_values[key]:g}")

    def send_pid(self) -> None:
        if not self.telemetry_online():
            self.notice.set("遥测未在线，PID 参数未发送。")
            return
        controller_names = ("Wheel speed loop", "Angle loop", "Speed loop")
        gain_names = ("Kp", "Ki", "Kd")
        try:
            controller = controller_names.index(self.pid_controller.get())
            gain = gain_names.index(self.pid_gain.get())
            value = float(self.pid_value.get())
            if not 0.0 <= value <= 10000.0:
                raise ValueError("PID gain range is 0 to 10000")
            self.port.write(pid_frame(controller, gain, value))
            self.pid_values[(self.pid_controller.get(), self.pid_gain.get())] = value
            self.notice.set("PID 已发送；车端收到后会进入保护，请确认遥测状态再重新发送平衡/导航。")
        except (ValueError, serial.SerialException, serial.SerialTimeoutException) as exc:
            self.notice.set(f"PID 发送失败: {exc}")
            if isinstance(exc, (serial.SerialException, serial.SerialTimeoutException)):
                self.disconnect()

    def telemetry_online(self) -> bool:
        return self.port is not None and self.last_frame_at > 0 and time.monotonic() - self.last_frame_at <= STALE_S

    def display(self, frame: Telemetry) -> None:
        if self.last_sequence is not None:
            gap = (frame.sequence - self.last_sequence - 1) & 0xFF
            if gap < 128:
                self.lost_frames += gap
        self.last_sequence = frame.sequence
        self.metrics["quality"].set(f"丢 {self.lost_frames} / 错 {self.parser.bad_frames}")
        self.metrics["uptime"].set(f"{frame.uptime_ms / 1000:.1f} s")
        self.metrics["mode"].set(MODES[frame.control_mode] if frame.valid & VALID_STATE and frame.control_mode < len(MODES) else "--")
        self.metrics["state"].set(STATES[frame.vehicle_mode] if frame.valid & VALID_STATE and frame.vehicle_mode < len(STATES) else "--")
        values = {
            "roll": frame.roll_deg, "pitch": frame.pitch_deg, "yaw_actual": frame.yaw_deg,
            "pitch_rate": frame.pitch_rate_dps, "gx": frame.gyro_x_dps,
            "gy": frame.gyro_y_dps, "gz": frame.gyro_z_dps,
            "ax": frame.acc_x_g, "ay": frame.acc_y_g, "az": frame.acc_z_g,
            "left_speed": frame.left_speed_m_s, "right_speed": frame.right_speed_m_s,
            "left_count": frame.left_delta_count, "right_count": frame.right_delta_count,
            "left_pwm": frame.left_pwm, "right_pwm": frame.right_pwm,
            "target_speed": frame.target_speed_m_s, "target_yaw": frame.target_yaw_deg,
            "target_pitch": frame.target_pitch_deg,
        }
        for key, value in values.items():
            required = (VALID_ATTITUDE if key in ("roll", "pitch", "yaw_actual", "pitch_rate")
                        else VALID_IMU if key in ("gx", "gy", "gz", "ax", "ay", "az")
                        else VALID_MOTION if key in ("left_speed", "right_speed", "left_count", "right_count")
                        else VALID_STATE if key in ("target_speed", "target_yaw")
                        else VALID_CONTROL)
            self.metrics[key].set("--" if required and not frame.valid & required else
                                  f"{value:.3f}" if isinstance(value, float) else str(value))

    def poll(self) -> None:
        now = time.monotonic()
        if self.port is not None:
            try:
                if now - self.last_heartbeat_at >= HEARTBEAT_MS / 1000:
                    self.port.write(command_frame(HEARTBEAT))
                    self.last_heartbeat_at = now
                data = self.port.read(min(self.port.in_waiting, 4096))
                for frame in self.parser.feed(data):
                    self.display(frame)
                    self.last_frame_at = now
                if self.last_frame_at > 0 and now - self.last_frame_at > STALE_S and now - self.last_protect_at >= HEARTBEAT_MS / 1000:
                    self.port.write(command_frame(STATE, PROTECT))
                    self.last_protect_at = now
                    self.notice.set("遥测超时，持续请求保护。")
                self.status.set("遥测在线" if self.telemetry_online() else "遥测超时")
            except (serial.SerialException, serial.SerialTimeoutException) as exc:
                self.notice.set(f"通信中断: {exc}")
                self.disconnect()
        self.after(POLL_MS, self.poll)

    def close(self) -> None:
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    Console().mainloop()

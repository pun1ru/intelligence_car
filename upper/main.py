"""Wireless UART monitor, controller, plots, and trajectory viewer."""

from __future__ import annotations

from collections import deque
import math
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial
from serial.tools import list_ports

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure
except ImportError:
    Figure = None
    FigureCanvasTkAgg = None

from protocol import (
    BALANCE, HEARTBEAT, MOTION, NAVIGATION, PROTECT, STATE, SUPPORT,
    VALID_ATTITUDE, VALID_CONTROL, VALID_IMU, VALID_LIFT, VALID_MOTION,
    VALID_STATE, CalibrationTelemetry, Telemetry, TelemetryParser,
    calibration_frame, command_frame, feedforward_frame, pid_frame,
    steering_feedforward_frame, steering_frame,
)

BAUD = 115200
POLL_MS = 50
HEARTBEAT_MS = 100
KEYBOARD_PERIOD_MS = 100
STALE_S = 0.5
HISTORY_LENGTH = 600
MODES = ("Button", "Serial", "Calibration")
STATES = ("Protect", "Balance", "Navigation", "Support")
CHARTS = (
    "Vehicle speed",
    "Pitch target / actual",
    "Wheel speed / target",
    "IMU Euler angles",
)


def wrap_angle_deg(angle: float) -> float:
    """Return an angle in [-180, 180], preserving the shortest turn command."""
    return (angle + 180.0) % 360.0 - 180.0


class Console(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Two-wheel balance vehicle console")
        self.geometry("1180x820")
        self.minsize(980, 700)
        self.port: serial.Serial | None = None
        self.parser = TelemetryParser()
        self.last_frame_at = 0.0
        self.last_heartbeat_at = 0.0
        self.last_sequence: int | None = None
        self.last_protect_at = 0.0
        self.lost_frames = 0
        self.latest_frame: Telemetry | None = None

        self.status = tk.StringVar(value="Disconnected")
        self.port_name = tk.StringVar()
        self.speed = tk.StringVar(value="0.000")
        self.yaw = tk.StringVar(value="0.00")
        self.feedforward = tk.StringVar(value="0.0")
        self.steering_feedforward = tk.StringVar(value="10.0")
        self.steering_enabled = tk.BooleanVar(value=False)
        self.keyboard_enabled = tk.BooleanVar(value=False)
        self.keyboard_speed = tk.StringVar(value="0.20")
        self.keyboard_yaw_rate = tk.StringVar(value="100.0")
        self.keyboard_yaw = 0.0
        self.pressed_keys: set[str] = set()
        self.keyboard_last_tick = time.monotonic()

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
            ("Speed loop", "Kp"): 16.0,
            ("Speed loop", "Ki"): 3.0,
            ("Speed loop", "Kd"): 0.0,
            ("Steering loop", "Kp"): 100.0,
            ("Steering loop", "Ki"): 2.0,
            ("Steering loop", "Kd"): 0.0,
        }

        self.metrics: dict[str, tk.StringVar] = {}
        self.samples: deque[dict[str, float | None]] = deque(maxlen=HISTORY_LENGTH)
        self.history_start_s: float | None = None
        self.trajectory_points: deque[tuple[float, float]] = deque(
            [(0.0, 0.0)], maxlen=HISTORY_LENGTH * 4)
        self.trajectory_x = 0.0
        self.trajectory_y = 0.0
        self.last_trajectory_uptime_ms: int | None = None
        self.trajectory_heading_zero_deg: float | None = None
        self.plot_choice = tk.StringVar(value=CHARTS[0])
        self.chart_readout_names: list[tk.StringVar] = []
        self.chart_readout_values: list[tk.StringVar] = []
        self.chart_readout_time = tk.StringVar(value="--")

        self._build()
        self._bind_keyboard()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.after(POLL_MS, self.poll)
        self.after(KEYBOARD_PERIOD_MS, self._keyboard_tick)

    def _build(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill="both", expand=True)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(4, weight=1)

        connection = ttk.LabelFrame(root, text="Connection", padding=8)
        connection.grid(row=0, column=0, sticky="ew")
        self.ports = ttk.Combobox(connection, textvariable=self.port_name, width=22)
        self.ports.grid(row=0, column=0, padx=(0, 6))
        ttk.Button(connection, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=1, padx=3)
        self.connect_button = ttk.Button(
            connection, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=3)
        ttk.Label(connection, textvariable=self.status).grid(row=0, column=3, padx=14)

        state = ttk.LabelFrame(root, text="Vehicle state", padding=8)
        state.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        for col in range(8):
            state.columnconfigure(col, weight=1)
        for col, (key, label) in enumerate((
            ("mode", "Control mode"), ("state", "Vehicle mode"),
            ("lift", "Lift diagnostic"), ("vehicle_speed", "Vehicle speed"),
            ("state_pitch", "Pitch"), ("state_yaw", "Yaw"),
            ("state_pwm", "PWM L/R"), ("quality", "Telemetry"),
        )):
            ttk.Label(state, text=label).grid(row=0, column=col, sticky="w")
            self.metrics[key] = tk.StringVar(value="--")
            ttk.Label(state, textvariable=self.metrics[key],
                      font=("Segoe UI", 11, "bold")).grid(
                          row=1, column=col, sticky="w", pady=(2, 0))

        control = ttk.LabelFrame(root, text="Motion control", padding=8)
        control.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        ttk.Button(control, text="Balance", command=lambda: self.send_state(BALANCE)).grid(
            row=0, column=0, padx=3, pady=3)
        ttk.Button(control, text="Navigation", command=lambda: self.send_state(NAVIGATION)).grid(
            row=0, column=1, padx=3, pady=3)
        ttk.Button(control, text="Protect", command=self.emergency_stop).grid(
            row=0, column=2, padx=3, pady=3)
        ttk.Button(control, text="Support", command=lambda: self.send_state(SUPPORT)).grid(
            row=0, column=3, padx=3, pady=3)
        ttk.Label(control, text="Target speed (m/s)").grid(row=1, column=0, sticky="w")
        ttk.Entry(control, textvariable=self.speed, width=10).grid(row=1, column=1, sticky="w")
        ttk.Label(control, text="Target yaw (deg)").grid(row=1, column=2, sticky="w")
        ttk.Entry(control, textvariable=self.yaw, width=10).grid(row=1, column=3, sticky="w")
        ttk.Button(control, text="Send motion", command=self.send_motion).grid(
            row=1, column=4, padx=5)
        ttk.Button(control, text="Stop motion", command=self.stop_motion).grid(
            row=1, column=5, padx=5)
        ttk.Checkbutton(control, text="Enable steering loop",
                        variable=self.steering_enabled,
                        command=self.toggle_steering).grid(row=2, column=0, columnspan=2,
                                                           sticky="w", pady=(4, 0))
        ttk.Button(control, text="Calibrate gyro bias",
                   command=self.start_calibration).grid(row=2, column=4, padx=5, pady=(4, 0))
        ttk.Label(control, text="Yaw uses shortest-angle error and differential wheels.").grid(
            row=2, column=2, columnspan=4, sticky="w", pady=(4, 0))

        pid = ttk.LabelFrame(root, text="Online PID tuning", padding=8)
        pid.grid(row=3, column=0, sticky="ew", pady=(8, 0))
        ttk.Label(pid, text="Loop").grid(row=0, column=0, sticky="w")
        self.pid_controller_box = ttk.Combobox(
            pid, textvariable=self.pid_controller, state="readonly", width=18,
            values=("Wheel speed loop", "Angle loop", "Speed loop", "Steering loop"))
        self.pid_controller_box.grid(row=0, column=1, padx=4)
        ttk.Label(pid, text="Gain").grid(row=0, column=2, sticky="w")
        self.pid_gain_box = ttk.Combobox(
            pid, textvariable=self.pid_gain, state="readonly", width=7,
            values=("Kp", "Ki", "Kd"))
        self.pid_gain_box.grid(row=0, column=3, padx=4)
        ttk.Label(pid, text="Value").grid(row=0, column=4, sticky="w")
        ttk.Entry(pid, textvariable=self.pid_value, width=10).grid(row=0, column=5, padx=4)
        ttk.Button(pid, text="Apply PID (protect)", command=self.send_pid).grid(
            row=0, column=6, padx=5)
        ttk.Label(pid, text="Wheel FF gain (PWM per m/s)").grid(
            row=1, column=0, sticky="w")
        ttk.Entry(pid, textvariable=self.feedforward, width=10).grid(row=1, column=1, padx=4)
        ttk.Button(pid, text="Apply wheel FF", command=self.send_feedforward).grid(
            row=1, column=2, columnspan=2, padx=4)
        ttk.Label(pid, text="Steering FF gain (PWM per deg)").grid(
            row=1, column=4, sticky="w")
        ttk.Entry(pid, textvariable=self.steering_feedforward, width=10).grid(row=1, column=5, padx=4)
        ttk.Button(pid, text="Apply steering FF",
                   command=self.send_steering_feedforward).grid(row=1, column=6, padx=4)
        self.pid_controller_box.bind("<<ComboboxSelected>>", self.update_pid_value)
        self.pid_gain_box.bind("<<ComboboxSelected>>", self.update_pid_value)

        notebook = ttk.Notebook(root)
        notebook.grid(row=4, column=0, sticky="nsew", pady=(8, 0))
        monitor_tab = ttk.Frame(notebook, padding=6)
        plot_tab = ttk.Frame(notebook, padding=6)
        trajectory_tab = ttk.Frame(notebook, padding=6)
        notebook.add(monitor_tab, text="Live values")
        notebook.add(plot_tab, text="Realtime charts")
        notebook.add(trajectory_tab, text="Trajectory")
        monitor_tab.columnconfigure(0, weight=1)
        monitor_tab.columnconfigure(1, weight=1)
        monitor_tab.rowconfigure(0, weight=1)
        self._readout(monitor_tab, 0, "Attitude and IMU", (
            ("roll", "Roll", "deg"), ("pitch", "Pitch", "deg"),
            ("yaw_actual", "Yaw", "deg"), ("pitch_rate", "Pitch rate", "deg/s"),
            ("gx", "Gyro X", "deg/s"), ("gy", "Gyro Y", "deg/s"),
            ("gz", "Gyro Z", "deg/s"), ("ax", "Accel X", "g"),
            ("ay", "Accel Y", "g"), ("az", "Accel Z", "g"),
            ("cal_status", "Gyro calibration", ""),
            ("bias_x", "Bias X", "deg/s"), ("bias_y", "Bias Y", "deg/s"),
            ("bias_z", "Bias Z", "deg/s"), ("cal_samples", "Bias samples", ""),
        ))
        self._readout(monitor_tab, 1, "Motion and diagnostic", (
            ("left_speed", "Left wheel speed", "m/s"),
            ("right_speed", "Right wheel speed", "m/s"),
            ("left_count", "Left encoder", "count/10ms"),
            ("right_count", "Right encoder", "count/10ms"),
            ("left_pwm", "Left PWM", "/10000"),
            ("right_pwm", "Right PWM", "/10000"),
            ("target_speed", "Target speed", "m/s"),
            ("target_yaw", "Target yaw", "deg"),
            ("target_pitch", "Target pitch", "deg"),
            ("trajectory", "Trajectory", "m"),
        ))

        if Figure is None or FigureCanvasTkAgg is None:
            ttk.Label(plot_tab, text="Install upper/requirements.txt to enable realtime charts.").pack(
                anchor="w")
            ttk.Label(trajectory_tab, text="Install upper/requirements.txt to enable trajectory view.").pack(
                anchor="w")
            self.chart_figure = None
            self.chart_canvas = None
            self.trajectory_figure = None
            self.trajectory_canvas = None
        else:
            plot_tab.columnconfigure(0, weight=1)
            plot_tab.rowconfigure(1, weight=1)
            chart_controls = ttk.Frame(plot_tab)
            chart_controls.grid(row=0, column=0, sticky="ew", pady=(0, 5))
            ttk.Label(chart_controls, text="Chart").pack(side="left")
            chart_box = ttk.Combobox(chart_controls, textvariable=self.plot_choice,
                                     state="readonly", values=CHARTS, width=26)
            chart_box.pack(side="left", padx=6)
            chart_box.bind("<<ComboboxSelected>>", lambda _event: self._draw_chart())
            ttk.Button(chart_controls, text="Clear history",
                       command=self.reset_history).pack(side="left", padx=5)
            self.chart_figure = Figure(figsize=(8, 4.4), dpi=100)
            self.chart_axis = self.chart_figure.add_subplot(111)
            self.chart_canvas = FigureCanvasTkAgg(self.chart_figure, master=plot_tab)
            self.chart_canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")
            chart_data = ttk.LabelFrame(plot_tab, text="Current data", padding=8)
            chart_data.grid(row=1, column=1, sticky="nse", padx=(8, 0))
            ttk.Label(chart_data, text="Time (s)").grid(row=0, column=0, sticky="w")
            ttk.Label(chart_data, textvariable=self.chart_readout_time,
                      font=("Segoe UI", 10, "bold")).grid(
                          row=0, column=1, sticky="e", padx=(12, 0))
            for row in range(3):
                name = tk.StringVar(value="--")
                value = tk.StringVar(value="--")
                self.chart_readout_names.append(name)
                self.chart_readout_values.append(value)
                ttk.Label(chart_data, textvariable=name).grid(
                    row=row + 1, column=0, sticky="w", pady=(8, 0))
                ttk.Label(chart_data, textvariable=value,
                          font=("Segoe UI", 10, "bold")).grid(
                              row=row + 1, column=1, sticky="e",
                              padx=(12, 0), pady=(8, 0))

            trajectory_tab.columnconfigure(0, weight=1)
            trajectory_tab.rowconfigure(1, weight=1)
            trajectory_controls = ttk.Frame(trajectory_tab)
            trajectory_controls.grid(row=0, column=0, sticky="ew", pady=(0, 5))
            ttk.Button(trajectory_controls, text="Reset trajectory",
                       command=self.reset_trajectory).pack(side="left")
            ttk.Label(trajectory_controls,
                      text="Body frame: +X forward, +Y left; origin and heading are set on reset.").pack(
                          side="left", padx=10)
            self.trajectory_figure = Figure(figsize=(8, 4.4), dpi=100)
            self.trajectory_axis = self.trajectory_figure.add_subplot(111)
            self.trajectory_canvas = FigureCanvasTkAgg(
                self.trajectory_figure, master=trajectory_tab)
            self.trajectory_canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")

        keyboard = ttk.LabelFrame(root, text="Keyboard control", padding=6)
        keyboard.grid(row=5, column=0, sticky="ew", pady=(8, 0))
        ttk.Checkbutton(keyboard, text="Enable W/S + A/D mode",
                        variable=self.keyboard_enabled,
                        command=self.toggle_keyboard).grid(row=0, column=0, sticky="w")
        ttk.Label(keyboard, text="W/S speed (m/s)").grid(row=0, column=1, padx=(18, 3))
        ttk.Entry(keyboard, textvariable=self.keyboard_speed, width=8).grid(row=0, column=2)
        ttk.Label(keyboard, text="A/D yaw rate (deg/s)").grid(row=0, column=3, padx=(18, 3))
        ttk.Entry(keyboard, textvariable=self.keyboard_yaw_rate, width=8).grid(row=0, column=4)
        ttk.Label(keyboard, text="Hold A/D for continuous yaw; steering loop must be enabled.").grid(
            row=0, column=5, padx=15, sticky="w")

        self.notice = tk.StringVar(value="Connect to receive telemetry.")
        ttk.Label(root, textvariable=self.notice).grid(row=6, column=0, sticky="w", pady=(6, 0))

    def _readout(self, parent: tk.Widget, column: int, title: str, rows) -> None:
        box = ttk.LabelFrame(parent, text=title, padding=8)
        box.grid(row=0, column=column, sticky="nsew", padx=(0, 5) if column == 0 else (5, 0))
        box.columnconfigure(1, weight=1)
        for index, (key, label, unit) in enumerate(rows):
            ttk.Label(box, text=label).grid(row=index, column=0, sticky="w", pady=2)
            variable = tk.StringVar(value="--")
            self.metrics[key] = variable
            ttk.Label(box, textvariable=variable, font=("Consolas", 10)).grid(
                row=index, column=1, sticky="e", padx=6)
            ttk.Label(box, text=unit).grid(row=index, column=2, sticky="w")

    def _bind_keyboard(self) -> None:
        for key in ("w", "a", "s", "d"):
            self.bind_all(f"<KeyPress-{key}>",
                          lambda event, pressed_key=key: self._key_event(pressed_key, True))
            self.bind_all(f"<KeyRelease-{key}>",
                          lambda event, pressed_key=key: self._key_event(pressed_key, False))
            self.bind_all(f"<KeyPress-{key.upper()}>",
                          lambda event, pressed_key=key: self._key_event(pressed_key, True))
            self.bind_all(f"<KeyRelease-{key.upper()}>",
                          lambda event, pressed_key=key: self._key_event(pressed_key, False))

    def _key_event(self, key: str, pressed: bool) -> None:
        if self.keyboard_enabled.get():
            if pressed:
                self.pressed_keys.add(key)
            else:
                self.pressed_keys.discard(key)

    def _keyboard_tick(self) -> None:
        now = time.monotonic()
        elapsed = min(0.25, max(0.0, now - self.keyboard_last_tick))
        self.keyboard_last_tick = now
        if self.keyboard_enabled.get():
            try:
                configured_speed = min(1.0, max(0.0, abs(float(self.keyboard_speed.get()))))
                yaw_rate = min(900.0, max(0.0, abs(float(self.keyboard_yaw_rate.get()))))
            except ValueError:
                configured_speed, yaw_rate = 0.2, 100.0
            forward = (1 if "w" in self.pressed_keys else 0) - (1 if "s" in self.pressed_keys else 0)
            turn = (1 if "d" in self.pressed_keys else 0) - (1 if "a" in self.pressed_keys else 0)
            if turn:
                self.keyboard_yaw = wrap_angle_deg(
                    self.keyboard_yaw + turn * yaw_rate * elapsed
                )
                self.yaw.set(f"{self.keyboard_yaw:.2f}")
            speed = forward * configured_speed
            self.speed.set(f"{speed:.3f}")
            if self.telemetry_online():
                self._send_motion_values(speed, self.keyboard_yaw)
        self.after(KEYBOARD_PERIOD_MS, self._keyboard_tick)

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
            messagebox.showerror("Serial connection failed", str(exc))
            return
        self.parser = TelemetryParser()
        self.last_sequence = None
        self.lost_frames = 0
        self.last_frame_at = 0.0
        self.last_heartbeat_at = 0.0
        self.last_protect_at = 0.0
        self.latest_frame = None
        self.reset_history()
        self.connect_button.configure(text="Disconnect")
        self.status.set("Waiting for telemetry")

    def disconnect(self) -> None:
        self.keyboard_enabled.set(False)
        self.pressed_keys.clear()
        if self.port is not None:
            try:
                self.port.write(command_frame(STATE, PROTECT))
            except serial.SerialException:
                pass
            self.port.close()
            self.port = None
        self.connect_button.configure(text="Connect")
        self.status.set("Disconnected")
        self.notice.set("Disconnected; vehicle was requested to protect.")

    def send(self, command: int, value: int = 0, value2: int = 0) -> None:
        if self.port is None:
            self.notice.set("Connect the serial port first.")
            return
        try:
            self.port.write(command_frame(command, value, value2))
        except (serial.SerialException, serial.SerialTimeoutException) as exc:
            self.notice.set(f"Send failed: {exc}")
            self.disconnect()

    def send_state(self, target: int) -> None:
        if target != PROTECT and not self.telemetry_online():
            self.notice.set("No live telemetry; state request was not sent.")
            return
        self.send(STATE, target)
        self.notice.set("State request sent; verify the returned vehicle state.")

    def emergency_stop(self) -> None:
        self.speed.set("0.000")
        self.pressed_keys.clear()
        self.send(STATE, PROTECT)
        self.notice.set("Protection requested.")

    def _send_motion_values(self, speed: float, yaw: float) -> None:
        if self.port is None:
            return
        try:
            self.port.write(command_frame(MOTION, round(speed * 1000), round(yaw * 100)))
        except (serial.SerialException, serial.SerialTimeoutException) as exc:
            self.notice.set(f"Motion send failed: {exc}")
            self.disconnect()

    def send_motion(self) -> None:
        if not self.telemetry_online():
            self.notice.set("No live telemetry; motion target was not sent.")
            return
        try:
            speed = float(self.speed.get())
            yaw = wrap_angle_deg(float(self.yaw.get()))
            if not -1.0 <= speed <= 1.0:
                raise ValueError("speed must be between -1 and 1 m/s")
            self.yaw.set(f"{yaw:.2f}")
            self.keyboard_yaw = yaw
            self._send_motion_values(speed, yaw)
            self.notice.set("Motion target sent.")
        except ValueError as exc:
            messagebox.showerror("Invalid motion target", str(exc))

    def stop_motion(self) -> None:
        self.speed.set("0.000")
        self.pressed_keys.clear()
        if self.telemetry_online():
            self._send_motion_values(0.0, self.keyboard_yaw)
        else:
            self.emergency_stop()

    def update_pid_value(self, _event=None) -> None:
        key = (self.pid_controller.get(), self.pid_gain.get())
        if key in self.pid_values:
            self.pid_value.set(f"{self.pid_values[key]:g}")

    def send_pid(self) -> None:
        if not self.telemetry_online():
            self.notice.set("No live telemetry; PID was not sent.")
            return
        controllers = ("Wheel speed loop", "Angle loop", "Speed loop", "Steering loop")
        gains = ("Kp", "Ki", "Kd")
        try:
            value = float(self.pid_value.get())
            if not 0.0 <= value <= 10000.0:
                raise ValueError("PID gain must be between 0 and 10000")
            controller = controllers.index(self.pid_controller.get())
            gain = gains.index(self.pid_gain.get())
            self.port.write(pid_frame(controller, gain, value))
            self.pid_values[(self.pid_controller.get(), self.pid_gain.get())] = value
            self.notice.set("PID sent; vehicle enters protection.")
        except (ValueError, serial.SerialException, serial.SerialTimeoutException) as exc:
            self.notice.set(f"PID send failed: {exc}")
            if isinstance(exc, (serial.SerialException, serial.SerialTimeoutException)):
                self.disconnect()

    def send_feedforward(self) -> None:
        if not self.telemetry_online():
            self.notice.set("No live telemetry; feedforward was not sent.")
            return
        try:
            value = float(self.feedforward.get())
            self.port.write(feedforward_frame(value))
            self.notice.set("Wheel feedforward gain sent; vehicle enters protection.")
        except (ValueError, serial.SerialException, serial.SerialTimeoutException) as exc:
            self.notice.set(f"Feedforward send failed: {exc}")
            if isinstance(exc, (serial.SerialException, serial.SerialTimeoutException)):
                self.disconnect()

    def toggle_steering(self) -> None:
        if not self.telemetry_online():
            self.steering_enabled.set(False)
            self.notice.set("No live telemetry; steering remains disabled.")
            return
        try:
            self.port.write(steering_frame(self.steering_enabled.get()))
            self.notice.set("Steering setting sent; vehicle enters protection.")
        except (serial.SerialException, serial.SerialTimeoutException) as exc:
            self.steering_enabled.set(False)
            self.notice.set(f"Steering send failed: {exc}")
            self.disconnect()

    def send_steering_feedforward(self) -> None:
        if not self.telemetry_online():
            self.notice.set("No live telemetry; steering feedforward was not sent.")
            return
        try:
            value = float(self.steering_feedforward.get())
            self.port.write(steering_feedforward_frame(value))
            self.notice.set("Steering feedforward gain sent; vehicle enters protection.")
        except (ValueError, serial.SerialException, serial.SerialTimeoutException) as exc:
            self.notice.set(f"Steering feedforward send failed: {exc}")
            if isinstance(exc, (serial.SerialException, serial.SerialTimeoutException)):
                self.disconnect()

    def start_calibration(self) -> None:
        if not self.telemetry_online():
            self.notice.set("No live telemetry; calibration was not started.")
            return
        try:
            self.port.write(calibration_frame())
            self.notice.set("Gyro bias calibration started; keep the vehicle still.")
        except (serial.SerialException, serial.SerialTimeoutException) as exc:
            self.notice.set(f"Calibration send failed: {exc}")
            self.disconnect()

    def telemetry_online(self) -> bool:
        return self.port is not None and self.last_frame_at > 0 and \
            time.monotonic() - self.last_frame_at <= STALE_S

    def display_calibration(self, frame: CalibrationTelemetry) -> None:
        status = "running" if frame.active else "complete" if frame.complete else "idle"
        for key, value in (("cal_status", status), ("bias_x", f"{frame.gyro_bias_x_dps:.3f}"),
                           ("bias_y", f"{frame.gyro_bias_y_dps:.3f}"),
                           ("bias_z", f"{frame.gyro_bias_z_dps:.3f}"),
                           ("cal_samples", str(frame.sample_count))):
            self.metrics[key].set(value)

    def display(self, frame: Telemetry) -> None:
        if self.last_sequence is not None:
            gap = (frame.sequence - self.last_sequence - 1) & 0xFF
            if gap < 128:
                self.lost_frames += gap
        self.last_sequence = frame.sequence
        self.latest_frame = frame
        self.metrics["quality"].set(f"lost {self.lost_frames} / bad {self.parser.bad_frames}")
        self.metrics["mode"].set(
            MODES[frame.control_mode] if frame.valid & VALID_STATE and
            frame.control_mode < len(MODES) else "--")
        self.metrics["state"].set(
            STATES[frame.vehicle_mode] if frame.valid & VALID_STATE and
            frame.vehicle_mode < len(STATES) else "--")
        if frame.valid & VALID_LIFT:
            lift_status = "AIRBORNE/ABNORMAL"
        elif frame.valid & VALID_MOTION:
            lift_status = "NORMAL"
        else:
            lift_status = "UNKNOWN"
        self.metrics["lift"].set(lift_status)
        actual_speed = (frame.left_speed_m_s + frame.right_speed_m_s) * 0.5
        self.metrics["vehicle_speed"].set(f"{actual_speed:.3f} m/s" if frame.valid & VALID_MOTION else "--")
        self.metrics["state_pitch"].set(f"{frame.pitch_deg:.2f} deg" if frame.valid & VALID_ATTITUDE else "--")
        self.metrics["state_yaw"].set(f"{frame.yaw_deg:.2f} deg" if frame.valid & VALID_ATTITUDE else "--")
        self.metrics["state_pwm"].set(
            f"{frame.left_pwm:+d} / {frame.right_pwm:+d}" if frame.valid & VALID_CONTROL else "--")

        values = {
            "roll": frame.roll_deg, "pitch": frame.pitch_deg, "yaw_actual": frame.yaw_deg,
            "pitch_rate": frame.pitch_rate_dps, "gx": frame.gyro_x_dps, "gy": frame.gyro_y_dps,
            "gz": frame.gyro_z_dps, "ax": frame.acc_x_g, "ay": frame.acc_y_g, "az": frame.acc_z_g,
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
                        else VALID_STATE if key in ("target_speed", "target_yaw") else VALID_CONTROL)
            self.metrics[key].set("--" if required and not frame.valid & required else
                                  f"{value:.3f}" if isinstance(value, float) else str(value))

        if self.history_start_s is None:
            self.history_start_s = frame.uptime_ms / 1000.0
        t = frame.uptime_ms / 1000.0 - self.history_start_s
        attitude_valid = bool(frame.valid & VALID_ATTITUDE)
        motion_valid = bool(frame.valid & VALID_MOTION)
        state_valid = bool(frame.valid & VALID_STATE)
        control_valid = bool(frame.valid & VALID_CONTROL)
        self.samples.append({
            "t": t,
            "vehicle_speed": actual_speed if motion_valid else None,
            "target_speed": frame.target_speed_m_s if state_valid else None,
            "pitch": frame.pitch_deg if attitude_valid else None,
            "target_pitch": frame.target_pitch_deg if control_valid else None,
            "left_speed": frame.left_speed_m_s if motion_valid else None,
            "right_speed": frame.right_speed_m_s if motion_valid else None,
            "roll": frame.roll_deg if attitude_valid else None,
            "yaw": frame.yaw_deg if attitude_valid else None,
        })
        self._update_trajectory(frame)
        self._draw_chart()
        self._draw_trajectory()

    def _update_trajectory(self, frame: Telemetry) -> None:
        if frame.valid & VALID_LIFT:
            # Keep the time origin current so the first grounded frame does not
            # integrate a fictitious distance over the airborne interval.
            self.last_trajectory_uptime_ms = frame.uptime_ms
            return
        if not (frame.valid & VALID_MOTION and frame.valid & VALID_ATTITUDE):
            return
        if self.last_trajectory_uptime_ms is None:
            self.last_trajectory_uptime_ms = frame.uptime_ms
            self.trajectory_heading_zero_deg = frame.yaw_deg
            return
        dt = (frame.uptime_ms - self.last_trajectory_uptime_ms) / 1000.0
        self.last_trajectory_uptime_ms = frame.uptime_ms
        if not 0.0 < dt <= 1.0:
            return
        speed = (frame.left_speed_m_s + frame.right_speed_m_s) * 0.5
        if self.trajectory_heading_zero_deg is None:
            self.trajectory_heading_zero_deg = frame.yaw_deg
        heading = math.radians(
            wrap_angle_deg(frame.yaw_deg - self.trajectory_heading_zero_deg))
        self.trajectory_x += speed * dt * math.cos(heading)
        self.trajectory_y += speed * dt * math.sin(heading)
        self.trajectory_points.append((self.trajectory_x, self.trajectory_y))
        self.metrics["trajectory"].set(
            f"{self.trajectory_x:.2f}, {self.trajectory_y:.2f}")

    def reset_history(self) -> None:
        self.samples.clear()
        self.history_start_s = None
        self.reset_trajectory()
        self._draw_chart()

    def reset_trajectory(self) -> None:
        self.trajectory_points.clear()
        self.trajectory_points.append((0.0, 0.0))
        self.trajectory_x = 0.0
        self.trajectory_y = 0.0
        self.last_trajectory_uptime_ms = None
        self.trajectory_heading_zero_deg = None
        if "trajectory" in self.metrics:
            self.metrics["trajectory"].set("0.00, 0.00")
        self._draw_trajectory()

    def _plot_series(self, key: str) -> tuple[list[float], list[float]]:
        points = [(item["t"], item[key]) for item in self.samples if item[key] is not None]
        return [point[0] for point in points], [point[1] for point in points]

    def _draw_chart(self) -> None:
        if self.chart_canvas is None:
            return
        axis = self.chart_axis
        axis.clear()
        chart = self.plot_choice.get()
        if chart == CHARTS[0]:
            series = (("vehicle_speed", "Vehicle speed", "tab:blue"),
                      ("target_speed", "Target speed", "tab:orange"))
            ylabel = "m/s"
        elif chart == CHARTS[1]:
            series = (("pitch", "Actual pitch", "tab:blue"),
                      ("target_pitch", "Target pitch", "tab:orange"))
            ylabel = "deg"
        elif chart == CHARTS[2]:
            series = (("left_speed", "Left wheel", "tab:blue"),
                      ("right_speed", "Right wheel", "tab:green"),
                      ("target_speed", "Target wheel speed (derived)", "tab:orange"))
            ylabel = "m/s"
        else:
            series = (("roll", "Roll", "tab:red"), ("pitch", "Pitch", "tab:blue"),
                      ("yaw", "Yaw", "tab:green"))
            ylabel = "deg"
        latest = self.samples[-1] if self.samples else None
        self.chart_readout_time.set(
            f"{latest['t']:.2f}" if latest is not None else "--")
        for index, (key, label, _color) in enumerate(series):
            self.chart_readout_names[index].set(label)
            value = latest.get(key) if latest is not None else None
            self.chart_readout_values[index].set(
                f"{value:.3f} {ylabel}" if value is not None else "--")
        for index in range(len(series), len(self.chart_readout_names)):
            self.chart_readout_names[index].set("--")
            self.chart_readout_values[index].set("--")
        for key, label, color in series:
            x, y = self._plot_series(key)
            axis.plot(x, y, label=label, color=color)
        axis.set_xlabel("time (s)")
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right")
        self.chart_figure.tight_layout()
        self.chart_canvas.draw_idle()

    def _draw_trajectory(self) -> None:
        if self.trajectory_canvas is None:
            return
        axis = self.trajectory_axis
        axis.clear()
        x_values = [point[0] for point in self.trajectory_points]
        y_values = [point[1] for point in self.trajectory_points]
        axis.plot(x_values, y_values, color="tab:blue", linewidth=1.8, label="fused path")
        axis.scatter([x_values[0]], [y_values[0]], color="tab:green", label="start")
        axis.scatter([x_values[-1]], [y_values[-1]], color="tab:red", label="current")
        span = max(max(x_values) - min(x_values), max(y_values) - min(y_values), 1.0)
        margin = max(0.25, span * 0.15)
        axis.set_xlim(min(x_values) - margin, max(x_values) + margin)
        axis.set_ylim(min(y_values) - margin, max(y_values) + margin)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("Body +X forward (m)")
        axis.set_ylabel("Body +Y left (m)")
        axis.grid(True, alpha=0.3)
        axis.set_title("Trajectory in body coordinate frame")
        arrow_length = min(max(span * 0.15, 0.1), margin * 0.8)
        axis.annotate("", xy=(arrow_length, 0.0), xytext=(0.0, 0.0),
                      arrowprops={"arrowstyle": "->", "color": "tab:red",
                                  "linewidth": 1.5})
        axis.annotate("", xy=(0.0, arrow_length), xytext=(0.0, 0.0),
                      arrowprops={"arrowstyle": "->", "color": "tab:green",
                                  "linewidth": 1.5})
        axis.text(arrow_length, 0.0, " +X", color="tab:red", va="bottom")
        axis.text(0.0, arrow_length, " +Y", color="tab:green", ha="left")
        scale = 0.5 if span < 5.0 else 1.0
        x0 = min(x_values) - margin * 0.75
        y0 = min(y_values) - margin * 0.75
        axis.plot([x0, x0 + scale], [y0, y0], color="black", linewidth=3)
        axis.text(x0 + scale * 0.5, y0 + margin * 0.12, f"{scale:g} m", ha="center")
        axis.legend(loc="upper right")
        self.trajectory_figure.tight_layout()
        self.trajectory_canvas.draw_idle()

    def poll(self) -> None:
        now = time.monotonic()
        if self.port is not None:
            try:
                if now - self.last_heartbeat_at >= HEARTBEAT_MS / 1000.0:
                    self.port.write(command_frame(HEARTBEAT))
                    self.last_heartbeat_at = now
                data = self.port.read(min(self.port.in_waiting, 4096))
                for frame in self.parser.feed(data):
                    if isinstance(frame, CalibrationTelemetry):
                        self.display_calibration(frame)
                    else:
                        self.display(frame)
                    self.last_frame_at = now
                if (self.last_frame_at > 0 and now - self.last_frame_at > STALE_S and
                        now - self.last_protect_at >= HEARTBEAT_MS / 1000.0):
                    self.port.write(command_frame(STATE, PROTECT))
                    self.last_protect_at = now
                    self.notice.set("Telemetry timeout; protection requested.")
                self.status.set("Telemetry online" if self.telemetry_online() else "Telemetry timeout")
            except (serial.SerialException, serial.SerialTimeoutException) as exc:
                self.notice.set(f"Communication error: {exc}")
                self.disconnect()
        self.after(POLL_MS, self.poll)

    def toggle_keyboard(self) -> None:
        if self.keyboard_enabled.get():
            self.keyboard_last_tick = time.monotonic()
            try:
                self.keyboard_yaw = wrap_angle_deg(float(self.yaw.get()))
            except ValueError:
                self.keyboard_yaw = self.latest_frame.target_yaw_deg if self.latest_frame else 0.0
            self.yaw.set(f"{self.keyboard_yaw:.2f}")
            self.focus_set()
            self.notice.set("Keyboard mode enabled: W/S speed, A/D shortest-angle yaw target.")
        else:
            self.keyboard_last_tick = time.monotonic()
            self.pressed_keys.clear()
            self.speed.set("0.000")
            self.notice.set("Keyboard mode disabled.")

    def close(self) -> None:
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    if Figure is None:
        raise SystemExit("Install dependencies with: python -m pip install -r upper/requirements.txt")
    Console().mainloop()

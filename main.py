from __future__ import annotations

import json
import queue
import sys
import threading
import time
from dataclasses import asdict, fields
from datetime import datetime
from pathlib import Path
from typing import Optional

from PySide6.QtCore import QThread, Qt, Signal
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QRadioButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from esp300 import (
    ControllerSnapshot,
    ESP300Controller,
    ESP300Error,
    ESP300Settings,
    SerialTransport,
    VisaTransport,
    find_esp300_gpib_resources,
)
from joystick import HIDJoystickManager, PID as JOYSTICK_PID, VID as JOYSTICK_VID

CONFIG_PATH = Path(__file__).with_name("config.json")
CONFIG_VERSION = 2
DEFAULT_JOYSTICK_POLL_INTERVAL_S = 0.1
JOYSTICK_MOTION_EPSILON = 0.01


class ESPWorkerThread(QThread):
    connected_changed = Signal(bool, str)
    snapshot_ready = Signal(object)
    max_jog_speed_changed = Signal(float)
    log_message = Signal(str)
    error_message = Signal(str)

    def __init__(self, settings: ESP300Settings) -> None:
        super().__init__()
        self._commands: queue.Queue = queue.Queue()
        self._running = True
        self._poll_interval_s = settings.poll_interval_s
        self._controller: Optional[ESP300Controller] = None
        self._active_velocity_mm_s = {1: 0.0, 2: 0.0}
        self._jog_lock = threading.Lock()
        self._pending_jog_normalized: Optional[tuple[float, float]] = None
        self._jog_update_queued = False

    def run(self) -> None:
        next_poll = time.monotonic()
        while self._running:
            try:
                command = self._commands.get(timeout=0.05)
                self._process_command(command)
                while True:
                    self._process_command(self._commands.get_nowait())
            except queue.Empty:
                pass

            if (
                self._controller
                and self._controller.is_connected
                and time.monotonic() >= next_poll
                and self._commands.empty()
                and not self._motion_active()
            ):
                self._safe_poll_snapshot()
                next_poll = time.monotonic() + self._poll_interval_s
            elif self._motion_active():
                next_poll = time.monotonic() + self._poll_interval_s

        self._close_controller()

    def stop(self) -> None:
        self._running = False
        self._commands.put(("wake",))

    def connect_controller(
        self,
        method: str,
        rs232_port: str,
        rs232_rtscts: bool,
        gpib_resource: Optional[str],
    ) -> None:
        self._commands.put(
            ("connect", method, rs232_port, rs232_rtscts, gpib_resource)
        )

    def disconnect_controller(self) -> None:
        self._commands.put(("disconnect",))

    def set_poll_interval(self, poll_interval_s: float) -> None:
        self._commands.put(("poll_interval", poll_interval_s))

    def request_poll(self) -> None:
        self._commands.put(("poll",))

    def refresh_max_velocity(self) -> None:
        self._commands.put(("refresh_max",))

    def jog_velocity(self, x_mm_s: float, y_mm_s: float) -> None:
        self._commands.put(("jog_velocity", x_mm_s, y_mm_s))

    def jog_normalized(self, x_norm: float, y_norm: float) -> None:
        with self._jog_lock:
            self._pending_jog_normalized = (float(x_norm), float(y_norm))
            if self._jog_update_queued:
                return
            self._jog_update_queued = True
        self._commands.put(("jog_update",))

    def stop_motion(self) -> None:
        self._commands.put(("stop",))

    def abort_motion(self) -> None:
        self._commands.put(("abort",))

    def enable_all_motors(self) -> None:
        self._commands.put(("enable_all",))

    def disable_all_motors(self) -> None:
        self._commands.put(("disable_all",))

    def zero_xy(self) -> None:
        self._commands.put(("zero",))

    def goto_xy(self, x_mm: float, y_mm: float) -> None:
        self._commands.put(("goto", x_mm, y_mm))

    def _process_command(self, command: tuple) -> None:
        name = command[0]
        try:
            if name == "wake":
                return
            if name == "connect":
                self._connect(*command[1:])
            elif name == "disconnect":
                self._close_controller()
                self.connected_changed.emit(False, "Disconnected")
            elif name == "poll_interval":
                self._poll_interval_s = max(0.05, float(command[1]))
            elif name == "poll":
                self._poll_snapshot()
            elif name == "refresh_max":
                self._refresh_max_velocity()
            elif name == "jog_velocity":
                if self._controller and self._controller.is_connected:
                    self._apply_velocity(command[1], command[2])
            elif name == "jog_update":
                motion = self._take_pending_jog_normalized()
                if motion and self._controller and self._controller.is_connected:
                    self._apply_normalized_velocity(*motion)
            elif name == "jog_normalized":
                if self._controller and self._controller.is_connected:
                    self._apply_normalized_velocity(command[1], command[2])
            elif name == "stop":
                self._stop_motion()
            elif name == "abort":
                self._abort_motion()
            elif name == "enable_all":
                self._require_controller().enable_all_motors()
                self._poll_snapshot()
            elif name == "disable_all":
                self._require_controller().disable_all_motors()
                self._active_velocity_mm_s = {1: 0.0, 2: 0.0}
                self._poll_snapshot()
            elif name == "zero":
                self._require_controller().zero_xy()
                self._poll_snapshot()
            elif name == "goto":
                self._stop_motion()
                self._require_controller().goto_xy_mm(command[1], command[2])
        except Exception as exc:
            self.error_message.emit(str(exc))

    def _connect(
        self,
        method: str,
        rs232_port: str,
        rs232_rtscts: bool,
        gpib_resource: Optional[str],
    ) -> None:
        self._close_controller()
        if method == "RS232":
            transport = SerialTransport(
                rs232_port,
                rtscts=rs232_rtscts,
                log_callback=self.log_message.emit,
            )
        else:
            if not gpib_resource:
                raise ESP300Error("No ESP300/ESP301 GPIB resource is selected")
            transport = VisaTransport(gpib_resource, log_callback=self.log_message.emit)

        controller = ESP300Controller(transport)
        controller.connect()
        self._controller = controller
        self._active_velocity_mm_s = {1: 0.0, 2: 0.0}
        self.connected_changed.emit(True, f"Connected via {method}")
        self.max_jog_speed_changed.emit(controller.max_jog_speed_mm_s)
        self._poll_snapshot()

    def _close_controller(self) -> None:
        if not self._controller:
            return
        try:
            if self._controller.is_connected:
                self._controller.stop_all()
        except Exception:
            pass
        self._controller.close()
        self._controller = None
        self._active_velocity_mm_s = {1: 0.0, 2: 0.0}

    def _require_controller(self) -> ESP300Controller:
        if not self._controller or not self._controller.is_connected:
            raise ESP300Error("Connect to the ESP300 first")
        return self._controller

    def _poll_snapshot(self) -> None:
        if not self._controller or not self._controller.is_connected:
            return
        snapshot = self._controller.read_snapshot()
        self.snapshot_ready.emit(snapshot)

    def _safe_poll_snapshot(self) -> None:
        try:
            self._poll_snapshot()
        except Exception as exc:
            self.error_message.emit(f"Position poll failed: {exc}")

    def _motion_active(self) -> bool:
        return any(abs(value) > 1e-9 for value in self._active_velocity_mm_s.values())

    def _take_pending_jog_normalized(self) -> Optional[tuple[float, float]]:
        with self._jog_lock:
            motion = self._pending_jog_normalized
            self._pending_jog_normalized = None
            self._jog_update_queued = False
            return motion

    def _refresh_max_velocity(self) -> None:
        controller = self._require_controller()
        controller.refresh_max_velocities()
        self.max_jog_speed_changed.emit(controller.max_jog_speed_mm_s)

    def _apply_normalized_velocity(self, x_norm: float, y_norm: float) -> None:
        controller = self._require_controller()
        x = x_norm * controller.max_velocity_mm_s.get(1, 0.0)
        y = y_norm * controller.max_velocity_mm_s.get(2, 0.0)
        self._apply_velocity(x, y)

    def _apply_velocity(self, x_mm_s: float, y_mm_s: float) -> None:
        self._set_axis_velocity(1, x_mm_s)
        self._set_axis_velocity(2, y_mm_s)

    def _set_axis_velocity(self, axis: int, velocity_mm_s: float) -> None:
        controller = self._require_controller()
        velocity_mm_s = float(velocity_mm_s)
        previous = self._active_velocity_mm_s[axis]
        if abs(velocity_mm_s) < 1e-9:
            if abs(previous) > 1e-9:
                controller.stop_axis(axis)
                self._active_velocity_mm_s[axis] = 0.0
            return

        direction = 1 if velocity_mm_s > 0 else -1
        previous_direction = 1 if previous > 0 else -1 if previous < 0 else 0
        if previous_direction and previous_direction != direction:
            controller.stop_axis(axis)

        speed = abs(velocity_mm_s)
        if (
            previous_direction != direction
            or abs(abs(previous) - speed) > 1e-4
        ):
            controller.jog_axis(axis, direction, speed)
            self._active_velocity_mm_s[axis] = velocity_mm_s

    def _stop_motion(self) -> None:
        if not self._controller or not self._controller.is_connected:
            return
        self._controller.stop_all()
        self._active_velocity_mm_s = {1: 0.0, 2: 0.0}

    def _abort_motion(self) -> None:
        if not self._controller or not self._controller.is_connected:
            return
        self._controller.abort()
        self._active_velocity_mm_s = {1: 0.0, 2: 0.0}


class JoystickPollingThread(QThread):
    connection_changed = Signal(bool, str)
    motion_changed = Signal(float, float)
    report_message = Signal(str)

    def __init__(
        self,
        manager: HIDJoystickManager,
        settings: ESP300Settings,
    ) -> None:
        super().__init__()
        self._manager = manager
        self._running = True
        self._settings_lock = threading.Lock()
        self._settings = asdict(settings)
        self._last_motion = (0.0, 0.0)
        self._last_connected: Optional[bool] = None
        self._last_status = ""
        self._report_logs_remaining = 3

    def update_settings(self, settings: ESP300Settings) -> None:
        with self._settings_lock:
            self._settings = asdict(settings)

    def run(self) -> None:
        next_detection = 0.0
        while self._running:
            now = time.monotonic()
            if now >= next_detection:
                connected, error = self._manager.refresh_connection()
                self._emit_connection_if_changed(connected, error)
                next_detection = now + 1.0

            interval_s = self._setting("joystick_poll_interval_s", 0.1)
            if not self._manager.connected:
                self._emit_motion_if_changed((0.0, 0.0))
                self.msleep(max(10, int(interval_s * 1000)))
                continue

            report = self._manager.read_latest()
            if report is not None:
                self._emit_report_debug(report)
                self._emit_motion_if_changed(self._map_report(report))
            self.msleep(max(10, int(interval_s * 1000)))

    def stop(self) -> None:
        self._running = False

    def _map_report(self, report) -> tuple[float, float]:
        x = self._map_axis(report.x_raw)
        y = self._map_axis(report.y_raw)
        if self._setting("swap_xy", False):
            x, y = y, x
        if self._setting("flip_x", False):
            x = -x
        if self._setting("flip_y", False):
            y = -y
        return round(x, 4), round(y, 4)

    def _map_axis(self, raw: int) -> float:
        centered = max(-1.0, min(1.0, (float(raw) - 512.0) / 512.0))
        sign = 1.0 if centered >= 0 else -1.0
        magnitude = abs(centered)
        low = max(0.0, min(0.95, self._setting("low_deadband_percent", 5.0) / 100.0))
        high = max(
            0.0, min(0.95, self._setting("high_deadband_percent", 10.0) / 100.0)
        )
        active_span = max(0.001, 1.0 - low - high)
        if magnitude <= low:
            return 0.0
        if magnitude >= 1.0 - high:
            return sign
        scaled = (magnitude - low) / active_span
        return sign * (scaled ** max(1.0, self._setting("joystick_exponent", 5.0)))

    def _emit_motion_if_changed(self, motion: tuple[float, float]) -> None:
        epsilon = JOYSTICK_MOTION_EPSILON
        if (
            abs(motion[0] - self._last_motion[0]) >= epsilon
            or abs(motion[1] - self._last_motion[1]) >= epsilon
            or (motion == (0.0, 0.0) and self._last_motion != (0.0, 0.0))
        ):
            self._last_motion = motion
            self.motion_changed.emit(*motion)

    def _emit_connection_if_changed(self, connected: bool, error: str) -> None:
        if connected:
            backend = self._manager.backend
            if backend:
                status = (
                    f"Connected ({JOYSTICK_VID:04X}:{JOYSTICK_PID:04X} via "
                    f"{backend})"
                )
            else:
                status = f"Connected ({JOYSTICK_VID:04X}:{JOYSTICK_PID:04X})"
        else:
            status = error or "Disconnected"
        if connected == self._last_connected and status == self._last_status:
            return
        self.connection_changed.emit(connected, status)
        self._last_connected = connected
        self._last_status = status

    def _emit_report_debug(self, report) -> None:
        if self._report_logs_remaining <= 0:
            return
        raw = " ".join(f"{byte:02X}" for byte in report.raw)
        self.report_message.emit(
            "JOYSTICK REPORT: "
            f"len={len(report.raw)} offset={report.data_offset} raw={raw} "
            f"axes=({report.x_raw},{report.y_raw},{report.z_raw}) "
            f"buttons=0x{report.button_byte:02X}"
        )
        self._report_logs_remaining -= 1

    def _setting(self, name: str, default):
        with self._settings_lock:
            return self._settings.get(name, default)


class OptionsDialog(QDialog):
    def __init__(
        self,
        settings: ESP300Settings,
        max_jog_speed_mm_s: Optional[float],
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Options")

        self.jog_speed = QDoubleSpinBox()
        max_jog_speed = max_jog_speed_mm_s if max_jog_speed_mm_s else 1000.0
        self.jog_speed.setRange(0.001, max_jog_speed)
        self.jog_speed.setDecimals(4)
        self.jog_speed.setSuffix(" mm/s")
        self.jog_speed.setValue(min(settings.jog_speed_mm_s, max_jog_speed))

        self.poll_interval = QDoubleSpinBox()
        self.poll_interval.setRange(0.05, 60.0)
        self.poll_interval.setDecimals(3)
        self.poll_interval.setSuffix(" s")
        self.poll_interval.setValue(settings.poll_interval_s)

        self.joystick_poll_interval = QDoubleSpinBox()
        self.joystick_poll_interval.setRange(0.02, 2.0)
        self.joystick_poll_interval.setDecimals(3)
        self.joystick_poll_interval.setSuffix(" s")
        self.joystick_poll_interval.setValue(settings.joystick_poll_interval_s)

        self.low_deadband = QDoubleSpinBox()
        self.low_deadband.setRange(0.0, 49.0)
        self.low_deadband.setDecimals(1)
        self.low_deadband.setSuffix(" %")
        self.low_deadband.setValue(settings.low_deadband_percent)

        self.high_deadband = QDoubleSpinBox()
        self.high_deadband.setRange(0.0, 49.0)
        self.high_deadband.setDecimals(1)
        self.high_deadband.setSuffix(" %")
        self.high_deadband.setValue(settings.high_deadband_percent)

        self.joystick_exponent = QDoubleSpinBox()
        self.joystick_exponent.setRange(1.0, 15.0)
        self.joystick_exponent.setDecimals(1)
        self.joystick_exponent.setValue(settings.joystick_exponent)

        self.rs232_rtscts = QCheckBox()
        self.rs232_rtscts.setChecked(settings.rs232_rtscts)

        self.flip_x = QCheckBox()
        self.flip_x.setChecked(settings.flip_x)

        self.flip_y = QCheckBox()
        self.flip_y.setChecked(settings.flip_y)

        self.swap_xy = QCheckBox()
        self.swap_xy.setChecked(settings.swap_xy)

        form = QFormLayout()
        form.addRow("Jog speed", self.jog_speed)
        if max_jog_speed_mm_s:
            form.addRow("Controller max", QLabel(f"{max_jog_speed_mm_s:.6g} mm/s"))
        form.addRow("Poll interval", self.poll_interval)
        form.addRow("Joystick poll", self.joystick_poll_interval)
        form.addRow("Low deadband", self.low_deadband)
        form.addRow("High deadband", self.high_deadband)
        form.addRow("Joystick exponent", self.joystick_exponent)
        form.addRow("RS232 RTS/CTS", self.rs232_rtscts)
        form.addRow("Flip X", self.flip_x)
        form.addRow("Flip Y", self.flip_y)
        form.addRow("Swap X/Y", self.swap_xy)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(buttons)

    def update_settings(self, settings: ESP300Settings) -> None:
        settings.jog_speed_mm_s = self.jog_speed.value()
        settings.poll_interval_s = self.poll_interval.value()
        settings.joystick_poll_interval_s = self.joystick_poll_interval.value()
        settings.low_deadband_percent = self.low_deadband.value()
        settings.high_deadband_percent = self.high_deadband.value()
        settings.joystick_exponent = self.joystick_exponent.value()
        settings.rs232_rtscts = self.rs232_rtscts.isChecked()
        settings.flip_x = self.flip_x.isChecked()
        settings.flip_y = self.flip_y.isChecked()
        settings.swap_xy = self.swap_xy.isChecked()


class GotoDialog(QDialog):
    def __init__(
        self,
        initial_x: float,
        initial_y: float,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Goto")

        self.x_value = QDoubleSpinBox()
        self.x_value.setRange(-1_000_000.0, 1_000_000.0)
        self.x_value.setDecimals(6)
        self.x_value.setSuffix(" mm")
        self.x_value.setValue(initial_x)

        self.y_value = QDoubleSpinBox()
        self.y_value.setRange(-1_000_000.0, 1_000_000.0)
        self.y_value.setDecimals(6)
        self.y_value.setSuffix(" mm")
        self.y_value.setValue(initial_y)

        form = QFormLayout()
        form.addRow("X", self.x_value)
        form.addRow("Y", self.y_value)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(buttons)

    @property
    def target(self) -> tuple[float, float]:
        return self.x_value.value(), self.y_value.value()


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ESP300 Joystick Controller")

        self.config = self.load_config()
        self.settings = self.load_settings()
        self.esp_connected = False
        self.max_jog_speed_mm_s = 0.0
        self.current_position = (0.0, 0.0)
        self.pressed_directions: set[tuple[str, int]] = set()
        self.all_motors_enabled: Optional[bool] = None

        self._build_menu()
        self._build_ui()
        self.restore_connection_config()
        self.start_workers()
        self._refresh_connection_ui()

    def start_workers(self) -> None:
        self.esp_worker = ESPWorkerThread(self.settings)
        self.esp_worker.connected_changed.connect(self.on_esp_connection_changed)
        self.esp_worker.snapshot_ready.connect(self.on_snapshot_ready)
        self.esp_worker.max_jog_speed_changed.connect(self.on_max_jog_speed_changed)
        self.esp_worker.log_message.connect(self.append_log)
        self.esp_worker.error_message.connect(self.on_worker_error)
        self.esp_worker.start()
        self.esp_worker.set_poll_interval(self.settings.poll_interval_s)

        self.joystick_manager = HIDJoystickManager()
        self.joystick_worker = JoystickPollingThread(
            self.joystick_manager,
            self.settings,
        )
        self.joystick_worker.connection_changed.connect(
            self.on_joystick_connection_changed
        )
        self.joystick_worker.motion_changed.connect(self.esp_worker.jog_normalized)
        self.joystick_worker.report_message.connect(self.append_log)
        self.append_log(
            f"Joystick poll interval: {self.settings.joystick_poll_interval_s:.3f} s"
        )
        self.joystick_worker.start()

    def load_config(self) -> dict:
        try:
            data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}
        return data if isinstance(data, dict) else {}

    def load_settings(self) -> ESP300Settings:
        settings = ESP300Settings()
        raw_settings = self.config.get("settings", {})
        if not isinstance(raw_settings, dict):
            return settings

        field_names = {field.name for field in fields(ESP300Settings)}
        float_fields = {
            "jog_speed_mm_s",
            "poll_interval_s",
            "joystick_poll_interval_s",
            "low_deadband_percent",
            "high_deadband_percent",
            "joystick_exponent",
        }
        bool_fields = {"rs232_rtscts", "flip_x", "flip_y", "swap_xy"}

        for name, value in raw_settings.items():
            if name not in field_names:
                continue
            try:
                if name in float_fields:
                    setattr(settings, name, float(value))
                elif name in bool_fields:
                    setattr(settings, name, bool(value))
            except (TypeError, ValueError):
                continue
        if self.config.get("version", 0) < 2:
            settings.joystick_poll_interval_s = min(
                settings.joystick_poll_interval_s,
                DEFAULT_JOYSTICK_POLL_INTERVAL_S,
            )
        return settings

    def save_config(self) -> None:
        config = {
            "version": CONFIG_VERSION,
            "settings": asdict(self.settings),
            "connection": self.connection_config(),
        }
        try:
            CONFIG_PATH.write_text(
                json.dumps(config, indent=2, sort_keys=True),
                encoding="utf-8",
            )
            self.config = config
        except OSError as exc:
            if hasattr(self, "statusBar"):
                self.statusBar().showMessage(f"Could not save config: {exc}")

    def connection_config(self) -> dict:
        if not hasattr(self, "rs232_radio"):
            connection = self.config.get("connection", {})
            return connection if isinstance(connection, dict) else {}
        return {
            "interface": self.selected_interface(),
            "rs232_port": self.port_combo.currentText().strip(),
            "gpib_resource": self.selected_gpib_resource_name(),
        }

    def restore_connection_config(self) -> None:
        connection = self.config.get("connection", {})
        if not isinstance(connection, dict):
            return

        rs232_port = connection.get("rs232_port")
        if isinstance(rs232_port, str) and rs232_port:
            self.set_combo_text(self.port_combo, rs232_port)

        gpib_resource = connection.get("gpib_resource")
        if isinstance(gpib_resource, str) and gpib_resource:
            for index in range(self.gpib_resource_combo.count()):
                if self.gpib_resource_combo.itemData(index) == gpib_resource:
                    self.gpib_resource_combo.setCurrentIndex(index)
                    break

        if connection.get("interface") == "GPIB":
            self.gpib_radio.setChecked(True)
        else:
            self.rs232_radio.setChecked(True)
        self._on_method_changed()

    def set_combo_text(self, combo: QComboBox, text: str) -> None:
        index = combo.findText(text)
        if index < 0:
            combo.insertItem(0, text)
            index = 0
        combo.setCurrentIndex(index)

    def _build_menu(self) -> None:
        options_action = QAction("Options...", self)
        options_action.triggered.connect(self.show_options)
        self.menuBar().addAction(options_action)

        goto_action = QAction("Goto...", self)
        goto_action.triggered.connect(self.show_goto)
        self.menuBar().addAction(goto_action)

    def _build_ui(self) -> None:
        root = QWidget()
        root_layout = QGridLayout(root)
        root_layout.setColumnStretch(0, 1)
        root_layout.setColumnStretch(1, 1)
        root_layout.setRowStretch(1, 1)
        root_layout.setRowStretch(2, 1)

        root_layout.addWidget(self._build_connection_panel(), 0, 0)
        root_layout.addWidget(self._build_readout_panel(), 0, 1)
        root_layout.addWidget(self._build_joystick_panel(), 1, 0, 1, 2)
        root_layout.addWidget(self._build_log_panel(), 2, 0, 1, 2)

        self.setCentralWidget(root)
        self.statusBar().showMessage("Disconnected")

    def _build_connection_panel(self) -> QGroupBox:
        group = QGroupBox("Connection")
        layout = QVBoxLayout(group)

        esp_group = QGroupBox("ESP300")
        esp_layout = QFormLayout(esp_group)

        self.rs232_radio = QRadioButton("RS232")
        self.gpib_radio = QRadioButton("GPIB")
        self.rs232_radio.setChecked(True)
        self.method_group = QButtonGroup(self)
        self.method_group.addButton(self.rs232_radio)
        self.method_group.addButton(self.gpib_radio)
        self.rs232_radio.toggled.connect(self._on_method_changed)
        self.gpib_radio.toggled.connect(self._on_method_changed)

        method_row = QHBoxLayout()
        method_row.addWidget(self.rs232_radio)
        method_row.addWidget(self.gpib_radio)
        method_row.addStretch()

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.currentTextChanged.connect(self._refresh_connection_ui)
        self.refresh_ports_button = QPushButton("Refresh")
        self.refresh_ports_button.clicked.connect(self.refresh_serial_ports)

        port_row = QHBoxLayout()
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(self.refresh_ports_button)

        rs232_page = QWidget()
        rs232_layout = QFormLayout(rs232_page)
        rs232_layout.addRow("Port", port_row)
        self.refresh_serial_ports()

        self.gpib_resource_combo = QComboBox()
        self.gpib_resource_combo.setEditable(False)
        self.gpib_resource_combo.currentIndexChanged.connect(
            self._refresh_connection_ui
        )
        self.refresh_gpib_button = QPushButton("Refresh")
        self.refresh_gpib_button.clicked.connect(self.refresh_gpib_resources)

        gpib_row = QHBoxLayout()
        gpib_row.addWidget(self.gpib_resource_combo, 1)
        gpib_row.addWidget(self.refresh_gpib_button)

        gpib_page = QWidget()
        gpib_layout = QFormLayout(gpib_page)
        gpib_layout.addRow("VISA resource", gpib_row)
        self.refresh_gpib_resources()

        self.connection_stack = QStackedWidget()
        self.connection_stack.addWidget(rs232_page)
        self.connection_stack.addWidget(gpib_page)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.toggle_connection)
        self.connection_status = QLabel("Not connected")

        esp_layout.addRow("Interface", method_row)
        esp_layout.addRow(self.connection_stack)
        esp_layout.addRow(self.connect_button)
        esp_layout.addRow("Status", self.connection_status)

        joystick_group = QGroupBox("Joystick")
        joystick_layout = QFormLayout(joystick_group)
        self.joystick_status = QLabel("Scanning...")
        joystick_layout.addRow("USB HID", self.joystick_status)
        joystick_layout.addRow(
            "VID:PID",
            QLabel(f"{JOYSTICK_VID:04X}:{JOYSTICK_PID:04X}"),
        )

        layout.addWidget(esp_group)
        layout.addWidget(joystick_group)
        return group

    def _build_readout_panel(self) -> QGroupBox:
        group = QGroupBox("Digital Readout")
        layout = QGridLayout(group)

        self.x_readout = QLabel("0.000000")
        self.y_readout = QLabel("0.000000")
        for label in (self.x_readout, self.y_readout):
            label.setAlignment(
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
            )
            label.setMinimumWidth(140)
            label.setStyleSheet("font-size: 24px; font-weight: 600;")

        self.zero_button = QPushButton("Zero")
        self.zero_button.clicked.connect(self.zero_position)
        self.motor_status = QLabel("--")
        self.limit_status = QLabel("--")
        self.motor_power_button = QPushButton("Enable all motors")
        self.motor_power_button.clicked.connect(self.toggle_motor_power)
        self.motor_power_button.setEnabled(False)

        layout.addWidget(QLabel("X mm"), 0, 0)
        layout.addWidget(self.x_readout, 0, 1)
        layout.addWidget(QLabel("Y mm"), 1, 0)
        layout.addWidget(self.y_readout, 1, 1)
        layout.addWidget(self.zero_button, 0, 2, 2, 1)
        layout.addWidget(QLabel("Motors"), 2, 0)
        layout.addWidget(self.motor_status, 2, 1)
        layout.addWidget(self.motor_power_button, 2, 2)
        layout.addWidget(QLabel("End switches"), 3, 0)
        layout.addWidget(self.limit_status, 3, 1, 1, 2)
        layout.setColumnStretch(1, 1)
        return group

    def _build_joystick_panel(self) -> QGroupBox:
        group = QGroupBox("Emulated Joystick")
        outer = QVBoxLayout(group)
        grid = QGridLayout()

        self.up_button = self._make_jog_button("Y+")
        self.down_button = self._make_jog_button("Y-")
        self.left_button = self._make_jog_button("X-")
        self.right_button = self._make_jog_button("X+")

        self._wire_jog_button(self.up_button, "y", 1)
        self._wire_jog_button(self.down_button, "y", -1)
        self._wire_jog_button(self.left_button, "x", -1)
        self._wire_jog_button(self.right_button, "x", 1)

        self.abort_button = QPushButton("Abort")
        self.abort_button.setMinimumSize(92, 56)
        self.abort_button.clicked.connect(self.abort_motion)

        grid.addWidget(self.up_button, 0, 1)
        grid.addWidget(self.left_button, 1, 0)
        grid.addWidget(self.abort_button, 1, 1)
        grid.addWidget(self.right_button, 1, 2)
        grid.addWidget(self.down_button, 2, 1)

        outer.addLayout(grid)
        return group

    def _build_log_panel(self) -> QGroupBox:
        group = QGroupBox("Message Log")
        layout = QVBoxLayout(group)

        self.command_log = QPlainTextEdit()
        self.command_log.setReadOnly(True)
        self.command_log.setMaximumBlockCount(2000)

        button_row = QHBoxLayout()
        button_row.addStretch()
        clear_button = QPushButton("Clear")
        clear_button.clicked.connect(self.command_log.clear)
        button_row.addWidget(clear_button)

        layout.addWidget(self.command_log)
        layout.addLayout(button_row)
        return group

    def _make_jog_button(self, text: str) -> QPushButton:
        button = QPushButton(text)
        button.setMinimumSize(92, 56)
        button.setAutoRepeat(False)
        return button

    def _wire_jog_button(self, button: QPushButton, axis: str, direction: int) -> None:
        button.pressed.connect(lambda: self._set_direction_pressed(axis, direction, True))
        button.released.connect(lambda: self._set_direction_pressed(axis, direction, False))

    def _on_method_changed(self, *_args) -> None:
        self.connection_stack.setCurrentIndex(1 if self.gpib_radio.isChecked() else 0)
        self._refresh_connection_ui()

    def selected_interface(self) -> str:
        return "GPIB" if self.gpib_radio.isChecked() else "RS232"

    def toggle_connection(self) -> None:
        if self.esp_connected:
            self.disconnect_controller()
        else:
            self.connect_controller()

    def connect_controller(self) -> None:
        method = self.selected_interface()
        self.connection_status.setText("Connecting...")
        self.connect_button.setEnabled(False)
        self.esp_worker.connect_controller(
            method,
            self.port_combo.currentText().strip(),
            self.settings.rs232_rtscts,
            self.selected_gpib_resource_name(),
        )

    def disconnect_controller(self) -> None:
        self.esp_worker.disconnect_controller()

    def on_esp_connection_changed(self, connected: bool, message: str) -> None:
        self.esp_connected = connected
        if not connected:
            self.pressed_directions.clear()
            self.reset_axis_statuses()
        self.connection_status.setText("Connected" if connected else "Not connected")
        self.statusBar().showMessage(message)
        self._refresh_connection_ui()
        if connected:
            self.save_config()

    def on_snapshot_ready(self, snapshot: ControllerSnapshot) -> None:
        self.current_position = (snapshot.x_mm, snapshot.y_mm)
        self.x_readout.setText(f"{snapshot.x_mm:.6f}")
        self.y_readout.setText(f"{snapshot.y_mm:.6f}")
        self.update_axis_statuses(snapshot.axis_states)

    def on_max_jog_speed_changed(self, max_speed_mm_s: float) -> None:
        self.max_jog_speed_mm_s = max_speed_mm_s
        self.apply_controller_limits()

    def on_worker_error(self, message: str) -> None:
        self.statusBar().showMessage(message)
        if not self.esp_connected:
            self.connection_status.setText("Not connected")
            self.reset_axis_statuses()
        self._refresh_connection_ui()

    def on_joystick_connection_changed(self, connected: bool, status: str) -> None:
        self.joystick_status.setText(status)
        self.append_log(f"JOYSTICK: {status}")

    def _refresh_connection_ui(self, *_args) -> None:
        if not hasattr(self, "connect_button"):
            return
        connected = self.esp_connected
        can_connect = connected or self.connection_selection_available()
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.connect_button.setEnabled(can_connect)
        self.connection_status.setText("Connected" if connected else "Not connected")
        self.rs232_radio.setEnabled(not connected)
        self.gpib_radio.setEnabled(not connected)
        self.port_combo.setEnabled(not connected)
        self.refresh_ports_button.setEnabled(not connected)
        self.gpib_resource_combo.setEnabled(not connected)
        self.refresh_gpib_button.setEnabled(not connected)
        self.update_motor_power_button()

    def refresh_serial_ports(self) -> None:
        current = self.port_combo.currentText().strip() or "COM1"
        ports = []
        try:
            from serial.tools import list_ports

            ports = [port.device for port in list_ports.comports()]
        except ImportError:
            ports = []

        if current and current not in ports:
            ports.insert(0, current)
        if not ports:
            ports = ["COM1"]

        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        index = self.port_combo.findText(current)
        self.port_combo.setCurrentIndex(index if index >= 0 else 0)
        self.port_combo.blockSignals(False)
        self._refresh_connection_ui()

    def refresh_gpib_resources(self) -> None:
        current = self.selected_gpib_resource_name()
        try:
            resources = find_esp300_gpib_resources()
        except Exception as exc:
            resources = []
            self.statusBar().showMessage(f"GPIB refresh failed: {exc}")

        self.gpib_resource_combo.blockSignals(True)
        self.gpib_resource_combo.clear()
        if resources:
            for resource in resources:
                self.gpib_resource_combo.addItem(
                    f"{resource.resource_name} - {resource.identity}",
                    resource.resource_name,
                )
            if current:
                for index in range(self.gpib_resource_combo.count()):
                    if self.gpib_resource_combo.itemData(index) == current:
                        self.gpib_resource_combo.setCurrentIndex(index)
                        break
        else:
            self.gpib_resource_combo.addItem("None", None)
        self.gpib_resource_combo.blockSignals(False)
        if resources:
            self.statusBar().showMessage(
                f"Found {len(resources)} ESP300/ESP301 GPIB resource(s)"
            )
        else:
            self.statusBar().showMessage("No ESP300/ESP301 GPIB resources found")
        self._refresh_connection_ui()

    def selected_gpib_resource_name(self) -> Optional[str]:
        resource_name = self.gpib_resource_combo.currentData()
        if isinstance(resource_name, str) and resource_name:
            return resource_name
        return None

    def connection_selection_available(self) -> bool:
        if self.selected_interface() == "RS232":
            return bool(self.port_combo.currentText().strip())
        return self.selected_gpib_resource_name() is not None

    def current_max_jog_speed_mm_s(self) -> Optional[float]:
        return self.max_jog_speed_mm_s if self.max_jog_speed_mm_s > 0 else None

    def apply_controller_limits(self) -> None:
        max_speed = self.current_max_jog_speed_mm_s()
        if max_speed and self.settings.jog_speed_mm_s > max_speed:
            self.settings.jog_speed_mm_s = max_speed

    def _set_direction_pressed(self, axis: str, direction: int, pressed: bool) -> None:
        item = (axis, direction)
        if pressed:
            self.pressed_directions.add(item)
        else:
            self.pressed_directions.discard(item)
        self.apply_emulated_joystick()

    def emulated_joystick_motion(self) -> tuple[float, float]:
        logical_x = self._logical_direction("x")
        logical_y = self._logical_direction("y")
        return self.transform_logical_motion(float(logical_x), float(logical_y))

    def apply_emulated_joystick(self) -> None:
        x_norm, y_norm = self.emulated_joystick_motion()
        self.esp_worker.jog_normalized(x_norm, y_norm)

    def transform_logical_motion(
        self,
        logical_x: float,
        logical_y: float,
    ) -> tuple[float, float]:
        x_dir = float(logical_x)
        y_dir = float(logical_y)
        if self.settings.swap_xy:
            x_dir, y_dir = y_dir, x_dir
        if self.settings.flip_x:
            x_dir = -x_dir
        if self.settings.flip_y:
            y_dir = -y_dir
        return round(x_dir, 4), round(y_dir, 4)

    def _logical_direction(self, axis: str) -> int:
        positive = (axis, 1) in self.pressed_directions
        negative = (axis, -1) in self.pressed_directions
        if positive == negative:
            return 0
        return 1 if positive else -1

    def stop_all(self) -> None:
        self.pressed_directions.clear()
        self.esp_worker.stop_motion()
        self.statusBar().showMessage("Stop sent")

    def abort_motion(self) -> None:
        self.pressed_directions.clear()
        self.esp_worker.abort_motion()
        self.statusBar().showMessage("Abort sent")

    def enable_all_motors(self) -> None:
        if not self.esp_connected:
            QMessageBox.information(self, "Not connected", "Connect to the ESP300 first.")
            return
        self.esp_worker.enable_all_motors()
        self.statusBar().showMessage("Enable all motors sent")

    def disable_all_motors(self) -> None:
        if not self.esp_connected:
            QMessageBox.information(self, "Not connected", "Connect to the ESP300 first.")
            return
        self.pressed_directions.clear()
        self.esp_worker.disable_all_motors()
        self.statusBar().showMessage("Disable all motors sent")

    def toggle_motor_power(self) -> None:
        if self.all_motors_enabled:
            self.disable_all_motors()
        else:
            self.enable_all_motors()

    def poll_position(self) -> None:
        if not self.esp_connected:
            return
        self.esp_worker.request_poll()

    def update_axis_statuses(self, axis_states) -> None:
        x_state = axis_states.get(1)
        y_state = axis_states.get(2)
        self.motor_status.setText(
            f"X {self.format_motor_status(x_state)} | "
            f"Y {self.format_motor_status(y_state)}"
        )
        self.limit_status.setText(
            f"X {self.format_limit_status(x_state)} | "
            f"Y {self.format_limit_status(y_state)}"
        )
        if x_state is not None and y_state is not None:
            self.all_motors_enabled = (
                x_state.motor_enabled and y_state.motor_enabled
            )
        else:
            self.all_motors_enabled = None
        self.update_motor_power_button()

    def reset_axis_statuses(self) -> None:
        if not hasattr(self, "motor_status"):
            return
        self.all_motors_enabled = None
        self.motor_status.setText("--")
        self.limit_status.setText("--")
        self.update_motor_power_button()

    def update_motor_power_button(self) -> None:
        if not hasattr(self, "motor_power_button"):
            return
        connected = self.esp_connected
        self.motor_power_button.setEnabled(
            connected and self.all_motors_enabled is not None
        )
        if self.all_motors_enabled:
            self.motor_power_button.setText("Disable all motors")
        else:
            self.motor_power_button.setText("Enable all motors")

    def format_motor_status(self, state) -> str:
        if state is None:
            return "--"
        return "On" if state.motor_enabled else "Off"

    def format_limit_status(self, state) -> str:
        if state is None:
            return "--"
        neg = "High" if state.negative_limit_high else "Low"
        pos = "High" if state.positive_limit_high else "Low"
        return f"-{neg} +{pos}"

    def append_log(self, message: str) -> None:
        if not hasattr(self, "command_log"):
            return
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.command_log.appendPlainText(f"{timestamp}  {message}")

    def zero_position(self) -> None:
        if not self.esp_connected:
            QMessageBox.information(self, "Not connected", "Connect to the ESP300 first.")
            return
        self.esp_worker.zero_xy()
        self.current_position = (0.0, 0.0)
        self.x_readout.setText("0.000000")
        self.y_readout.setText("0.000000")
        self.statusBar().showMessage("Digital zero sent")

    def show_options(self) -> None:
        if self.esp_connected:
            self.esp_worker.refresh_max_velocity()
        max_speed = self.current_max_jog_speed_mm_s()
        dialog = OptionsDialog(self.settings, max_speed, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        self.stop_all()
        dialog.update_settings(self.settings)
        self.apply_controller_limits()
        self.esp_worker.set_poll_interval(self.settings.poll_interval_s)
        self.joystick_worker.update_settings(self.settings)
        if self.pressed_directions:
            self.apply_emulated_joystick()
        self.statusBar().showMessage("Options updated")
        self.save_config()

    def show_goto(self) -> None:
        if not self.esp_connected:
            QMessageBox.information(self, "Not connected", "Connect to the ESP300 first.")
            return
        dialog = GotoDialog(*self.current_position, parent=self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        self.stop_all()
        x_mm, y_mm = dialog.target
        self.esp_worker.goto_xy(x_mm, y_mm)
        self.statusBar().showMessage(f"Goto sent: X {x_mm:.6f}, Y {y_mm:.6f}")

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API name
        self.save_config()
        self.stop_workers()
        event.accept()

    def stop_workers(self) -> None:
        if hasattr(self, "joystick_worker"):
            self.joystick_worker.stop()
            self.joystick_worker.wait(1500)
        if hasattr(self, "joystick_manager"):
            self.joystick_manager.close()
        if hasattr(self, "esp_worker"):
            self.esp_worker.stop()
            self.esp_worker.wait(2000)

    def __del__(self) -> None:
        try:
            self.stop_workers()
        except Exception:
            pass


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(760, 520)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

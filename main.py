from __future__ import annotations

import sys
from datetime import datetime
from typing import Optional

from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QAction
from PySide6.QtWidgets import (
    QApplication,
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
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from esp300 import (
    ESP300Controller,
    ESP300Error,
    ESP300Settings,
    SERIAL_BAUDRATE,
    SERIAL_BYTESIZE,
    SERIAL_PARITY,
    SERIAL_STOPBITS,
    SerialTransport,
    VisaTransport,
    find_esp300_gpib_resources,
)


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

        self.settings = ESP300Settings()
        self.controller: Optional[ESP300Controller] = None
        self.current_position = (0.0, 0.0)
        self.pressed_directions: set[tuple[str, int]] = set()
        self.active_physical_dirs = {1: 0, 2: 0}

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.poll_position)
        self.poll_timer.start(int(self.settings.poll_interval_s * 1000))

        self._build_menu()
        self._build_ui()
        self._refresh_connection_ui()

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

        self.method_combo = QComboBox()
        self.method_combo.addItems(["RS232", "GPIB"])
        self.method_combo.currentIndexChanged.connect(self._on_method_changed)

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.currentTextChanged.connect(self._refresh_connection_ui)
        self.refresh_ports_button = QPushButton("Refresh")
        self.refresh_ports_button.clicked.connect(self.refresh_serial_ports)

        port_row = QHBoxLayout()
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(self.refresh_ports_button)

        self.rs232_info = QLabel(
            f"{SERIAL_BAUDRATE} baud, {SERIAL_BYTESIZE} data bits, "
            f"parity {SERIAL_PARITY}, {SERIAL_STOPBITS} stop bit, CR terminator"
        )
        self.rs232_info.setWordWrap(True)
        self.rtscts_check = QCheckBox("Use RTS/CTS hardware handshake")
        self.rtscts_check.setChecked(True)

        rs232_page = QWidget()
        rs232_layout = QFormLayout(rs232_page)
        rs232_layout.addRow("Port", port_row)
        rs232_layout.addRow("Parameters", self.rs232_info)
        rs232_layout.addRow("", self.rtscts_check)
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

        esp_layout.addRow("Method", self.method_combo)
        esp_layout.addRow(self.connection_stack)
        esp_layout.addRow(self.connect_button)
        esp_layout.addRow("Status", self.connection_status)

        joystick_group = QGroupBox("Joystick")
        joystick_layout = QVBoxLayout(joystick_group)
        joystick_layout.addWidget(QLabel("USB HID joystick support: TBD"))

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

        self.poll_label = QLabel("Poll: 0.500 s")
        self.zero_button = QPushButton("Zero")
        self.zero_button.clicked.connect(self.zero_position)
        self.motor_status = QLabel("--")
        self.limit_status = QLabel("--")

        layout.addWidget(QLabel("X mm"), 0, 0)
        layout.addWidget(self.x_readout, 0, 1)
        layout.addWidget(QLabel("Y mm"), 1, 0)
        layout.addWidget(self.y_readout, 1, 1)
        layout.addWidget(self.zero_button, 0, 2, 2, 1)
        layout.addWidget(QLabel("Motors"), 2, 0)
        layout.addWidget(self.motor_status, 2, 1, 1, 2)
        layout.addWidget(QLabel("End switches"), 3, 0)
        layout.addWidget(self.limit_status, 3, 1, 1, 2)
        layout.addWidget(self.poll_label, 4, 0)
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

        grid.addWidget(self.up_button, 0, 1)
        grid.addWidget(self.left_button, 1, 0)
        grid.addWidget(self.right_button, 1, 2)
        grid.addWidget(self.down_button, 2, 1)

        stop_row = QHBoxLayout()
        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_all)
        self.abort_button = QPushButton("Abort")
        self.abort_button.clicked.connect(self.abort_motion)
        stop_row.addWidget(self.stop_button)
        stop_row.addWidget(self.abort_button)

        motor_row = QHBoxLayout()
        self.enable_motors_button = QPushButton("Enable all motors")
        self.enable_motors_button.clicked.connect(self.enable_all_motors)
        self.disable_motors_button = QPushButton("Disable all motors")
        self.disable_motors_button.clicked.connect(self.disable_all_motors)
        motor_row.addWidget(self.enable_motors_button)
        motor_row.addWidget(self.disable_motors_button)

        outer.addLayout(grid)
        outer.addLayout(stop_row)
        outer.addLayout(motor_row)
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

    def _on_method_changed(self, index: int) -> None:
        self.connection_stack.setCurrentIndex(index)
        self._refresh_connection_ui()

    def toggle_connection(self) -> None:
        if self.controller and self.controller.is_connected:
            self.disconnect_controller()
        else:
            self.connect_controller()

    def connect_controller(self) -> None:
        method = self.method_combo.currentText()
        try:
            if method == "RS232":
                transport = SerialTransport(
                    self.port_combo.currentText().strip(),
                    rtscts=self.rtscts_check.isChecked(),
                    log_callback=self.append_log,
                )
            else:
                resource_name = self.selected_gpib_resource_name()
                if resource_name is None:
                    raise ESP300Error("No ESP300/ESP301 GPIB resource is selected")
                transport = VisaTransport(resource_name, log_callback=self.append_log)

            controller = ESP300Controller(transport)
            controller.connect()
            self.controller = controller
            self.apply_controller_limits()
            self.statusBar().showMessage(f"Connected via {method}")
            self.poll_position()
        except Exception as exc:
            self.controller = None
            self.reset_axis_statuses()
            QMessageBox.critical(self, "Connection failed", str(exc))
        self._refresh_connection_ui()

    def disconnect_controller(self) -> None:
        if self.controller:
            try:
                if self.controller.is_connected:
                    self.controller.stop_all()
            except ESP300Error:
                pass
            self.controller.close()
        self.controller = None
        self.pressed_directions.clear()
        self.active_physical_dirs = {1: 0, 2: 0}
        self.reset_axis_statuses()
        self.statusBar().showMessage("Disconnected")
        self._refresh_connection_ui()

    def _refresh_connection_ui(self, *_args) -> None:
        if not hasattr(self, "connect_button"):
            return
        connected = bool(self.controller and self.controller.is_connected)
        can_connect = connected or self.connection_selection_available()
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.connect_button.setEnabled(can_connect)
        self.connection_status.setText("Connected" if connected else "Not connected")
        self.method_combo.setEnabled(not connected)
        self.port_combo.setEnabled(not connected)
        self.refresh_ports_button.setEnabled(not connected)
        self.gpib_resource_combo.setEnabled(not connected)
        self.refresh_gpib_button.setEnabled(not connected)
        self.rtscts_check.setEnabled(not connected)

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
        if self.method_combo.currentText() == "RS232":
            return bool(self.port_combo.currentText().strip())
        return self.selected_gpib_resource_name() is not None

    def current_max_jog_speed_mm_s(self) -> Optional[float]:
        if not self.controller or not self.controller.is_connected:
            return None
        max_speed = self.controller.max_jog_speed_mm_s
        return max_speed if max_speed > 0 else None

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
        self.apply_joystick_state()

    def apply_joystick_state(self) -> None:
        desired = {1: 0, 2: 0}
        logical_x = self._logical_direction("x")
        logical_y = self._logical_direction("y")

        if self.controller:
            for axis, direction in (("x", logical_x), ("y", logical_y)):
                if direction:
                    physical_axis, physical_direction = (
                        self.controller.logical_axis_to_physical(
                            axis, direction, self.settings
                        )
                    )
                    desired[physical_axis] = physical_direction

        for axis in (1, 2):
            if desired[axis] == self.active_physical_dirs[axis]:
                continue
            self._change_axis_motion(axis, desired[axis])

    def _logical_direction(self, axis: str) -> int:
        positive = (axis, 1) in self.pressed_directions
        negative = (axis, -1) in self.pressed_directions
        if positive == negative:
            return 0
        return 1 if positive else -1

    def _change_axis_motion(self, axis: int, direction: int) -> None:
        if not self.controller or not self.controller.is_connected:
            self.active_physical_dirs[axis] = 0
            return
        try:
            if self.active_physical_dirs[axis] != 0:
                self.controller.stop_axis(axis)
            if direction != 0:
                self.controller.jog_axis(axis, direction, self.settings.jog_speed_mm_s)
            self.active_physical_dirs[axis] = direction
        except Exception as exc:
            self.active_physical_dirs[axis] = 0
            self.statusBar().showMessage(str(exc))
            QMessageBox.warning(self, "Jog command failed", str(exc))

    def stop_all(self) -> None:
        self.pressed_directions.clear()
        self.active_physical_dirs = {1: 0, 2: 0}
        if self.controller and self.controller.is_connected:
            try:
                self.controller.stop_all()
                self.statusBar().showMessage("Stop sent")
            except Exception as exc:
                QMessageBox.warning(self, "Stop failed", str(exc))

    def abort_motion(self) -> None:
        self.pressed_directions.clear()
        self.active_physical_dirs = {1: 0, 2: 0}
        if self.controller and self.controller.is_connected:
            try:
                self.controller.abort()
                self.statusBar().showMessage("Abort sent")
            except Exception as exc:
                QMessageBox.warning(self, "Abort failed", str(exc))

    def enable_all_motors(self) -> None:
        if not self.controller or not self.controller.is_connected:
            QMessageBox.information(self, "Not connected", "Connect to the ESP300 first.")
            return
        try:
            self.controller.enable_all_motors()
            self.poll_position()
            self.statusBar().showMessage("All motors enabled")
        except Exception as exc:
            QMessageBox.warning(self, "Enable motors failed", str(exc))

    def disable_all_motors(self) -> None:
        if not self.controller or not self.controller.is_connected:
            QMessageBox.information(self, "Not connected", "Connect to the ESP300 first.")
            return
        try:
            self.pressed_directions.clear()
            self.active_physical_dirs = {1: 0, 2: 0}
            self.controller.disable_all_motors()
            self.poll_position()
            self.statusBar().showMessage("All motors disabled")
        except Exception as exc:
            QMessageBox.warning(self, "Disable motors failed", str(exc))

    def poll_position(self) -> None:
        if not self.controller or not self.controller.is_connected:
            return
        try:
            snapshot = self.controller.read_snapshot()
            self.current_position = (snapshot.x_mm, snapshot.y_mm)
            self.x_readout.setText(f"{self.current_position[0]:.6f}")
            self.y_readout.setText(f"{self.current_position[1]:.6f}")
            self.update_axis_statuses(snapshot.axis_states)
            self.statusBar().showMessage("Position updated")
        except Exception as exc:
            self.statusBar().showMessage(f"Position poll failed: {exc}")

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

    def reset_axis_statuses(self) -> None:
        if not hasattr(self, "motor_status"):
            return
        self.motor_status.setText("--")
        self.limit_status.setText("--")

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
        if not self.controller or not self.controller.is_connected:
            QMessageBox.information(self, "Not connected", "Connect to the ESP300 first.")
            return
        try:
            self.controller.zero_xy()
            self.current_position = (0.0, 0.0)
            self.x_readout.setText("0.000000")
            self.y_readout.setText("0.000000")
            self.statusBar().showMessage("Digital zero set")
        except Exception as exc:
            QMessageBox.warning(self, "Zero failed", str(exc))

    def show_options(self) -> None:
        if self.controller and self.controller.is_connected:
            try:
                self.controller.refresh_max_velocities()
                self.apply_controller_limits()
            except Exception as exc:
                self.statusBar().showMessage(f"Could not refresh max velocity: {exc}")
        max_speed = self.current_max_jog_speed_mm_s()
        dialog = OptionsDialog(self.settings, max_speed, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        self.stop_all()
        dialog.update_settings(self.settings)
        self.apply_controller_limits()
        self.poll_timer.setInterval(int(self.settings.poll_interval_s * 1000))
        self.poll_label.setText(f"Poll: {self.settings.poll_interval_s:.3f} s")
        self.statusBar().showMessage("Options updated")

    def show_goto(self) -> None:
        if not self.controller or not self.controller.is_connected:
            QMessageBox.information(self, "Not connected", "Connect to the ESP300 first.")
            return
        dialog = GotoDialog(*self.current_position, parent=self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        try:
            self.stop_all()
            x_mm, y_mm = dialog.target
            self.controller.goto_xy_mm(x_mm, y_mm)
            self.statusBar().showMessage(f"Goto sent: X {x_mm:.6f}, Y {y_mm:.6f}")
        except Exception as exc:
            QMessageBox.warning(self, "Goto failed", str(exc))

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt API name
        self.disconnect_controller()
        event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(760, 520)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

from __future__ import annotations

import sys
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
    QLineEdit,
    QMainWindow,
        QMessageBox,
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
)


class OptionsDialog(QDialog):
    def __init__(self, settings: ESP300Settings, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Options")

        self.jog_speed = QDoubleSpinBox()
        self.jog_speed.setRange(0.001, 1000.0)
        self.jog_speed.setDecimals(4)
        self.jog_speed.setSuffix(" mm/s")
        self.jog_speed.setValue(settings.jog_speed_mm_s)

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

        root_layout.addWidget(self._build_connection_panel(), 0, 0)
        root_layout.addWidget(self._build_readout_panel(), 0, 1)
        root_layout.addWidget(self._build_joystick_panel(), 1, 0, 1, 2)

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
        self.gpib_resource_combo.setEditable(True)
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

        layout.addWidget(QLabel("X mm"), 0, 0)
        layout.addWidget(self.x_readout, 0, 1)
        layout.addWidget(QLabel("Y mm"), 1, 0)
        layout.addWidget(self.y_readout, 1, 1)
        layout.addWidget(self.poll_label, 2, 0)
        layout.addWidget(self.zero_button, 2, 1)
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

        outer.addLayout(grid)
        outer.addLayout(stop_row)
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
                )
            else:
                transport = VisaTransport(self.gpib_resource_combo.currentText().strip())

            controller = ESP300Controller(transport)
            controller.connect()
            self.controller = controller
            self.statusBar().showMessage(f"Connected via {method}")
            self.poll_position()
        except Exception as exc:
            self.controller = None
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
        self.statusBar().showMessage("Disconnected")
        self._refresh_connection_ui()

    def _refresh_connection_ui(self) -> None:
        connected = bool(self.controller and self.controller.is_connected)
        self.connect_button.setText("Disconnect" if connected else "Connect")
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

    def refresh_gpib_resources(self) -> None:
        current = self.gpib_resource_combo.currentText().strip() or "GPIB0::1::INSTR"
        resources = []
        try:
            import pyvisa

            resource_manager = pyvisa.ResourceManager()
            resources = [
                name
                for name in resource_manager.list_resources()
                if "GPIB" in name.upper()
            ]
            resource_manager.close()
        except Exception:
            resources = []

        if current and current not in resources:
            resources.insert(0, current)
        if not resources:
            resources = ["GPIB0::1::INSTR"]

        self.gpib_resource_combo.blockSignals(True)
        self.gpib_resource_combo.clear()
        self.gpib_resource_combo.addItems(resources)
        index = self.gpib_resource_combo.findText(current)
        self.gpib_resource_combo.setCurrentIndex(index if index >= 0 else 0)
        self.gpib_resource_combo.blockSignals(False)

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

    def poll_position(self) -> None:
        if not self.controller or not self.controller.is_connected:
            return
        try:
            self.current_position = self.controller.read_position_mm()
            self.x_readout.setText(f"{self.current_position[0]:.6f}")
            self.y_readout.setText(f"{self.current_position[1]:.6f}")
            self.statusBar().showMessage("Position updated")
        except Exception as exc:
            self.statusBar().showMessage(f"Position poll failed: {exc}")

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
        dialog = OptionsDialog(self.settings, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        self.stop_all()
        dialog.update_settings(self.settings)
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

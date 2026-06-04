from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol


SERIAL_BAUDRATE = 19200
SERIAL_BYTESIZE = 8
SERIAL_PARITY = "N"
SERIAL_STOPBITS = 1
SERIAL_WRITE_TERMINATOR = "\r"
SERIAL_READ_TERMINATOR = "\r\n"

GPIB_WRITE_TERMINATOR = "\r"
GPIB_READ_TERMINATOR = "\n"


class ESP300Error(RuntimeError):
    pass


class ESP300UnitError(ESP300Error):
    pass


class ESP300Transport(Protocol):
    def open(self) -> None:
        ...

    def close(self) -> None:
        ...

    def write(self, command: str) -> None:
        ...

    def query(self, command: str) -> str:
        ...

    @property
    def is_open(self) -> bool:
        ...


class SerialTransport:
    def __init__(
        self,
        port: str,
        timeout_s: float = 1.0,
        rtscts: bool = True,
    ) -> None:
        self.port = port
        self.timeout_s = timeout_s
        self.rtscts = rtscts
        self._serial = None

    @property
    def is_open(self) -> bool:
        return bool(self._serial and self._serial.is_open)

    def open(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise ESP300Error("pyserial is required for RS232 connections") from exc

        self._serial = serial.Serial(
            port=self.port,
            baudrate=SERIAL_BAUDRATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout_s,
            write_timeout=self.timeout_s,
            rtscts=self.rtscts,
        )

    def close(self) -> None:
        if self._serial:
            self._serial.close()

    def write(self, command: str) -> None:
        if not self._serial or not self._serial.is_open:
            raise ESP300Error("Serial connection is not open")
        payload = _command_payload(command)
        self._serial.write(payload.encode("ascii"))
        self._serial.flush()

    def query(self, command: str) -> str:
        if not self._serial or not self._serial.is_open:
            raise ESP300Error("Serial connection is not open")
        self._serial.reset_input_buffer()
        self.write(command)
        response = self._serial.readline().decode("ascii", errors="replace")
        if response == "":
            raise ESP300Error(f"Timed out waiting for response to {command!r}")
        return response.strip()


class VisaTransport:
    def __init__(self, resource_name: str, timeout_s: float = 1.0) -> None:
        self.resource_name = resource_name
        self.timeout_s = timeout_s
        self._resource_manager = None
        self._resource = None

    @property
    def is_open(self) -> bool:
        return self._resource is not None

    def open(self) -> None:
        try:
            import pyvisa
        except ImportError as exc:
            raise ESP300Error("pyvisa is required for GPIB connections") from exc

        self._resource_manager = pyvisa.ResourceManager()
        self._resource = self._resource_manager.open_resource(self.resource_name)
        self._resource.timeout = int(self.timeout_s * 1000)
        self._resource.write_termination = GPIB_WRITE_TERMINATOR
        self._resource.read_termination = GPIB_READ_TERMINATOR

    def close(self) -> None:
        if self._resource:
            self._resource.close()
            self._resource = None
        if self._resource_manager:
            self._resource_manager.close()
            self._resource_manager = None

    def write(self, command: str) -> None:
        if not self._resource:
            raise ESP300Error("VISA connection is not open")
        self._resource.write(_strip_command(command))

    def query(self, command: str) -> str:
        if not self._resource:
            raise ESP300Error("VISA connection is not open")
        return str(self._resource.query(_strip_command(command))).strip()


@dataclass
class AxisScale:
    unit_code: int = 2
    mm_per_encoder_count: Optional[float] = None
    mm_per_motor_step: Optional[float] = None

    def controller_units_from_mm(self, value_mm: float) -> float:
        return value_mm / self._mm_per_controller_unit()

    def mm_from_controller_units(self, value: float) -> float:
        return value * self._mm_per_controller_unit()

    def _mm_per_controller_unit(self) -> float:
        if self.unit_code == 0:
            if self.mm_per_encoder_count is None:
                raise ESP300UnitError(
                    "Axis is configured in encoder counts; set mm_per_encoder_count"
                )
            return self.mm_per_encoder_count
        if self.unit_code == 1:
            if self.mm_per_motor_step is None:
                raise ESP300UnitError(
                    "Axis is configured in motor steps; set mm_per_motor_step"
                )
            return self.mm_per_motor_step
        if self.unit_code == 2:
            return 1.0
        if self.unit_code == 3:
            return 0.001
        if self.unit_code == 4:
            return 25.4
        if self.unit_code == 5:
            return 0.0254
        if self.unit_code == 6:
            return 0.0000254
        raise ESP300UnitError(
            f"Axis unit code {self.unit_code} is not a linear displacement unit"
        )


@dataclass
class ESP300Settings:
    jog_speed_mm_s: float = 1.0
    poll_interval_s: float = 0.5
    flip_x: bool = False
    flip_y: bool = False
    swap_xy: bool = False


class ESP300Controller:
    def __init__(self, transport: ESP300Transport) -> None:
        self.transport = transport
        self.axis_scales = {
            1: AxisScale(),
            2: AxisScale(),
        }

    @property
    def is_connected(self) -> bool:
        return self.transport.is_open

    def connect(self) -> None:
        self.transport.open()
        self.refresh_axis_units()

    def close(self) -> None:
        self.transport.close()

    def refresh_axis_units(self) -> None:
        for axis in (1, 2):
            raw = self.transport.query(f"{axis}SN?")
            self.axis_scales[axis].unit_code = int(float(raw))

    def read_position_mm(self) -> tuple[float, float]:
        x = self.axis_scales[1].mm_from_controller_units(
            float(self.transport.query("1TP"))
        )
        y = self.axis_scales[2].mm_from_controller_units(
            float(self.transport.query("2TP"))
        )
        return x, y

    def zero_xy(self) -> None:
        self.transport.write("1DH0;2DH0")

    def jog_axis(self, axis: int, direction: int, speed_mm_s: float) -> None:
        if direction == 0:
            self.stop_axis(axis)
            return
        if axis not in (1, 2):
            raise ESP300Error(f"Unsupported axis {axis}")
        speed = abs(self.axis_scales[axis].controller_units_from_mm(speed_mm_s))
        sign = "+" if direction > 0 else "-"
        self.transport.write(f"{axis}VA{_fmt(speed)};{axis}MV{sign}")

    def stop_axis(self, axis: int) -> None:
        if axis not in (1, 2):
            raise ESP300Error(f"Unsupported axis {axis}")
        self.transport.write(f"{axis}ST")

    def stop_all(self) -> None:
        self.transport.write("ST")

    def abort(self) -> None:
        self.transport.write("AB")

    def goto_xy_mm(self, x_mm: float, y_mm: float) -> None:
        x = self.axis_scales[1].controller_units_from_mm(x_mm)
        y = self.axis_scales[2].controller_units_from_mm(y_mm)
        # Issuing both absolute moves on one line starts them as close together as
        # the controller command parser allows without leaving axes in a group.
        self.transport.write(f"1PA{_fmt(x)};2PA{_fmt(y)}")

    def logical_axis_to_physical(
        self,
        logical_axis: str,
        logical_direction: int,
        settings: ESP300Settings,
    ) -> tuple[int, int]:
        x_dir = logical_direction if logical_axis == "x" else 0
        y_dir = logical_direction if logical_axis == "y" else 0

        if settings.swap_xy:
            x_dir, y_dir = y_dir, x_dir
        if settings.flip_x:
            x_dir = -x_dir
        if settings.flip_y:
            y_dir = -y_dir

        if x_dir:
            return 1, x_dir
        if y_dir:
            return 2, y_dir
        raise ESP300Error("No joystick direction selected")


def _strip_command(command: str) -> str:
    return command.rstrip("\r\n")


def _command_payload(command: str) -> str:
    return _strip_command(command) + SERIAL_WRITE_TERMINATOR


def _fmt(value: float) -> str:
    return f"{value:.9g}"

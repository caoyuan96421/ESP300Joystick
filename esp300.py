from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Protocol


SERIAL_BAUDRATE = 19200
SERIAL_BYTESIZE = 8
SERIAL_PARITY = "N"
SERIAL_STOPBITS = 1
SERIAL_WRITE_TERMINATOR = "\r"
SERIAL_READ_TERMINATOR = "\r\n"

GPIB_WRITE_TERMINATOR = "\r"
GPIB_READ_TERMINATOR = "\n"

CommandLogger = Callable[[str], None]


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
        log_callback: Optional[CommandLogger] = None,
    ) -> None:
        self.port = port
        self.timeout_s = timeout_s
        self.rtscts = rtscts
        self.log_callback = log_callback
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
        self._log(f">> {_strip_command(command)}")
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
            self._log(f"!! timeout waiting for {_strip_command(command)}")
            raise ESP300Error(f"Timed out waiting for response to {command!r}")
        response = response.strip()
        self._log(f"<< {response}")
        return response

    def _log(self, message: str) -> None:
        if self.log_callback:
            self.log_callback(message)


class VisaTransport:
    def __init__(
        self,
        resource_name: str,
        timeout_s: float = 1.0,
        log_callback: Optional[CommandLogger] = None,
    ) -> None:
        self.resource_name = resource_name
        self.timeout_s = timeout_s
        self.log_callback = log_callback
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
        command = _strip_command(command)
        self._log(f">> {command}")
        self._resource.write(command)

    def query(self, command: str) -> str:
        if not self._resource:
            raise ESP300Error("VISA connection is not open")
        command = _strip_command(command)
        self._log(f">> {command}")
        response = str(self._resource.query(command)).strip()
        self._log(f"<< {response}")
        return response

    def _log(self, message: str) -> None:
        if self.log_callback:
            self.log_callback(message)


@dataclass(frozen=True)
class VisaResourceInfo:
    resource_name: str
    identity: str


@dataclass(frozen=True)
class AxisState:
    motor_enabled: bool
    negative_limit_high: bool
    positive_limit_high: bool


@dataclass(frozen=True)
class ControllerSnapshot:
    x_mm: float
    y_mm: float
    axis_states: dict[int, AxisState]


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
    joystick_poll_interval_s: float = 0.1
    low_deadband_percent: float = 5.0
    high_deadband_percent: float = 10.0
    joystick_exponent: float = 5.0
    rs232_rtscts: bool = True
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
        self.max_velocity_mm_s = {
            1: 0.0,
            2: 0.0,
        }

    @property
    def is_connected(self) -> bool:
        return self.transport.is_open

    def connect(self) -> None:
        self.transport.open()
        try:
            self.refresh_axis_units()
            self.refresh_max_velocities()
        except Exception:
            self.transport.close()
            raise

    def close(self) -> None:
        self.transport.close()

    def refresh_axis_units(self) -> None:
        for axis in (1, 2):
            raw = self.transport.query(f"{axis}SN?")
            self.axis_scales[axis].unit_code = int(float(raw))

    def refresh_max_velocities(self) -> None:
        for axis in (1, 2):
            raw = self.transport.query(f"{axis}VU?")
            self.max_velocity_mm_s[axis] = self.axis_scales[
                axis
            ].mm_from_controller_units(float(raw))

    @property
    def max_jog_speed_mm_s(self) -> float:
        values = [value for value in self.max_velocity_mm_s.values() if value > 0]
        if not values:
            return 0.0
        return min(values)

    def read_position_mm(self) -> tuple[float, float]:
        x = self.axis_scales[1].mm_from_controller_units(
            float(self.transport.query("1TP"))
        )
        y = self.axis_scales[2].mm_from_controller_units(
            float(self.transport.query("2TP"))
        )
        return x, y

    def read_snapshot(self) -> ControllerSnapshot:
        x, y = self.read_position_mm()
        return ControllerSnapshot(
            x_mm=x,
            y_mm=y,
            axis_states=self.read_axis_states(),
        )

    def read_axis_states(self) -> dict[int, AxisState]:
        hardware_registers = self.read_hardware_status_registers()
        register_1 = hardware_registers[0] if hardware_registers else 0
        states = {}
        for axis in (1, 2):
            motor_enabled = bool(int(float(self.transport.query(f"{axis}MO?"))))
            positive_bit = axis - 1
            negative_bit = axis + 7
            states[axis] = AxisState(
                motor_enabled=motor_enabled,
                negative_limit_high=bool(register_1 & (1 << negative_bit)),
                positive_limit_high=bool(register_1 & (1 << positive_bit)),
            )
        return states

    def read_hardware_status_registers(self) -> list[int]:
        response = self.transport.query("PH")
        registers = []
        for token in response.split(","):
            token = token.strip().rstrip("Hh")
            if token:
                registers.append(int(token, 16))
        return registers

    def zero_xy(self) -> None:
        self.transport.write("1DH0;2DH0")

    def enable_all_motors(self) -> None:
        self.transport.write("1MO;2MO")

    def disable_all_motors(self) -> None:
        self.transport.write("1MF;2MF")

    def jog_axis(self, axis: int, direction: int, speed_mm_s: float) -> None:
        if direction == 0:
            self.stop_axis(axis)
            return
        if axis not in (1, 2):
            raise ESP300Error(f"Unsupported axis {axis}")
        max_speed = self.max_velocity_mm_s.get(axis, 0.0)
        if max_speed > 0 and speed_mm_s > max_speed:
            raise ESP300Error(
                f"Requested jog speed {speed_mm_s:g} mm/s exceeds axis {axis} "
                f"maximum {max_speed:g} mm/s"
            )
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


def find_esp300_gpib_resources(timeout_s: float = 0.5) -> list[VisaResourceInfo]:
    try:
        import pyvisa
    except ImportError as exc:
        raise ESP300Error("pyvisa is required to scan GPIB resources") from exc

    resource_manager = pyvisa.ResourceManager()
    matches: list[VisaResourceInfo] = []
    try:
        resource_names = [
            name
            for name in resource_manager.list_resources()
            if "GPIB" in name.upper()
        ]
        for resource_name in resource_names:
            identity = _probe_esp300_resource(
                resource_manager, resource_name, timeout_s
            )
            if identity:
                matches.append(VisaResourceInfo(resource_name, identity))
    finally:
        resource_manager.close()
    return matches


def _probe_esp300_resource(resource_manager, resource_name: str, timeout_s: float) -> str:
    resource = None
    try:
        resource = resource_manager.open_resource(resource_name)
        resource.timeout = int(timeout_s * 1000)
        resource.write_termination = GPIB_WRITE_TERMINATOR
        resource.read_termination = GPIB_READ_TERMINATOR
        identity = str(resource.query("VE?")).strip()
        identity_upper = identity.upper()
        if (
            "ESP300" in identity_upper
            or "ESP301" in identity_upper
            or "ESP0300" in identity_upper
        ):
            return identity
    except Exception:
        return ""
    finally:
        if resource is not None:
            try:
                resource.close()
            except Exception:
                pass
    return ""

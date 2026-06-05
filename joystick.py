from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Optional


VID = 0x054C
PID = 0x0061
INTERFACE = 0
IN_ENDPOINT = 0x81
REPORT_LEN = 10
READ_TIMEOUT_MS = 1


@dataclass(frozen=True)
class JoystickReport:
    x_raw: int
    y_raw: int
    z_raw: int
    button_byte: int
    tail: bytes


class HIDJoystickManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._device = None
        self._connected = False
        self._last_error = ""

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    @property
    def last_error(self) -> str:
        with self._lock:
            return self._last_error

    def refresh_connection(self) -> tuple[bool, str]:
        with self._lock:
            try:
                import usb.core
                import usb.util
            except ImportError:
                return self._set_disconnected_locked("PyUSB is not installed")

            found = usb.core.find(idVendor=VID, idProduct=PID)
            if found is None:
                return self._set_disconnected_locked("Joystick not found")

            if self._connected and self._device is not None:
                return True, ""

            try:
                found.set_configuration()
                usb.util.claim_interface(found, INTERFACE)
            except Exception as exc:
                return self._set_disconnected_locked(f"Joystick open failed: {exc}")

            self._device = found
            self._connected = True
            self._last_error = ""
            return True, ""

    def read_latest(self) -> Optional[JoystickReport]:
        with self._lock:
            if not self._connected or self._device is None:
                return None
            data = None
            while True:
                try:
                    data = self._device.read(
                        IN_ENDPOINT, REPORT_LEN, READ_TIMEOUT_MS
                    )
                except Exception as exc:
                    if self._is_timeout(exc):
                        break
                    self._set_disconnected_locked(f"Joystick read failed: {exc}")
                    return None

            if data is None:
                return None
            return parse_report(data)

    def close(self) -> None:
        with self._lock:
            self._release_locked()
            self._connected = False
            self._last_error = ""

    def _set_disconnected_locked(self, error: str) -> tuple[bool, str]:
        self._release_locked()
        self._connected = False
        self._last_error = error
        return False, error

    def _release_locked(self) -> None:
        if self._device is None:
            return
        try:
            import usb.util

            usb.util.release_interface(self._device, INTERFACE)
        except Exception:
            pass
        self._device = None

    def _is_timeout(self, exc: Exception) -> bool:
        errno = getattr(exc, "errno", None)
        if errno in (60, 110, 116):
            return True
        return "timed out" in str(exc).lower() or "timeout" in str(exc).lower()


def parse_report(data) -> JoystickReport:
    if len(data) < REPORT_LEN:
        raise ValueError(f"Expected {REPORT_LEN}-byte report, got {len(data)}")
    return JoystickReport(
        x_raw=data[0] | (data[1] << 8),
        y_raw=data[2] | (data[3] << 8),
        z_raw=data[4] | (data[5] << 8),
        button_byte=data[6],
        tail=bytes(data[7:10]),
    )

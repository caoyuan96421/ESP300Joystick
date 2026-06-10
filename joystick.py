from __future__ import annotations

import sys
import threading
from dataclasses import dataclass
from typing import Optional


VID = 0x054C
PID = 0x0061
INTERFACE = 0
IN_ENDPOINT = 0x81
REPORT_LEN = 10
READ_TIMEOUT_MS = 1
BACKEND_INSTALL_HINT = (
    "Install on the machine running the app: "
    f"{sys.executable} -m pip install pyusb libusb-package"
)


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
        self._backend = ""
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

    @property
    def backend(self) -> str:
        with self._lock:
            return self._backend

    def refresh_connection(self) -> tuple[bool, str]:
        with self._lock:
            if self._connected and self._device is not None:
                return True, ""

            errors = []
            found, error = self._open_hidapi_device_locked()
            if found is None:
                errors.append(error)
                found, error = self._open_pyusb_device_locked()
            if found is None:
                errors.append(error)
                return self._set_disconnected_locked("; ".join(errors))

            self._device = found
            self._connected = True
            self._last_error = ""
            return True, ""

    def _open_hidapi_device_locked(self):
        try:
            import hid
        except ImportError as exc:
            return None, f"hidapi is not installed ({exc})"

        try:
            devices = hid.enumerate(VID, PID)
        except Exception as exc:
            return None, f"hidapi enumerate failed: {exc}"
        if not devices:
            return None, "hidapi: joystick not found"

        try:
            device = hid.device()
            path = devices[0].get("path")
            if path:
                device.open_path(path)
            else:
                device.open(VID, PID)
            if hasattr(device, "set_nonblocking"):
                device.set_nonblocking(False)
        except Exception as exc:
            return None, f"hidapi open failed: {exc}"

        self._backend = "hidapi"
        return device, ""

    def _open_pyusb_device_locked(self):
        try:
            import usb.core
            import usb.util
        except ImportError as exc:
            return None, f"PyUSB is not installed ({exc}). {BACKEND_INSTALL_HINT}"

        try:
            found = self._find_pyusb_device_locked(usb.core)
        except Exception as exc:
            return None, str(exc)
        if found is None:
            return None, "PyUSB: joystick not found"

        try:
            found.set_configuration()
            usb.util.claim_interface(found, INTERFACE)
        except Exception as exc:
            return None, f"PyUSB open failed: {exc}"

        self._backend = "pyusb"
        return found, ""

    def _find_pyusb_device_locked(self, usb_core):
        try:
            return usb_core.find(idVendor=VID, idProduct=PID)
        except Exception as exc:
            if exc.__class__.__name__ != "NoBackendError":
                raise
            try:
                import libusb_package
                import usb.backend.libusb1
            except ImportError as import_exc:
                raise RuntimeError(
                    "PyUSB/libusb backend package is not importable in this "
                    f"Python environment: {import_exc}. Run "
                    f"`{sys.executable} -m pip install --upgrade pyusb "
                    "libusb-package` "
                    "with the same Python used to start this app."
                ) from import_exc

            backend = None
            backend_errors = []
            if hasattr(libusb_package, "get_libusb1_backend"):
                try:
                    backend = libusb_package.get_libusb1_backend()
                except Exception as exc:
                    backend_errors.append(f"get_libusb1_backend failed: {exc}")
            if backend is None and hasattr(libusb_package, "find_library"):
                try:
                    backend = usb.backend.libusb1.get_backend(
                        find_library=self._libusb_package_find_library(
                            libusb_package
                        )
                    )
                except Exception as exc:
                    backend_errors.append(f"find_library backend failed: {exc}")
            if backend is None:
                detail = "; ".join(backend_errors) or "no backend factory found"
                raise RuntimeError(
                    "libusb-package is installed, but PyUSB could not load a "
                    f"libusb 1.0 backend ({detail})."
                )
            try:
                return usb_core.find(idVendor=VID, idProduct=PID, backend=backend)
            except Exception as backend_exc:
                raise RuntimeError(
                    f"PyUSB backend unavailable: {backend_exc}"
                ) from backend_exc

    def _libusb_package_find_library(self, libusb_package):
        def find_library(name=None):
            try:
                return libusb_package.find_library(name)
            except TypeError:
                return libusb_package.find_library()

        return find_library

    def read_latest(self) -> Optional[JoystickReport]:
        with self._lock:
            if not self._connected or self._device is None:
                return None
            if self._backend == "hidapi":
                return self._read_latest_hidapi_locked()
            return self._read_latest_pyusb_locked()

    def _read_latest_pyusb_locked(self) -> Optional[JoystickReport]:
        data = None
        while True:
            try:
                data = self._device.read(IN_ENDPOINT, REPORT_LEN, READ_TIMEOUT_MS)
            except Exception as exc:
                if self._is_timeout(exc):
                    break
                self._set_disconnected_locked(f"Joystick read failed: {exc}")
                return None

        if data is None:
            return None
        return parse_report(data)

    def _read_latest_hidapi_locked(self) -> Optional[JoystickReport]:
        data = None
        while True:
            try:
                packet = self._device.read(REPORT_LEN + 1, READ_TIMEOUT_MS)
            except Exception as exc:
                self._set_disconnected_locked(f"Joystick read failed: {exc}")
                return None
            if not packet:
                break
            data = packet

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
            self._backend = ""
            return
        if self._backend == "pyusb":
            try:
                import usb.util

                usb.util.release_interface(self._device, INTERFACE)
            except Exception:
                pass
        elif self._backend == "hidapi":
            try:
                self._device.close()
            except Exception:
                pass
        self._device = None
        self._backend = ""

    def _is_timeout(self, exc: Exception) -> bool:
        errno = getattr(exc, "errno", None)
        if errno in (60, 110, 116):
            return True
        return "timed out" in str(exc).lower() or "timeout" in str(exc).lower()


def parse_report(data) -> JoystickReport:
    data = list(data)
    if len(data) == REPORT_LEN + 1:
        data = data[1:]
    if len(data) < REPORT_LEN:
        raise ValueError(f"Expected {REPORT_LEN}-byte report, got {len(data)}")
    data = data[:REPORT_LEN]
    return JoystickReport(
        x_raw=data[0] | (data[1] << 8),
        y_raw=data[2] | (data[3] << 8),
        z_raw=data[4] | (data[5] << 8),
        button_byte=data[6],
        tail=bytes(data[7:10]),
    )

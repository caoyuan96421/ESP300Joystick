from __future__ import annotations

import platform
import sys
import traceback


def libusb_package_find_library(package):
    def find_library(name=None):
        try:
            return package.find_library(name)
        except TypeError:
            return package.find_library()

    return find_library


def report(label: str, func) -> None:
    print(f"\n[{label}]")
    try:
        result = func()
    except Exception:
        traceback.print_exc()
    else:
        if result is not None:
            print(result)


def main() -> int:
    print("Python executable:", sys.executable)
    print("Python version:", sys.version.replace("\n", " "))
    print("Platform:", platform.platform())
    print("Architecture:", platform.architecture())

    usb_core = None
    usb_backend_libusb1 = None
    libusb_package = None

    def import_pyusb():
        nonlocal usb_core, usb_backend_libusb1
        import usb
        import usb.backend.libusb1
        import usb.core

        usb_core = usb.core
        usb_backend_libusb1 = usb.backend.libusb1
        print("usb module:", getattr(usb, "__file__", "unknown"))
        return "PyUSB import OK"

    report("PyUSB import", import_pyusb)

    def import_libusb_package():
        nonlocal libusb_package
        import libusb_package as package

        libusb_package = package
        print("libusb_package module:", getattr(package, "__file__", "unknown"))
        print("has find_library:", hasattr(package, "find_library"))
        print(
            "has get_libusb1_backend:",
            hasattr(package, "get_libusb1_backend"),
        )
        return "libusb-package import OK"

    report("libusb-package import", import_libusb_package)

    def import_hidapi():
        import hid

        print("hid module:", getattr(hid, "__file__", "unknown"))
        print("has enumerate:", hasattr(hid, "enumerate"))
        print("has device:", hasattr(hid, "device"))
        return "hidapi import OK"

    report("hidapi import", import_hidapi)

    def default_backend():
        if usb_backend_libusb1 is None:
            return "Skipped; PyUSB import failed"
        backend = usb_backend_libusb1.get_backend()
        return f"default usb.backend.libusb1 backend: {backend!r}"

    report("Default libusb backend", default_backend)

    def packaged_library():
        if libusb_package is None:
            return "Skipped; libusb-package import failed"
        if hasattr(libusb_package, "get_library_path"):
            print(
                "libusb_package.get_library_path():",
                repr(libusb_package.get_library_path()),
            )
        if not hasattr(libusb_package, "find_library"):
            return "Skipped; libusb_package.find_library is unavailable"
        finder = libusb_package_find_library(libusb_package)
        return f"libusb_package.find_library(None): {finder(None)!r}"

    report("Packaged libusb DLL", packaged_library)

    def packaged_backend():
        if usb_backend_libusb1 is None or libusb_package is None:
            return "Skipped; required imports failed"

        backend = None
        if hasattr(libusb_package, "get_libusb1_backend"):
            backend = libusb_package.get_libusb1_backend()
        if backend is None and hasattr(libusb_package, "find_library"):
            backend = usb_backend_libusb1.get_backend(
                find_library=libusb_package_find_library(libusb_package)
            )
        return f"packaged libusb backend: {backend!r}"

    report("Packaged libusb backend", packaged_backend)

    def find_joystick():
        if usb_core is None or usb_backend_libusb1 is None:
            return "Skipped; PyUSB import failed"
        from joystick import PID, VID

        backend = None
        if libusb_package is not None and hasattr(libusb_package, "find_library"):
            backend = usb_backend_libusb1.get_backend(
                find_library=libusb_package_find_library(libusb_package)
            )
        device = usb_core.find(idVendor=VID, idProduct=PID, backend=backend)
        return f"joystick {VID:04X}:{PID:04X}: {device!r}"

    report("Joystick lookup", find_joystick)

    def find_joystick_hidapi():
        import hid
        from joystick import PID, VID

        devices = hid.enumerate(VID, PID)
        return f"hidapi joystick {VID:04X}:{PID:04X}: {devices!r}"

    report("Joystick lookup via hidapi", find_joystick_hidapi)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

# SMC71-USB USB HID Reference

This note is derived from the previously working `microscope_control.py`.
No hardware was connected during this pass, so fields not used by that script
are marked as unknown rather than guessed.

## Device Identity

The old controller opens the joystick directly with PyUSB:

```python
usb.core.find(idVendor=0x054C, idProduct=0x0061)
```

Keep these IDs for the ESP300 controller project:

| Field | Value |
| --- | --- |
| USB vendor ID | `0x054C` |
| USB product ID | `0x0061` |
| Interface used | `0` |
| Input endpoint | `0x81` |
| Input report length requested | `10` bytes |
| Read timeout | `1` ms |

For the ESP300 implementation, initialize the device in this order:

```python
device.set_configuration()
usb.util.claim_interface(device, 0)
device.read(0x81, 10, 1)
```

## Input Report Layout

Each input report is read as a 10-byte packet. The first 7 bytes are decoded by
the old script. Bytes 7 through 9 are read but unused.

When reading through `hidapi` on Windows, the same device may return an 8-byte
report instead of the 10-byte PyUSB interrupt packet. This is acceptable as long
as the first 7 decoded bytes are present. The current parser accepts shorter
reports and auto-detects whether byte `0` is an HID report ID or the X low byte.

| Byte offset | Field | Decode | Notes |
| --- | --- | --- | --- |
| `0` | X low byte | `x_raw = data[0] \| (data[1] << 8)` | Little-endian unsigned value |
| `1` | X high byte | See above | Treated as a 10-bit axis |
| `2` | Y low byte | `y_raw = data[2] \| (data[3] << 8)` | Little-endian unsigned value |
| `3` | Y high byte | See above | Treated as a 10-bit axis |
| `4` | Z low byte | `z_raw = data[4] \| (data[5] << 8)` | Little-endian unsigned value |
| `5` | Z high byte | See above | Treated as a 10-bit axis |
| `6` | Button byte | Bitmask, see table below | Old code only handled one active bit at a time |
| `7` | Unknown | Not decoded | Preserve for logging |
| `8` | Unknown | Not decoded | Preserve for logging |
| `9` | Unknown | Not decoded | Preserve for logging |

The axes are interpreted as centered around `0x0200` / `512`, with a nominal
range of `0x0000` to `0x03FF` / `1023`.

## Buttons

Byte `6` is an 8-bit button mask.

| Mask | Logical button |
| --- | --- |
| `0x01` | Button 1 |
| `0x02` | Button 2 |
| `0x04` | Button 3 |
| `0x08` | Button 4 |
| `0x10` | Button 5 |
| `0x20` | Button 6 |
| `0x40` | Button 7 |
| `0x80` | Button 8 |

The old `check_keyboard()` function compares `data[6]` against exact values,
so simultaneous button presses would not be reported correctly. New code should
treat byte `6` as a bitmask:

```python
button_1 = bool(data[6] & 0x01)
button_2 = bool(data[6] & 0x02)
button_3 = bool(data[6] & 0x04)
button_4 = bool(data[6] & 0x08)
button_5 = bool(data[6] & 0x10)
button_6 = bool(data[6] & 0x20)
button_7 = bool(data[6] & 0x40)
button_8 = bool(data[6] & 0x80)
```

The old microscope application assigned arbitrary actions to a few of these
buttons for a different system. Those action meanings should not be reused for
the ESP300 controller. Treat byte `6` as raw button state until the desired
ESP300 behavior is defined.

Physical labels for the buttons are not recoverable from the code alone.

## Axis Calibration From Old Code

The old microscope script applies per-axis endpoint calibration before mapping
motion:

| Axis | Minimum observed/used | Center | Maximum observed/used |
| --- | --- | --- | --- |
| X | `2` | `512` | `982` |
| Y | `46` | `512` | `1023` |
| Z | `63` | `512` | `1023` |

After calibration, the script maps each axis back around the ideal center of
`512`. It then uses a deadband:

| Axis | Neutral condition after calibration |
| --- | --- |
| X | `abs(x - 512) < 32` |
| Y | `abs(y - 512) < 32` |
| Z | `abs(z - 512) < 16` |

For the old microscope motion mapping, X was inverted, while Y and Z were not:

```python
x_speed = -map_value(x, 0x0000, 0x03FF, 4000)
y_speed =  map_value(y, 0x0000, 0x03FF, 4000)
z_speed =  map_value_z(z, 0x0000, 0x03FF, 31000)
```

That sign convention belongs to the old microscope coordinate system. The
ESP300 controller should choose signs based on the actuator orientation.

## Reading Strategy

The old `read_hid_data()` function effectively drains the endpoint and returns
the freshest report:

1. Start with `data = None`.
2. Call `device.read(0x81, 10, 1)` in a tight loop.
3. If a USB timeout/error happens before any packet was read, keep waiting.
4. If a USB timeout/error happens after at least one packet was read, return
   the most recent packet.

The surrounding control loop sleeps to a period of about `0.04` s, or 25 Hz.

This behavior is useful for motion control because stale queued joystick states
are discarded.

## Minimal Parser

```python
VID = 0x054C
PID = 0x0061
IN_ENDPOINT = 0x81
REPORT_LEN = 10

BUTTON_MASKS = {
    "button_1": 0x01,
    "button_2": 0x02,
    "button_3": 0x04,
    "button_4": 0x08,
    "button_5": 0x10,
    "button_6": 0x20,
    "button_7": 0x40,
    "button_8": 0x80,
}

def parse_smc71_report(data):
    if len(data) < 10:
        raise ValueError(f"Expected 10-byte report, got {len(data)} bytes")

    x_raw = data[0] | (data[1] << 8)
    y_raw = data[2] | (data[3] << 8)
    z_raw = data[4] | (data[5] << 8)
    button_byte = data[6]

    return {
        "x_raw": x_raw,
        "y_raw": y_raw,
        "z_raw": z_raw,
        "buttons": {
            name: bool(button_byte & mask)
            for name, mask in BUTTON_MASKS.items()
        },
        "button_byte": button_byte,
        "unknown_tail": bytes(data[7:10]),
    }
```

## LED Control

Ignore joystick LED control for now. The old script does not contain any USB
output report, feature report, or control transfer for LEDs.

## Open Questions For Hardware Testing

- Confirm whether bytes `7`, `8`, and `9` ever change.
- Confirm whether the report has an implicit or omitted report ID. The old
  code treats byte `0` as X low byte, so no report ID is visible in this path.
- Confirm actual physical labels for buttons 1 through 8.
- Recalibrate X, Y, and Z min/max values on the actual joystick that will be
  used with the ESP300.

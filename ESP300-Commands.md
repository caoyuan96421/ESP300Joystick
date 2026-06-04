# ESP300 Command Notes For Joystick Control

Source checked: Newport `ESP300 Motion Controller/Driver User's Manual`.

Manual URL:
https://www.newport.com.cn/medias/sys_master/images/images/h2f/haa/8797091299358/ESP300-User-Manual.pdf

## Command Syntax

The ESP300 command format is:

```text
{axis_or_group}{COMMAND}{parameter}\r
```

Multiple commands can be sent on one command line separated by semicolons:

```text
1VA2.0;1MV+;2VA1.5;2MV-\r
```

Command lines must end with carriage return `\r`. Responses are terminated by
carriage return plus line feed `\r\n`.

## RS232 Parameters

The manual states that the RS232 configuration is fixed at:

| Parameter | Value |
| --- | --- |
| Baud rate | `19200` factory default |
| Data bits | `8` |
| Parity | none |
| Stop bits | `1` |
| Flow control | RTS/CTS hardware handshake |
| Command terminator | carriage return, `\r` |
| Response terminator | carriage return plus line feed, `\r\n` |

For GPIB/IEEE-488, the controller still uses the same ESP ASCII command set.
The manual says to terminate reads on line feed.

## Unit Handling

Do not assume motion values are in millimeters. The controller has a per-axis
unit setting:

```text
1SN?
2SN?
```

`SN` unit codes:

| Code | Unit label |
| --- | --- |
| `0` | encoder count |
| `1` | motor step |
| `2` | millimeter |
| `3` | micrometer |
| `4` | inch |
| `5` | milli-inch |
| `6` | micro-inch |
| `7` | degree |
| `8` | gradient |
| `9` | radian |
| `10` | milliradian |
| `11` | microradian |

The manual says the unit setting is a label only. Changing `SN` does not
automatically convert existing position, velocity, acceleration, deceleration, or
limit values. Therefore the joystick controller should not silently issue `SN`
commands during startup.

Use millimeters internally in Python and convert at the command boundary.

### Conversion To Controller Units

For linear axes:

| `SN` code | Controller value for a position in `mm` |
| --- | --- |
| `0` encoder count | `mm / mm_per_encoder_count` |
| `1` motor step | `mm / mm_per_motor_step` |
| `2` millimeter | `mm` |
| `3` micrometer | `mm * 1000` |
| `4` inch | `mm / 25.4` |
| `5` milli-inch | `mm / 0.0254` |
| `6` micro-inch | `mm / 0.0000254` |

Velocity uses the same scale per second. Acceleration and deceleration use the
same scale per second squared.

If an axis reports `SN=0` or `SN=1`, the controller is using raw counts or raw
steps. The Python controller must know the physical scale for that actuator:

```text
mm_per_encoder_count
mm_per_motor_step
```

The controller settings `SU?` and `FR?` are relevant diagnostics:

```text
1SU?    # encoder resolution, in user-defined units
1FR?    # encoder full-step resolution, for compatible stepper axes
2SU?
2FR?
```

Still, the safest implementation is to keep an explicit per-axis calibration in
the Python config and verify it with a known move.

### Conversion From Controller Units

For `TP`, `HP`, `TV`, and similar responses:

| `SN` code | Position response converted to `mm` |
| --- | --- |
| `0` encoder count | `value * mm_per_encoder_count` |
| `1` motor step | `value * mm_per_motor_step` |
| `2` millimeter | `value` |
| `3` micrometer | `value / 1000` |
| `4` inch | `value * 25.4` |
| `5` milli-inch | `value * 0.0254` |
| `6` micro-inch | `value * 0.0000254` |

Angular units should be rejected for this XY linear-actuator application unless a
future configuration deliberately maps them to linear travel.

## Startup Queries

At startup, query and cache:

```text
1SN?;2SN?
1SU?;2SU?
1FR?;2FR?
1TP;2TP
PH
```

If both axes are configured as millimeters, commands can be sent directly in mm.
If not, convert all positions, velocities, acceleration, and deceleration through
the per-axis scale.

For coordinated group motion, prefer both axes to have the same physical unit
scale. If X and Y are in different controller units, `HL` targets can still be
converted per axis, but the vector velocity commands `HV`, `HA`, and `HD` are
much easier and safer when the group coordinate system is millimeter-based.

## Required Functions

### Read X/Y Position

```text
1TP
2TP
```

`TP` returns the actual position in the axis' configured/predefined units.
Convert the numeric response to mm before exposing it to the rest of the app.

### Set Digital Zero

```text
1DH0;2DH0
```

`DH` defines the current position as the supplied HOME value. Sending `DH0` for
X and Y presets the current physical position to digital coordinate zero without
running a hardware home-search move.

For a configured XY group:

```text
1HP
```

`HP` returns comma-separated positions of all axes in the group.

### Read Motion Status

Per axis:

```text
1MD?
2MD?
```

`MD?` returns:

| Value | Meaning |
| --- | --- |
| `0` | motion not done, axis is still moving |
| `1` | motion done |

Controller-wide compact status:

```text
TS
```

`TS` returns an ASCII character that must be converted to a byte with
`ord(response[0])`. Mask the low bits for the useful status flags:

```python
status = ord(response[0]) & 0x1F
```

Relevant low bits:

| Bit | Meaning when high |
| --- | --- |
| `0` | axis 1 in motion |
| `1` | axis 2 in motion |
| `2` | axis 3 in motion |
| `4` | motor power is on for at least one axis |

### Read Hardware Limit Switch Signals

```text
PH
```

`PH` returns hardware status registers in hexadecimal notation.
For register 1:

| Bit | Signal |
| --- | --- |
| `0` | axis 1 positive hardware travel limit |
| `1` | axis 2 positive hardware travel limit |
| `8` | axis 1 negative hardware travel limit |
| `9` | axis 2 negative hardware travel limit |

The bit value reports whether the signal is low or high. Whether high means
"limit reached" depends on the limit switch wiring and controller
configuration, so this should be verified on hardware.

### Start Jog Motion At A Given Velocity

For each axis, set speed with `VA` and direction with `MV+` or `MV-`:

```text
1VA{abs_x_velocity};1MV+
1VA{abs_x_velocity};1MV-
2VA{abs_y_velocity};2MV+
2VA{abs_y_velocity};2MV-
```

`VA` is in configured axis units per second. Convert requested `mm/s` before
sending. `MV` starts indefinite motion using the predefined acceleration and
velocity.

Start both axes close together by packing the commands on one line:

```text
1VA{vx};1MV{dir_x};2VA{vy};2MV{dir_y}\r
```

Do not reverse a moving axis directly. Stop the axis first, wait for `MD?` or a
short settling interval, then issue the opposite `MV` direction.

### Stop Motion Gradually

Per axis:

```text
1ST
2ST
```

All axes:

```text
ST
```

`ST` stops using the deceleration set by `AG`.

For a coordinated group:

```text
1HS
```

`HS` stops the group using the vector deceleration set by `HD`.

### Emergency Stop

```text
AB
```

`AB` invokes emergency-stop behavior for all axes, as configured by `ZE` and
`AE`. The manual notes the default behavior can turn motor power off.

### Move To A Given X/Y Coordinate Simultaneously

Use group motion for true coordinated XY positioning:

```text
1HN1,2
1HV{vector_velocity}
1HA{vector_acceleration}
1HD{vector_deceleration}
1HO
1HL{x_target},{y_target}
```

`HN` creates group 1 from physical axes 1 and 2. `HL` moves the group along a
line to the target positions. Convert `x_target` and `y_target` from mm to each
axis' controller units before sending, unless both axes are already configured
and verified as millimeters.

Important: once axes are assigned to a group, individual `PA`, `PR`, and `MV`
commands for those axes can be rejected. For joystick jogging, either avoid
leaving axes grouped during jog mode, or create/use the group only for explicit
coordinated positioning moves.

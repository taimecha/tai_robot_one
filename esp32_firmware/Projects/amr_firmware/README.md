# AMR base-drive ESP32 firmware

Production firmware for the four independently driven wheels. It deliberately
contains no Wi-Fi or HTTP server. A Raspberry Pi 5 will connect to this ESP32
through its own USB serial port; `tai_robot_one/TaiRobotSerialSystem` uses the
protocol below.

The controller powers up **disabled with zero PWM**. Test with all four wheels
raised before allowing the vehicle to touch the floor.

## Wheel mapping

USB fields and telemetry always use this canonical order:

| Index | Joint / field | Existing web command preserved | RPWM | LPWM | Encoder A | Encoder B |
|---:|---|---|---:|---:|---:|---:|
| 0 | `front_left` (`FL`) | `fl` | 33 | 32 | 4 | 23 |
| 1 | `front_right` (`FR`) | `fr` | 14 | 27 | 34 | 35 |
| 2 | `rear_left` (`RL`) | `bl` | 25 | 26 | 19 | 18 |
| 3 | `rear_right` (`RR`) | `br` | 13 | 22 | 16 | 17 |

The table is copied directly from the supplied PID tuner and reordered from
`FR, FL, BL, BR` into the ROS-facing `FL, FR, RL, RR` protocol order.

`command_sign` and `encoder_sign` are explicit in `src/main.cpp`. They default
to `+1` to preserve the old controller. Raise the chassis and verify each wheel
individually; change only the sign for a row if physical forward or encoder
positive is reversed.

## Limits and watchdog

- Wheel radius: `0.0875 m`, matching the robot Xacro.
- Maximum vehicle/wheel tangential speed: `0.5 m/s`.
- Maximum accepted wheel velocity: `5.71429 rad/s` (`54.57 RPM`).
- Control loop: 50 Hz; telemetry: 25 Hz.
- No fresh `CMD` for 300 ms: target becomes zero and the acceleration limiter
  performs a controlled stop; `COMMAND_TIMEOUT` latches.
- No fresh `CMD` for 1000 ms: PWM is cut and the drive is disabled.
- `PING` proves that the link is alive but intentionally does **not** renew the
  motion lease. Only a valid `CMD` can do that.
- Recovery requires a fresh all-zero `CMD`, then `CLEAR`, then `ENABLE`. This
  prevents reconnecting software from replaying an old non-zero command.

The software `ESTOP` command is an additional interlock, not a substitute for a
hard-wired emergency-stop circuit that removes motor-driver enable/power.

## USB serial protocol v1

Settings: 460800 baud, 8-N-1. Every frame is one ASCII line:

```text
@PAYLOAD*CCCC\n
```

`CCCC` is uppercase CRC-16/CCITT-FALSE of `PAYLOAD` only (initial value
`0xFFFF`, polynomial `0x1021`). Maximum received line length is 255 bytes.
Sequence numbers are unsigned 32-bit values and may wrap.

Host-to-ESP32 frames:

```text
@CMD,1,SEQ,FL_RAD_S,FR_RAD_S,RL_RAD_S,RR_RAD_S*CCCC
@ENABLE,1,SEQ*CCCC
@DISABLE,1,SEQ*CCCC
@ESTOP,1,SEQ*CCCC
@CLEAR,1,SEQ*CCCC
@PING,1,SEQ*CCCC
```

Send `CMD` at 50 Hz. Values outside `+/-5.71429 rad/s`, non-finite values,
wrong field counts, malformed CRCs, and non-zero commands while inhibited are
rejected. Normal startup is:

1. Send all-zero `CMD` and wait for its `ACK` / echoed sequence in `STATE`.
2. If a fault is present, send `CLEAR`.
3. Send `ENABLE`.
4. Start sending fresh wheel commands continuously.

Replies are:

```text
@ACK,1,SEQ,OK,COMMAND*CCCC
@NACK,1,SEQ,REASON,COMMAND*CCCC
```

The state frame is positional to keep parsing deterministic:

```text
@STATE,1,STATE_SEQ,LAST_CMD_SEQ,UPTIME_MS,STATUS,FAULT,CMD_AGE_MS,
  FL_POS,FR_POS,RL_POS,RR_POS,
  FL_VEL,FR_VEL,RL_VEL,RR_VEL,
  FL_TARGET,FR_TARGET,RL_TARGET,RR_TARGET,
  FL_PWM,FR_PWM,RL_PWM,RR_PWM,RX_ERRORS,LAST_VALID_FRAME_AGE_MS*CCCC
```

The actual wire frame contains no spaces or line breaks. Position is radians;
velocity and ramped target are radians/second. Encoder position and velocity
are measured feedback, not open-loop estimates.

Status bits:

| Bit | Value | Meaning |
|---:|---:|---|
| 0 | 1 | drive enabled |
| 1 | 2 | motion command younger than 300 ms |
| 2 | 4 | watchdog-controlled soft stop active |
| 3 | 8 | software E-stop latched |
| 4 | 16 | at least one PWM output is non-zero |

Fault bits:

| Bit | Value | Meaning |
|---:|---:|---|
| 0 | 1 | command timeout |
| 1 | 2 | software E-stop |

## Build

```bash
cd esp32_firmware/Projects/amr_firmware
pio run
```

The reusable PID implementation is the single file
`../../Common/amr_drive_pid/src/pid.cpp`; its header contains only declarations
and configuration types. Firmware production này chỉ giao tiếp USB Serial
460800 baud qua `/dev/tai_drive`; Wi-Fi và Bluetooth luôn tắt.

# AMR wheel PID web tuner

This is the original Wi-Fi/HTTP bench application, separated from the
production USB firmware. It is intended only for tuning with the chassis
secured and the wheels raised.

- Access point: `AMR_Forklift_40kg`
- Address: `http://192.168.4.1`
- The speed input is limited to 54.57 RPM, equivalent to 0.5 m/s for the
  87.5 mm wheel radius.
- While a non-zero command is active, the page refreshes it every 200 ms.
- If the ESP32 receives no refresh for 500 ms, its local watchdog commands a
  soft stop. Hiding/closing the page also requests a stop.
- The shared controller is implemented only in
  `../../Common/amr_drive_pid/src/pid.cpp`, so gains tested here use the same
  PID code as production.

Wheel fields retain the existing web behavior (`FR, FL, BL, BR`). See the
production project's README for the exact GPIO mapping and ROS order.

```bash
cd esp32_firmware/Projects/amr_pid_tuner
pio run
```

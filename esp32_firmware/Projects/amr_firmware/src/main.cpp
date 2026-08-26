#include <Arduino.h>
#include <ESP32Encoder.h>
#include <esp_arduino_version.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "pid.hpp"

namespace {

constexpr size_t kWheelCount = 4;
constexpr uint32_t kSerialBaud = 460800;
constexpr uint32_t kControlPeriodUs = 20000;       // 50 Hz
constexpr uint32_t kTelemetryPeriodMs = 40;        // 25 Hz
constexpr uint32_t kCommandSoftTimeoutMs = 300;
constexpr uint32_t kCommandHardTimeoutMs = 1000;
constexpr uint16_t kPwmFrequencyHz = 5000;
constexpr uint8_t kPwmResolutionBits = 8;
constexpr float kEncoderCountsPerRevolution = 1200.0f;
constexpr float kWheelRadiusM = 0.0875f;
constexpr float kMaxVehicleSpeedMps = 0.5f;
constexpr float kMaxWheelRadPerSec = kMaxVehicleSpeedMps / kWheelRadiusM;
constexpr float kMaxWheelRpm =
    kMaxWheelRadPerSec * 60.0f / (2.0f * PI);
constexpr float kMaxAccelerationRpmPerSec = 320.0f;
constexpr float kVelocityFilterAlpha = 0.2f;
constexpr uint8_t kProtocolVersion = 1;
constexpr size_t kMaxRxLineLength = 255;

// Canonical order used on USB and by ros2_control: FL, FR, RL, RR.
enum WheelIndex : size_t {
  kFrontLeft = 0,
  kFrontRight = 1,
  kRearLeft = 2,
  kRearRight = 3,
};

struct MotorPins {
  const char *name;
  uint8_t rpwm;
  uint8_t lpwm;
  uint8_t encoder_a;
  uint8_t encoder_b;
  int8_t command_sign;
  int8_t encoder_sign;
};

// This table preserves the behavior of the original web commands:
//   web FL -> GPIO 27/14, encoder 18/19
//   web FR -> GPIO 32/33, encoder 23/4
//   web BL -> GPIO 22/13, encoder 16/17
//   web BR -> GPIO 25/26, encoder 34/35
// Verify one raised wheel at a time before floor testing. If a wheel is
// reversed, change only that row's command_sign and/or encoder_sign.
constexpr MotorPins kMotorPins[kWheelCount] = {
    {"FL", 27, 14, 18, 19, +1, +1},
    {"FR", 32, 33, 23, 4, +1, +1},
    {"RL", 22, 13, 16, 17, +1, +1},
    {"RR", 25, 26, 34, 35, +1, +1},
};

constexpr amr_drive::PidConfig kPidConfig = {
    0.3f,   // Kp
    0.02f,  // Ki per 20 ms sample
    0.01f,  // Kd per 20 ms sample
    1.47f,  // feed-forward PWM/RPM
    3,      // static-friction feed-forward PWM
    255.0f,
    255,
};

enum StatusBits : uint16_t {
  kStatusEnabled = 1U << 0,
  kStatusCommandFresh = 1U << 1,
  kStatusSoftStopping = 1U << 2,
  kStatusEstopLatched = 1U << 3,
  kStatusMotorOutputActive = 1U << 4,
};

enum FaultBits : uint16_t {
  kFaultNone = 0,
  kFaultCommandTimeout = 1U << 0,
  kFaultSoftwareEstop = 1U << 1,
};

struct MotorRuntime {
  int64_t encoder_count = 0;
  int64_t previous_encoder_count = 0;
  float raw_rpm = 0.0f;
  float measured_rpm = 0.0f;
  float requested_rad_s = 0.0f;
  float ramped_rpm = 0.0f;
  int16_t output_pwm = 0;
  amr_drive::VelocityPid pid;
};

ESP32Encoder encoders[kWheelCount];
MotorRuntime motors[kWheelCount];

char rx_line[kMaxRxLineLength + 1];
size_t rx_length = 0;
bool rx_overflow = false;
uint32_t rx_error_count = 0;
uint32_t tx_state_sequence = 0;
uint32_t last_command_sequence = 0;
uint32_t last_command_ms = 0;
uint32_t last_valid_frame_ms = 0;
uint32_t last_control_us = 0;
uint32_t last_telemetry_ms = 0;
bool have_motion_command = false;
bool drive_enabled = false;
bool soft_timeout_active = false;
uint16_t fault_bits = kFaultNone;

float rpmToRadPerSec(float rpm) {
  return rpm * (2.0f * PI / 60.0f);
}

float radPerSecToRpm(float rad_s) {
  return rad_s * (60.0f / (2.0f * PI));
}

float moveTowards(float current, float target, float maximum_step) {
  if (current < target) {
    return fminf(current + maximum_step, target);
  }
  if (current > target) {
    return fmaxf(current - maximum_step, target);
  }
  return target;
}

bool allRequestedSpeedsAreZero() {
  for (size_t i = 0; i < kWheelCount; ++i) {
    if (fabsf(motors[i].requested_rad_s) > 0.001f) {
      return false;
    }
  }
  return true;
}

void setRequestedSpeedsToZero() {
  for (size_t i = 0; i < kWheelCount; ++i) {
    motors[i].requested_rad_s = 0.0f;
  }
}

void attachPwm(size_t wheel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(kMotorPins[wheel].rpwm, kPwmFrequencyHz, kPwmResolutionBits);
  ledcAttach(kMotorPins[wheel].lpwm, kPwmFrequencyHz, kPwmResolutionBits);
#else
  const uint8_t right_channel = static_cast<uint8_t>(wheel * 2);
  const uint8_t left_channel = static_cast<uint8_t>(right_channel + 1);
  ledcSetup(right_channel, kPwmFrequencyHz, kPwmResolutionBits);
  ledcSetup(left_channel, kPwmFrequencyHz, kPwmResolutionBits);
  ledcAttachPin(kMotorPins[wheel].rpwm, right_channel);
  ledcAttachPin(kMotorPins[wheel].lpwm, left_channel);
#endif
}

void writePwmPin(size_t wheel, bool right_input, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(right_input ? kMotorPins[wheel].rpwm : kMotorPins[wheel].lpwm,
            duty);
#else
  const uint8_t channel = static_cast<uint8_t>(wheel * 2 +
      (right_input ? 0 : 1));
  ledcWrite(channel, duty);
#endif
}

void writeMotorOutput(size_t wheel, int16_t canonical_pwm) {
  int16_t physical_pwm = canonical_pwm * kMotorPins[wheel].command_sign;
  physical_pwm = constrain(physical_pwm, -255, 255);

  if (physical_pwm > 0) {
    writePwmPin(wheel, true, static_cast<uint8_t>(physical_pwm));
    writePwmPin(wheel, false, 0);
  } else if (physical_pwm < 0) {
    writePwmPin(wheel, true, 0);
    writePwmPin(wheel, false, static_cast<uint8_t>(-physical_pwm));
  } else {
    writePwmPin(wheel, true, 0);
    writePwmPin(wheel, false, 0);
  }
}

void hardStopMotorOutputs() {
  for (size_t i = 0; i < kWheelCount; ++i) {
    motors[i].requested_rad_s = 0.0f;
    motors[i].ramped_rpm = 0.0f;
    motors[i].output_pwm = 0;
    motors[i].pid.reset(motors[i].measured_rpm);
    writeMotorOutput(i, 0);
  }
}

uint16_t crc16Ccitt(const char *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(static_cast<uint8_t>(data[i])) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void sendPayload(const char *payload) {
  const uint16_t crc = crc16Ccitt(payload, strlen(payload));
  Serial.print('@');
  Serial.print(payload);
  Serial.print('*');
  char crc_text[5];
  snprintf(crc_text, sizeof(crc_text), "%04X", crc);
  Serial.print(crc_text);
  Serial.print('\n');
}

void sendReply(const char *kind,
               uint32_t sequence,
               const char *code,
               const char *detail) {
  char payload[128];
  snprintf(payload,
           sizeof(payload),
           "%s,%u,%lu,%s,%s",
           kind,
           kProtocolVersion,
           static_cast<unsigned long>(sequence),
           code,
           detail);
  sendPayload(payload);
}

bool parseUint32(const char *text, uint32_t &value) {
  if (text == nullptr || *text == '\0' || *text == '-') {
    return false;
  }
  char *end = nullptr;
  const unsigned long parsed = strtoul(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }
  value = static_cast<uint32_t>(parsed);
  return true;
}

bool parseFloatStrict(const char *text, float &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char *end = nullptr;
  value = strtof(text, &end);
  return end != text && *end == '\0' && isfinite(value);
}

bool parseHeader(char *&save,
                 uint32_t &sequence,
                 const char *command_name) {
  char *version_text = strtok_r(nullptr, ",", &save);
  char *sequence_text = strtok_r(nullptr, ",", &save);
  uint32_t version = 0;
  if (!parseUint32(version_text, version) || version != kProtocolVersion ||
      !parseUint32(sequence_text, sequence)) {
    ++rx_error_count;
    sendReply("NACK", sequence, "BAD_HEADER", command_name);
    return false;
  }
  return true;
}

void handleMotionCommand(char *save, uint32_t sequence, uint32_t now_ms) {
  float values[kWheelCount];
  for (size_t i = 0; i < kWheelCount; ++i) {
    char *value_text = strtok_r(nullptr, ",", &save);
    if (!parseFloatStrict(value_text, values[i]) ||
        fabsf(values[i]) > kMaxWheelRadPerSec + 0.0001f) {
      ++rx_error_count;
      sendReply("NACK", sequence, "BAD_SPEED", "CMD");
      return;
    }
  }
  if (strtok_r(nullptr, ",", &save) != nullptr) {
    ++rx_error_count;
    sendReply("NACK", sequence, "FIELD_COUNT", "CMD");
    return;
  }

  bool command_is_zero = true;
  for (float value : values) {
    command_is_zero = command_is_zero && fabsf(value) <= 0.001f;
  }
  if ((fault_bits != kFaultNone || !drive_enabled) && !command_is_zero) {
    sendReply("NACK", sequence, "INHIBITED", "CMD");
    return;
  }

  for (size_t i = 0; i < kWheelCount; ++i) {
    motors[i].requested_rad_s = values[i];
  }
  have_motion_command = true;
  last_command_ms = now_ms;
  last_command_sequence = sequence;
  sendReply("ACK", sequence, "OK", "CMD");
}

void handleSimpleCommand(const char *command,
                         char *save,
                         uint32_t sequence,
                         uint32_t now_ms) {
  if (strtok_r(nullptr, ",", &save) != nullptr) {
    ++rx_error_count;
    sendReply("NACK", sequence, "FIELD_COUNT", command);
    return;
  }

  if (strcmp(command, "PING") == 0) {
    sendReply("ACK", sequence, "OK", "PING");
    return;
  }

  if (strcmp(command, "DISABLE") == 0) {
    drive_enabled = false;
    soft_timeout_active = false;
    hardStopMotorOutputs();
    sendReply("ACK", sequence, "OK", "DISABLE");
    return;
  }

  if (strcmp(command, "ESTOP") == 0) {
    drive_enabled = false;
    soft_timeout_active = false;
    fault_bits |= kFaultSoftwareEstop;
    hardStopMotorOutputs();
    sendReply("ACK", sequence, "OK", "ESTOP");
    return;
  }

  const uint32_t command_age = have_motion_command
                                   ? now_ms - last_command_ms
                                   : UINT32_MAX;
  if (strcmp(command, "CLEAR") == 0) {
    if (command_age > kCommandSoftTimeoutMs ||
        !allRequestedSpeedsAreZero()) {
      sendReply("NACK", sequence, "ZERO_CMD_REQUIRED", "CLEAR");
      return;
    }
    fault_bits = kFaultNone;
    soft_timeout_active = false;
    drive_enabled = false;
    hardStopMotorOutputs();
    sendReply("ACK", sequence, "OK", "CLEAR");
    return;
  }

  if (strcmp(command, "ENABLE") == 0) {
    if (fault_bits != kFaultNone) {
      sendReply("NACK", sequence, "FAULT_ACTIVE", "ENABLE");
      return;
    }
    if (command_age > kCommandSoftTimeoutMs ||
        !allRequestedSpeedsAreZero()) {
      sendReply("NACK", sequence, "ZERO_CMD_REQUIRED", "ENABLE");
      return;
    }
    drive_enabled = true;
    soft_timeout_active = false;
    sendReply("ACK", sequence, "OK", "ENABLE");
    return;
  }

  ++rx_error_count;
  sendReply("NACK", sequence, "UNKNOWN", command);
}

bool decodeFrame(char *line, char *&payload) {
  if (line[0] != '@') {
    return false;
  }
  char *star = strrchr(line, '*');
  if (star == nullptr || strlen(star + 1) != 4) {
    return false;
  }
  char *crc_end = nullptr;
  const unsigned long received_crc = strtoul(star + 1, &crc_end, 16);
  if (crc_end != star + 5 || received_crc > 0xFFFFUL) {
    return false;
  }
  *star = '\0';
  payload = line + 1;
  return crc16Ccitt(payload, strlen(payload)) == received_crc;
}

void processLine(char *line, uint32_t now_ms) {
  char *payload = nullptr;
  if (!decodeFrame(line, payload)) {
    ++rx_error_count;
    return;
  }

  last_valid_frame_ms = now_ms;
  char *save = nullptr;
  char *command = strtok_r(payload, ",", &save);
  if (command == nullptr) {
    ++rx_error_count;
    return;
  }

  uint32_t sequence = 0;
  if (!parseHeader(save, sequence, command)) {
    return;
  }

  if (strcmp(command, "CMD") == 0) {
    handleMotionCommand(save, sequence, now_ms);
  } else {
    handleSimpleCommand(command, save, sequence, now_ms);
  }
}

void readSerial(uint32_t now_ms) {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\n') {
      if (!rx_overflow && rx_length > 0) {
        if (rx_line[rx_length - 1] == '\r') {
          --rx_length;
        }
        rx_line[rx_length] = '\0';
        processLine(rx_line, now_ms);
      } else if (rx_overflow) {
        ++rx_error_count;
      }
      rx_length = 0;
      rx_overflow = false;
    } else if (!rx_overflow) {
      if (rx_length < kMaxRxLineLength) {
        rx_line[rx_length++] = character;
      } else {
        rx_overflow = true;
      }
    }
  }
}

void updateCommandWatchdog(uint32_t now_ms) {
  if (!drive_enabled || !have_motion_command) {
    return;
  }
  const uint32_t age_ms = now_ms - last_command_ms;
  if (age_ms > kCommandSoftTimeoutMs && !soft_timeout_active) {
    soft_timeout_active = true;
    fault_bits |= kFaultCommandTimeout;
    setRequestedSpeedsToZero();
  }
  if (soft_timeout_active && age_ms > kCommandHardTimeoutMs) {
    drive_enabled = false;
    hardStopMotorOutputs();
  }
}

void updateMotorControl(uint32_t now_us) {
  const uint32_t elapsed_us = now_us - last_control_us;
  if (elapsed_us < kControlPeriodUs) {
    return;
  }
  last_control_us = now_us;
  const float elapsed_seconds = elapsed_us * 1.0e-6f;
  const float maximum_ramp_step =
      kMaxAccelerationRpmPerSec * elapsed_seconds;

  for (size_t i = 0; i < kWheelCount; ++i) {
    const int64_t raw_count = encoders[i].getCount();
    motors[i].encoder_count = raw_count * kMotorPins[i].encoder_sign;
    const int64_t delta_count =
        motors[i].encoder_count - motors[i].previous_encoder_count;
    motors[i].previous_encoder_count = motors[i].encoder_count;
    motors[i].raw_rpm =
        static_cast<float>(delta_count) * 60.0f /
        (kEncoderCountsPerRevolution * elapsed_seconds);
    motors[i].measured_rpm =
        kVelocityFilterAlpha * motors[i].raw_rpm +
        (1.0f - kVelocityFilterAlpha) * motors[i].measured_rpm;

    if (!drive_enabled || (fault_bits & kFaultSoftwareEstop) != 0U) {
      motors[i].ramped_rpm = 0.0f;
      motors[i].output_pwm = 0;
      motors[i].pid.reset(motors[i].measured_rpm);
      writeMotorOutput(i, 0);
      continue;
    }

    const float target_rpm = radPerSecToRpm(motors[i].requested_rad_s);
    motors[i].ramped_rpm =
        moveTowards(motors[i].ramped_rpm, target_rpm, maximum_ramp_step);

    if (fabsf(target_rpm) < 0.001f &&
        fabsf(motors[i].ramped_rpm) < 0.001f) {
      motors[i].ramped_rpm = 0.0f;
      motors[i].output_pwm = 0;
      motors[i].pid.reset(motors[i].measured_rpm);
    } else {
      motors[i].output_pwm = motors[i].pid.update(
          motors[i].ramped_rpm, motors[i].measured_rpm, kPidConfig);
    }
    writeMotorOutput(i, motors[i].output_pwm);
  }
}

uint16_t buildStatusBits(uint32_t now_ms) {
  uint16_t status = 0;
  const uint32_t command_age = have_motion_command
                                   ? now_ms - last_command_ms
                                   : UINT32_MAX;
  if (drive_enabled) {
    status |= kStatusEnabled;
  }
  if (command_age <= kCommandSoftTimeoutMs) {
    status |= kStatusCommandFresh;
  }
  if (soft_timeout_active) {
    status |= kStatusSoftStopping;
  }
  if ((fault_bits & kFaultSoftwareEstop) != 0U) {
    status |= kStatusEstopLatched;
  }
  for (const MotorRuntime &motor : motors) {
    if (motor.output_pwm != 0) {
      status |= kStatusMotorOutputActive;
      break;
    }
  }
  return status;
}

void sendTelemetry(uint32_t now_ms) {
  if (now_ms - last_telemetry_ms < kTelemetryPeriodMs) {
    return;
  }
  last_telemetry_ms = now_ms;
  const uint32_t command_age = have_motion_command
                                   ? now_ms - last_command_ms
                                   : UINT32_MAX;
  const double radians_per_count =
      2.0 * static_cast<double>(PI) / kEncoderCountsPerRevolution;
  const uint16_t status = buildStatusBits(now_ms);

  char payload[512];
  snprintf(
      payload,
      sizeof(payload),
      "STATE,%u,%lu,%lu,%lu,%u,%u,%lu,"
      "%.6f,%.6f,%.6f,%.6f,"
      "%.5f,%.5f,%.5f,%.5f,"
      "%.5f,%.5f,%.5f,%.5f,"
      "%d,%d,%d,%d,%lu,%lu",
      kProtocolVersion,
      static_cast<unsigned long>(tx_state_sequence++),
      static_cast<unsigned long>(last_command_sequence),
      static_cast<unsigned long>(now_ms),
      status,
      fault_bits,
      static_cast<unsigned long>(command_age),
      motors[kFrontLeft].encoder_count * radians_per_count,
      motors[kFrontRight].encoder_count * radians_per_count,
      motors[kRearLeft].encoder_count * radians_per_count,
      motors[kRearRight].encoder_count * radians_per_count,
      rpmToRadPerSec(motors[kFrontLeft].measured_rpm),
      rpmToRadPerSec(motors[kFrontRight].measured_rpm),
      rpmToRadPerSec(motors[kRearLeft].measured_rpm),
      rpmToRadPerSec(motors[kRearRight].measured_rpm),
      rpmToRadPerSec(motors[kFrontLeft].ramped_rpm),
      rpmToRadPerSec(motors[kFrontRight].ramped_rpm),
      rpmToRadPerSec(motors[kRearLeft].ramped_rpm),
      rpmToRadPerSec(motors[kRearRight].ramped_rpm),
      motors[kFrontLeft].output_pwm,
      motors[kFrontRight].output_pwm,
      motors[kRearLeft].output_pwm,
      motors[kRearRight].output_pwm,
      static_cast<unsigned long>(rx_error_count),
      static_cast<unsigned long>(now_ms - last_valid_frame_ms));
  sendPayload(payload);
}

void initializeHardware() {
  for (size_t i = 0; i < kWheelCount; ++i) {
    encoders[i].attachHalfQuad(kMotorPins[i].encoder_a,
                               kMotorPins[i].encoder_b);
    encoders[i].clearCount();
    encoders[i].setFilter(1023);
    motors[i].encoder_count = 0;
    motors[i].previous_encoder_count = 0;
    attachPwm(i);
    writeMotorOutput(i, 0);
  }
}

}  // namespace

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(kSerialBaud);
  initializeHardware();
  hardStopMotorOutputs();

  const uint32_t now_ms = millis();
  last_valid_frame_ms = now_ms;
  last_control_us = micros();
  last_telemetry_ms = now_ms;

  char hello[128];
  snprintf(hello,
           sizeof(hello),
           "HELLO,%u,BASE_DRIVE,FL_FR_RL_RR,%.5f,%lu,%lu",
           kProtocolVersion,
           kMaxWheelRadPerSec,
           static_cast<unsigned long>(kCommandSoftTimeoutMs),
           static_cast<unsigned long>(kCommandHardTimeoutMs));
  sendPayload(hello);
}

void loop() {
  const uint32_t now_ms = millis();
  readSerial(now_ms);
  updateCommandWatchdog(now_ms);
  updateMotorControl(micros());
  sendTelemetry(now_ms);
  delay(1);
}

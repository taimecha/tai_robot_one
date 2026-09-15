#include <Arduino.h>
#include <AccelStepper.h>

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Lift ESP32 (separate from the four-wheel ESP32).
// TB6600 STEP/DIR wiring is unchanged.
constexpr uint8_t DIR_PIN = 18;
constexpr uint8_t STEP_PIN = 19;

// Fail-safe normally-closed (NC) limit switches:
//   GPIO -- NC contact -- GND
// INPUT_PULLUP makes an open contact (pressed switch or broken wire) active HIGH.
// Do not connect 5 V or 24 V industrial signals directly to an ESP32 GPIO.
constexpr uint8_t LOWER_LIMIT_PIN = 25;
constexpr uint8_t UPPER_LIMIT_PIN = 26;
constexpr uint8_t LIMIT_ACTIVE_LEVEL = HIGH;
constexpr uint32_t LIMIT_DEBOUNCE_MS = 15;

// Mechanical configuration:
// - 1.8 degree motor: 200 full steps/revolution
// - TB6600 at 1/4 microstep: 800 pulses/revolution
// - T8x8 screw: 8 mm/revolution
constexpr float MOTOR_FULL_STEPS_PER_REV = 200.0F;
constexpr float MICROSTEP_DIVISION = 4.0F;
constexpr float SCREW_LEAD_MM_PER_REV = 8.0F;
constexpr float STEPS_PER_REV = MOTOR_FULL_STEPS_PER_REV * MICROSTEP_DIVISION;
constexpr float STEPS_PER_MM = STEPS_PER_REV / SCREW_LEAD_MM_PER_REV;

// These values match lift_joint in forklift.xacro. ROS-facing values use SI units.
constexpr float LIFT_MIN_M = 0.0F;
constexpr float LIFT_MAX_M = 0.520F;
constexpr float MAX_VELOCITY_M_S = 0.050F;
constexpr float ACCELERATION_M_S2 = 0.050F;
constexpr float HOMING_VELOCITY_M_S = 0.010F;
constexpr uint32_t HOMING_TIMEOUT_MS = 65000;
constexpr uint32_t HOMING_RELEASE_TIMEOUT_MS = 5000;
constexpr float HOMING_RELEASE_MAX_TRAVEL_M = 0.025F;
constexpr float ENDSTOP_POSITION_TOLERANCE_M = 0.010F;

constexpr float MAX_SPEED_STEPS_S = MAX_VELOCITY_M_S * 1000.0F * STEPS_PER_MM;
constexpr float ACCELERATION_STEPS_S2 =
    ACCELERATION_M_S2 * 1000.0F * STEPS_PER_MM;
constexpr float HOMING_SPEED_STEPS_S =
    HOMING_VELOCITY_M_S * 1000.0F * STEPS_PER_MM;

// A valid SET command or HB must arrive faster than this while armed/homing.
constexpr uint32_t COMMAND_WATCHDOG_MS = 300;
constexpr uint32_t STATE_PUBLISH_PERIOD_MS = 50;  // 20 Hz telemetry

// Set true only if positive AccelStepper motion physically lowers the forks.
constexpr bool DIRECTION_INVERTED = false;

constexpr size_t RX_BUFFER_SIZE = 128;
constexpr size_t MAX_TOKENS = 6;

enum class FaultCode : uint8_t {
  NONE,
  COMM_TIMEOUT,
  BOTH_LIMITS_ACTIVE,
  HOMING_TIMEOUT,
  HOMING_RELEASE_FAILED,
  ENDSTOP_POSITION_MISMATCH,
};

enum class HomingStage : uint8_t {
  NONE,
  RELEASE_LOWER,
  RELEASE_UPPER,
  SEEK_LOWER,
};

struct DebouncedInput {
  uint8_t pin;
  uint8_t stableLevel = LOW;
  uint8_t candidateLevel = LOW;
  uint32_t candidateSinceMs = 0;

  explicit DebouncedInput(uint8_t inputPin) : pin(inputPin) {}

  void begin(uint32_t nowMs) {
    pinMode(pin, INPUT_PULLUP);
    stableLevel = digitalRead(pin);
    candidateLevel = stableLevel;
    candidateSinceMs = nowMs;
  }

  void update(uint32_t nowMs) {
    const uint8_t sample = digitalRead(pin);
    if (sample != candidateLevel) {
      candidateLevel = sample;
      candidateSinceMs = nowMs;
      return;
    }
    if (stableLevel != candidateLevel &&
        static_cast<uint32_t>(nowMs - candidateSinceMs) >= LIMIT_DEBOUNCE_MS) {
      stableLevel = candidateLevel;
    }
  }

  bool active() const { return stableLevel == LIMIT_ACTIVE_LEVEL; }
};

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
DebouncedInput lowerLimit{LOWER_LIMIT_PIN};
DebouncedInput upperLimit{UPPER_LIMIT_PIN};

char rxBuffer[RX_BUFFER_SIZE];
size_t rxLength = 0;
bool droppingOversizeLine = false;

FaultCode fault = FaultCode::NONE;
bool homed = false;
bool armed = false;
bool homing = false;
HomingStage homingStage = HomingStage::NONE;

bool sessionActive = false;
uint32_t sessionId = 0;
uint32_t retiredSessionId = 0;
bool sequenceSeen = false;
uint32_t lastSequence = 0;
uint32_t lastCommandMs = 0;
uint32_t homingStartedMs = 0;
uint32_t homingStageStartedMs = 0;
long homingReleaseStartSteps = 0;
uint32_t lastStatePublishMs = 0;

float stepsToMetres(long steps) {
  return static_cast<float>(steps) / (STEPS_PER_MM * 1000.0F);
}

long metresToSteps(float metres) {
  return lroundf(metres * 1000.0F * STEPS_PER_MM);
}

void stopMotionImmediately() {
  const long position = stepper.currentPosition();
  stepper.setCurrentPosition(position);  // Also clears AccelStepper speed.
  stepper.moveTo(position);
}

const char *faultName(FaultCode code) {
  switch (code) {
    case FaultCode::NONE:
      return "NONE";
    case FaultCode::COMM_TIMEOUT:
      return "COMM_TIMEOUT";
    case FaultCode::BOTH_LIMITS_ACTIVE:
      return "BOTH_LIMITS_ACTIVE";
    case FaultCode::HOMING_TIMEOUT:
      return "HOMING_TIMEOUT";
    case FaultCode::HOMING_RELEASE_FAILED:
      return "HOMING_RELEASE_FAILED";
    case FaultCode::ENDSTOP_POSITION_MISMATCH:
      return "ENDSTOP_POSITION_MISMATCH";
  }
  return "UNKNOWN";
}

const char *stateName() {
  if (fault != FaultCode::NONE) {
    return "FAULT";
  }
  if (!sessionActive) {
    return "WAIT_SESSION";
  }
  if (homing) {
    return "HOMING";
  }
  if (!armed) {
    return "DISARMED";
  }
  return stepper.distanceToGo() == 0 ? "ARMED_IDLE" : "MOVING";
}

void publishState() {
  // STATE,session,last_seq,state,fault,homed,armed,position_m,target_m,
  // velocity_m_s,lower_active,upper_active
  Serial.print("STATE,");
  Serial.print(sessionActive ? sessionId : 0);
  Serial.print(',');
  Serial.print(sequenceSeen ? lastSequence : 0);
  Serial.print(',');
  Serial.print(stateName());
  Serial.print(',');
  Serial.print(faultName(fault));
  Serial.print(',');
  Serial.print(homed ? 1 : 0);
  Serial.print(',');
  Serial.print(armed ? 1 : 0);
  Serial.print(',');
  Serial.print(stepsToMetres(stepper.currentPosition()), 5);
  Serial.print(',');
  Serial.print(stepsToMetres(stepper.targetPosition()), 5);
  Serial.print(',');
  Serial.print(stepper.speed() / (STEPS_PER_MM * 1000.0F), 5);
  Serial.print(',');
  Serial.print(lowerLimit.active() ? 1 : 0);
  Serial.print(',');
  Serial.println(upperLimit.active() ? 1 : 0);
}

void sendAck(const char *command, uint32_t sequence) {
  Serial.print("ACK,");
  Serial.print(sessionActive ? sessionId : 0);
  Serial.print(',');
  Serial.print(sequence);
  Serial.print(',');
  Serial.println(command);
}

void sendError(uint32_t requestedSession, uint32_t sequence, const char *reason) {
  Serial.print("ERR,");
  Serial.print(requestedSession);
  Serial.print(',');
  Serial.print(sequence);
  Serial.print(',');
  Serial.println(reason);
}

void latchFault(FaultCode code) {
  stopMotionImmediately();
  armed = false;
  homing = false;
  homingStage = HomingStage::NONE;
  // A geometry/endstop fault makes the step-count reference untrustworthy.
  // COMM_TIMEOUT only stops pulse generation, so its position may remain
  // homed and can be recovered through a new authenticated session.
  if (code != FaultCode::NONE && code != FaultCode::COMM_TIMEOUT) {
    homed = false;
  }
  if (fault == FaultCode::NONE || code == FaultCode::BOTH_LIMITS_ACTIVE) {
    fault = code;
  }
}

void retireSession() {
  if (sessionId != 0) {
    retiredSessionId = sessionId;
  }
  sessionActive = false;
  sessionId = 0;
  sequenceSeen = false;
  lastSequence = 0;
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

bool parseFloat(const char *text, float &value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  char *end = nullptr;
  value = strtof(text, &end);
  return end != text && *end == '\0' && isfinite(value);
}

bool sequenceIsNew(uint32_t sequence) {
  return !sequenceSeen || static_cast<int32_t>(sequence - lastSequence) > 0;
}

bool validateEnvelope(char *tokens[], size_t count, size_t expectedCount,
                      uint32_t &requestedSession, uint32_t &sequence) {
  requestedSession = 0;
  sequence = 0;
  if (count != expectedCount || !parseUint32(tokens[1], requestedSession) ||
      !parseUint32(tokens[2], sequence)) {
    sendError(requestedSession, sequence, "BAD_FORMAT");
    return false;
  }
  if (!sessionActive || requestedSession != sessionId) {
    sendError(requestedSession, sequence, "BAD_SESSION");
    return false;
  }
  if (!sequenceIsNew(sequence)) {
    sendError(requestedSession, sequence, "STALE_SEQUENCE");
    return false;
  }
  sequenceSeen = true;
  lastSequence = sequence;
  return true;
}

void refreshWatchdog(uint32_t nowMs) { lastCommandMs = nowMs; }

void beginSession(uint32_t requestedSession, uint32_t nowMs) {
  stopMotionImmediately();
  armed = false;
  homing = false;
  homingStage = HomingStage::NONE;

  if (sessionActive && requestedSession == sessionId) {
    refreshWatchdog(nowMs);
    sendAck("HELLO", 0);
    return;
  }

  if (requestedSession == 0 || requestedSession == retiredSessionId) {
    sendError(requestedSession, 0, "SESSION_ID_REUSE");
    return;
  }

  if (sessionActive) {
    retiredSessionId = sessionId;
  }
  sessionId = requestedSession;
  sessionActive = true;
  sequenceSeen = false;
  lastSequence = 0;
  refreshWatchdog(nowMs);
  sendAck("HELLO", 0);
}

void completeHoming() {
  stopMotionImmediately();
  stepper.setCurrentPosition(metresToSteps(LIFT_MIN_M));
  stepper.moveTo(stepper.currentPosition());
  homed = true;
  homing = false;
  homingStage = HomingStage::NONE;
  armed = false;  // Require an explicit ARM after homing.
  Serial.println("EVENT,HOME_COMPLETE");
}

void startHoming(uint32_t nowMs) {
  stopMotionImmediately();
  armed = false;
  homed = false;
  homing = true;
  homingStartedMs = nowMs;
  homingStageStartedMs = nowMs;
  if (lowerLimit.active()) {
    // Do not trust an already-active NC input as zero. First move away until
    // the switch closes again, then reverse and detect a fresh edge. A broken
    // wire never clears and therefore faults instead of creating a false zero.
    homingStage = HomingStage::RELEASE_LOWER;
    homingReleaseStartSteps = stepper.currentPosition();
    stepper.setSpeed(HOMING_SPEED_STEPS_S);
    Serial.println("EVENT,HOME_RELEASE_LOWER");
    return;
  }
  if (upperLimit.active()) {
    // Moving down from the upper switch is permitted only as a short verified
    // release. If DIR is wired/configured backwards, the switch will not close
    // again and this phase faults after at most 25 mm / 5 seconds rather than
    // driving into the upper stop for the full homing timeout.
    homingStage = HomingStage::RELEASE_UPPER;
    homingReleaseStartSteps = stepper.currentPosition();
    stepper.setSpeed(-HOMING_SPEED_STEPS_S);
    Serial.println("EVENT,HOME_RELEASE_UPPER");
    return;
  }
  homingStage = HomingStage::SEEK_LOWER;
  stepper.setSpeed(-HOMING_SPEED_STEPS_S);
  Serial.println("EVENT,HOME_STARTED");
}

void handleHello(char *tokens[], size_t count, uint32_t nowMs) {
  uint32_t requestedSession = 0;
  if (count != 2 || !parseUint32(tokens[1], requestedSession)) {
    sendError(requestedSession, 0, "BAD_FORMAT");
    return;
  }
  beginSession(requestedSession, nowMs);
}

void handleAuthenticatedCommand(char *tokens[], size_t count, uint32_t nowMs) {
  uint32_t requestedSession = 0;
  uint32_t sequence = 0;
  const char *command = tokens[0];

  if (strcmp(command, "HB") == 0) {
    if (!validateEnvelope(tokens, count, 3, requestedSession, sequence)) {
      return;
    }
    refreshWatchdog(nowMs);
    sendAck(command, sequence);
    return;
  }

  if (strcmp(command, "STATUS") == 0) {
    if (!validateEnvelope(tokens, count, 3, requestedSession, sequence)) {
      return;
    }
    sendAck(command, sequence);
    publishState();
    return;
  }

  if (strcmp(command, "STOP") == 0) {
    if (!validateEnvelope(tokens, count, 3, requestedSession, sequence)) {
      return;
    }
    stopMotionImmediately();
    armed = false;
    homing = false;
    homingStage = HomingStage::NONE;
    refreshWatchdog(nowMs);
    sendAck(command, sequence);
    return;
  }

  if (strcmp(command, "CLEAR") == 0) {
    if (!validateEnvelope(tokens, count, 3, requestedSession, sequence)) {
      return;
    }
    stopMotionImmediately();
    armed = false;
    homing = false;
    homingStage = HomingStage::NONE;
    if (lowerLimit.active() && upperLimit.active()) {
      sendError(requestedSession, sequence, "LIMITS_STILL_INVALID");
      return;
    }
    fault = FaultCode::NONE;
    refreshWatchdog(nowMs);
    sendAck(command, sequence);
    return;
  }

  if (strcmp(command, "HOME") == 0) {
    if (!validateEnvelope(tokens, count, 3, requestedSession, sequence)) {
      return;
    }
    if (fault != FaultCode::NONE) {
      sendError(requestedSession, sequence, faultName(fault));
      return;
    }
    refreshWatchdog(nowMs);
    startHoming(nowMs);
    sendAck(command, sequence);
    return;
  }

  if (strcmp(command, "ARM") == 0) {
    if (!validateEnvelope(tokens, count, 3, requestedSession, sequence)) {
      return;
    }
    if (fault != FaultCode::NONE) {
      sendError(requestedSession, sequence, faultName(fault));
      return;
    }
    if (!homed) {
      sendError(requestedSession, sequence, "NOT_HOMED");
      return;
    }
    armed = true;
    homing = false;
    homingStage = HomingStage::NONE;
    stepper.moveTo(stepper.currentPosition());
    refreshWatchdog(nowMs);
    sendAck(command, sequence);
    return;
  }

  if (strcmp(command, "SET") == 0) {
    // SET <session_id> <sequence> <position_m>
    if (count != 4) {
      sendError(0, 0, "BAD_FORMAT");
      return;
    }
    float targetMetres = 0.0F;
    if (!parseFloat(tokens[3], targetMetres)) {
      sendError(0, 0, "BAD_POSITION");
      return;
    }
    if (!validateEnvelope(tokens, count, 4, requestedSession, sequence)) {
      return;
    }
    if (fault != FaultCode::NONE) {
      sendError(requestedSession, sequence, faultName(fault));
      return;
    }
    if (!armed || !homed) {
      sendError(requestedSession, sequence, "NOT_ARMED");
      return;
    }
    if (targetMetres < LIFT_MIN_M || targetMetres > LIFT_MAX_M) {
      sendError(requestedSession, sequence, "POSITION_RANGE");
      return;
    }

    const long targetSteps = metresToSteps(targetMetres);
    const long currentSteps = stepper.currentPosition();
    if ((targetSteps < currentSteps && lowerLimit.active()) ||
        (targetSteps > currentSteps && upperLimit.active())) {
      sendError(requestedSession, sequence, "LIMIT_ACTIVE");
      return;
    }

    stepper.moveTo(targetSteps);
    refreshWatchdog(nowMs);
    sendAck(command, sequence);
    return;
  }

  sendError(0, 0, "UNKNOWN_COMMAND");
}

void uppercaseCommand(char *command) {
  while (*command != '\0') {
    *command = static_cast<char>(toupper(static_cast<unsigned char>(*command)));
    ++command;
  }
}

void processLine(char *line, uint32_t nowMs) {
  char *tokens[MAX_TOKENS] = {nullptr};
  size_t count = 0;
  char *save = nullptr;
  char *token = strtok_r(line, " ,\t", &save);
  while (token != nullptr && count < MAX_TOKENS) {
    tokens[count++] = token;
    token = strtok_r(nullptr, " ,\t", &save);
  }
  if (count == 0) {
    return;
  }
  if (token != nullptr) {
    sendError(0, 0, "TOO_MANY_FIELDS");
    return;
  }

  uppercaseCommand(tokens[0]);
  if (strcmp(tokens[0], "HELLO") == 0) {
    handleHello(tokens, count, nowMs);
    return;
  }
  handleAuthenticatedCommand(tokens, count, nowMs);
}

void readSerial(uint32_t nowMs) {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (droppingOversizeLine) {
        droppingOversizeLine = false;
        rxLength = 0;
        sendError(0, 0, "LINE_TOO_LONG");
      } else if (rxLength > 0) {
        rxBuffer[rxLength] = '\0';
        processLine(rxBuffer, nowMs);
        rxLength = 0;
      }
      continue;
    }

    if (droppingOversizeLine) {
      continue;
    }
    if (rxLength + 1 >= RX_BUFFER_SIZE) {
      droppingOversizeLine = true;
      rxLength = 0;
      continue;
    }
    rxBuffer[rxLength++] = c;
  }
}

void handleLimitSwitches(uint32_t nowMs) {
  const bool lowerActive = lowerLimit.active();
  const bool upperActive = upperLimit.active();

  if (lowerActive && upperActive) {
    latchFault(FaultCode::BOTH_LIMITS_ACTIVE);
    return;
  }

  if (homing) {
    if (homingStage == HomingStage::RELEASE_LOWER) {
      // Releasing the lower switch deliberately moves upward.  The upper
      // switch must therefore remain an unconditional hard boundary even
      // during this special homing phase.
      if (upperActive) {
        homed = false;
        latchFault(FaultCode::HOMING_RELEASE_FAILED);
        Serial.println("EVENT,HOMING_RELEASE_FAILED");
        return;
      }
      if (!lowerActive) {
        stopMotionImmediately();
        homingStage = HomingStage::SEEK_LOWER;
        homingStageStartedMs = nowMs;
        stepper.setSpeed(-HOMING_SPEED_STEPS_S);
        Serial.println("EVENT,HOME_STARTED");
      }
      return;
    }
    if (homingStage == HomingStage::RELEASE_UPPER) {
      if (!upperActive) {
        stopMotionImmediately();
        homingStage = HomingStage::SEEK_LOWER;
        homingStageStartedMs = nowMs;
        stepper.setSpeed(-HOMING_SPEED_STEPS_S);
        Serial.println("EVENT,HOME_STARTED");
      }
      return;
    }
    if (homingStage == HomingStage::SEEK_LOWER) {
      if (upperActive) {
        latchFault(FaultCode::HOMING_RELEASE_FAILED);
        Serial.println("EVENT,HOMING_RELEASE_FAILED");
        return;
      }
      if (lowerActive) {
        completeHoming();
      }
    }
    return;
  }

  if (armed) {
    const float measuredPosition = stepsToMetres(stepper.currentPosition());
    // An NC input also opens when its wire breaks.  Treat an active switch at
    // an impossible estimated position as a fault immediately, including
    // while the lift happens to be moving away from that end.
    if ((lowerActive &&
         measuredPosition > LIFT_MIN_M + ENDSTOP_POSITION_TOLERANCE_M) ||
        (upperActive &&
         measuredPosition < LIFT_MAX_M - ENDSTOP_POSITION_TOLERANCE_M)) {
      latchFault(FaultCode::ENDSTOP_POSITION_MISMATCH);
      return;
    }
  }

  const long direction = stepper.distanceToGo();
  if (armed && lowerActive && direction < 0) {
    stopMotionImmediately();
    stepper.setCurrentPosition(metresToSteps(LIFT_MIN_M));
    stepper.moveTo(stepper.currentPosition());
    return;
  }

  if (armed && upperActive && direction > 0) {
    stopMotionImmediately();
    stepper.setCurrentPosition(metresToSteps(LIFT_MAX_M));
    stepper.moveTo(stepper.currentPosition());
  }
}

void checkWatchdog(uint32_t nowMs) {
  if (!sessionActive || (!armed && !homing)) {
    return;
  }
  if (static_cast<uint32_t>(nowMs - lastCommandMs) <= COMMAND_WATCHDOG_MS) {
    return;
  }

  latchFault(FaultCode::COMM_TIMEOUT);
  retireSession();
  Serial.println("EVENT,COMM_TIMEOUT");
}

void runMotion(uint32_t nowMs) {
  if (fault != FaultCode::NONE) {
    return;
  }

  if (homing) {
    if (homingStage == HomingStage::RELEASE_LOWER ||
        homingStage == HomingStage::RELEASE_UPPER) {
      const float releaseTravel = fabsf(stepsToMetres(
          stepper.currentPosition() - homingReleaseStartSteps));
      if (releaseTravel > HOMING_RELEASE_MAX_TRAVEL_M ||
          static_cast<uint32_t>(nowMs - homingStageStartedMs) >
              HOMING_RELEASE_TIMEOUT_MS) {
        homed = false;
        latchFault(FaultCode::HOMING_RELEASE_FAILED);
        Serial.println("EVENT,HOMING_RELEASE_FAILED");
        return;
      }
      stepper.setSpeed(homingStage == HomingStage::RELEASE_LOWER
                           ? HOMING_SPEED_STEPS_S
                           : -HOMING_SPEED_STEPS_S);
      stepper.runSpeed();
      return;
    }
    if (static_cast<uint32_t>(nowMs - homingStartedMs) > HOMING_TIMEOUT_MS) {
      homed = false;
      latchFault(FaultCode::HOMING_TIMEOUT);
      Serial.println("EVENT,HOMING_TIMEOUT");
      return;
    }
    stepper.setSpeed(-HOMING_SPEED_STEPS_S);
    stepper.runSpeed();
    return;
  }

  if (armed) {
    stepper.run();
  }
}

void setup() {
  Serial.begin(115200);

  const uint32_t nowMs = millis();
  lowerLimit.begin(nowMs);
  upperLimit.begin(nowMs);

  stepper.setPinsInverted(DIRECTION_INVERTED, false, false);
  stepper.setMinPulseWidth(5);
  stepper.setMaxSpeed(MAX_SPEED_STEPS_S);
  stepper.setAcceleration(ACCELERATION_STEPS_S2);
  stepper.setCurrentPosition(0);

  // Never infer zero from a static switch level at boot: an open NC wire has the
  // same level. HOME verifies a release-and-retrigger edge before setting zero.
  if (lowerLimit.active() && upperLimit.active()) {
    latchFault(FaultCode::BOTH_LIMITS_ACTIVE);
  }

  Serial.println("BOOT,LIFT_CONTROL,1");
  publishState();
}

void loop() {
  const uint32_t nowMs = millis();
  lowerLimit.update(nowMs);
  upperLimit.update(nowMs);

  readSerial(nowMs);
  handleLimitSwitches(nowMs);
  checkWatchdog(nowMs);
  runMotion(nowMs);

  if (static_cast<uint32_t>(nowMs - lastStatePublishMs) >=
      STATE_PUBLISH_PERIOD_MS) {
    lastStatePublishMs = nowMs;
    publishState();
  }
}

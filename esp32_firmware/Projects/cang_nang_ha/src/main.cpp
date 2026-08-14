#include <AccelStepper.h>

// TB6600: chi dieu khien PUL/STEP va DIR.
// ENA duoc noi co dinh ben ngoai, ESP32 khong dieu khien ENA.
constexpr uint8_t DIR_PIN = 18;
constexpr uint8_t STEP_PIN = 19;

// Cau hinh co khi hien tai:
// - Dong co 1.8 do: 200 full-step/vong
// - TB6600 dat 1/4 microstep: 800 xung/vong
// - Vit me T8x8: di chuyen 8 mm/vong
constexpr float MOTOR_FULL_STEPS_PER_REV = 200.0F;
constexpr float MICROSTEP_DIVISION = 4.0F;
constexpr float SCREW_LEAD_MM_PER_REV = 8.0F;
constexpr float STEPS_PER_REV = MOTOR_FULL_STEPS_PER_REV * MICROSTEP_DIVISION;
constexpr float STEPS_PER_MM = STEPS_PER_REV / SCREW_LEAD_MM_PER_REV;

// Dong bo voi lift_joint trong forklift.xacro:
// lower=0.0 m, upper=0.520 m, velocity=0.05 m/s.
constexpr float LIFT_MIN_MM = 0.0F;
constexpr float LIFT_MAX_MM = 520.0F;
constexpr float MAX_VELOCITY_MM_S = 50.0F;
constexpr float ACCELERATION_MM_S2 = 50.0F;

constexpr float MAX_SPEED_STEPS_S = MAX_VELOCITY_MM_S * STEPS_PER_MM;
constexpr float ACCELERATION_STEPS_S2 = ACCELERATION_MM_S2 * STEPS_PER_MM;

// Doi thanh true neu lenh duong lam cang di xuong thay vi di len.
constexpr bool DIRECTION_INVERTED = false;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

String serialBuffer;
bool zeroIsSet = false;
bool moveWasRunning = false;

float stepsToMm(long steps) {
  return static_cast<float>(steps) / STEPS_PER_MM;
}

long mmToSteps(float millimetres) {
  return lroundf(millimetres * STEPS_PER_MM);
}

void printStatus() {
  Serial.print("Vi tri uoc tinh: ");
  Serial.print(stepsToMm(stepper.currentPosition()), 2);
  Serial.print(" mm; dich: ");
  Serial.print(stepsToMm(stepper.targetPosition()), 2);
  Serial.print(" mm; zero: ");
  Serial.println(zeroIsSet ? "DA DAT" : "CHUA DAT");
}

void printHelp() {
  Serial.println("----------------------------------------------");
  Serial.println("Dieu khien nang/ha TB6600 (khong encoder/endstop)");
  Serial.println("1) Dua cang ve vi tri thap nhat bang tay, sau do gui: zero");
  Serial.println("2) goto <mm> : den vi tri tuyet doi 0..520 mm");
  Serial.println("3) move <mm> : di chuyen tuong doi, co the am");
  Serial.println("4) stop      : giam toc va dung");
  Serial.println("5) status    : xem vi tri uoc tinh");
  Serial.println("Vi du: goto 312");
  Serial.println("----------------------------------------------");
}

void moveToMm(float targetMm) {
  if (!zeroIsSet) {
    Serial.println("TU CHOI: Hay dua cang ve day va gui lenh 'zero' truoc.");
    return;
  }

  if (targetMm < LIFT_MIN_MM || targetMm > LIFT_MAX_MM) {
    Serial.println("TU CHOI: Vi tri phai nam trong 0..520 mm (gioi han URDF).");
    return;
  }

  stepper.moveTo(mmToSteps(targetMm));
  moveWasRunning = true;
  Serial.print("Dang di den ");
  Serial.print(targetMm, 2);
  Serial.println(" mm");
}

bool parseNumber(const String &text, float &value) {
  const char *start = text.c_str();
  char *end = nullptr;
  value = strtof(start, &end);
  while (end != nullptr && *end == ' ') {
    ++end;
  }
  return end != start && end != nullptr && *end == '\0' && isfinite(value);
}

void handleCommand(String input) {
  input.trim();
  input.toLowerCase();

  if (input.length() == 0) {
    return;
  }

  if (input == "help") {
    printHelp();
    return;
  }

  if (input == "status") {
    printStatus();
    return;
  }

  if (input == "stop") {
    stepper.stop();
    Serial.println("Dang giam toc de dung...");
    return;
  }

  if (input == "zero") {
    if (stepper.isRunning()) {
      Serial.println("TU CHOI: Phai dung dong co truoc khi dat zero.");
      return;
    }
    stepper.setCurrentPosition(0);
    zeroIsSet = true;
    moveWasRunning = false;
    Serial.println("Da dat vi tri hien tai = 0 mm.");
    return;
  }

  float value = 0.0F;
  if (input.startsWith("goto ") && parseNumber(input.substring(5), value)) {
    moveToMm(value);
    return;
  }

  if (input.startsWith("move ") && parseNumber(input.substring(5), value)) {
    if (!zeroIsSet) {
      Serial.println("TU CHOI: Hay gui lenh 'zero' truoc.");
      return;
    }
    moveToMm(stepsToMm(stepper.currentPosition()) + value);
    return;
  }

  Serial.println("Lenh khong hop le. Gui 'help' de xem huong dan.");
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handleCommand(serialBuffer);
        serialBuffer = "";
      }
    } else if (serialBuffer.length() < 80) {
      serialBuffer += c;
    }
  }
}

void setup() {
  Serial.begin(115200);

  stepper.setPinsInverted(DIRECTION_INVERTED, false, false);
  stepper.setMinPulseWidth(5);
  stepper.setMaxSpeed(MAX_SPEED_STEPS_S);
  stepper.setAcceleration(ACCELERATION_STEPS_S2);
  stepper.setCurrentPosition(0);

  printHelp();
  Serial.print("Quy doi: ");
  Serial.print(STEPS_PER_MM, 2);
  Serial.print(" xung/mm; max ");
  Serial.print(MAX_SPEED_STEPS_S, 0);
  Serial.println(" xung/s");
}

void loop() {
  readSerialCommands();
  stepper.run();

  if (moveWasRunning && !stepper.isRunning()) {
    moveWasRunning = false;
    Serial.println("Da den vi tri dich (uoc tinh theo so xung da phat).");
    printStatus();
  }
}

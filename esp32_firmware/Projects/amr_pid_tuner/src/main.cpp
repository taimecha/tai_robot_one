#include <Arduino.h>
#include <ESP32Encoder.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_arduino_version.h>
#include <math.h>

#include "pid.hpp"

// ========================================================
// 1. CẤU HÌNH WI-FI ACCESS POINT
// ========================================================
const char *WIFI_NAME = "AMR_Forklift_40kg";
const char *WIFI_PASSWORD = "12345678";

// Địa chỉ truy cập giao diện:
// http://192.168.4.1
IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// ========================================================
// 2. THÔNG SỐ CỐ ĐỊNH
// ========================================================
const int PWM_FREQ = 5000;
const int PWM_RES = 8;

const float PULSES_PER_REV = 1200.0f;
const int PID_INTERVAL_MS = 20;

const float WHEEL_RADIUS_M = 0.0875f;
const float MAX_VEHICLE_SPEED_MPS = 0.5f;
const unsigned long COMMAND_WATCHDOG_MS = 500;

// 0.5 m/s với bán kính bánh 87.5 mm = khoảng 54.57 RPM.
const float MAX_COMMAND_RPM =
  MAX_VEHICLE_SPEED_MPS /
  WHEEL_RADIUS_M *
  60.0f /
  (2.0f * PI);

// ========================================================
// 3. THÔNG SỐ CÓ THỂ THAY ĐỔI TỪ GIAO DIỆN
// ========================================================
struct ControlParameters {
  float maxAccelRPMPerSec;
  float lpfAlpha;

  float Kf;
  int deadbandPWM;

  float Kp;
  float Ki;
  float Kd;
};

ControlParameters sharedParams = {
  320.0f,  // MAX_ACCEL_RPM_PER_SEC
  0.2f,    // LPF_ALPHA
  1.47f,   // Kf
  3,       // deadbandPWM
  0.3f,    // Kp
  0.02f,   // Ki
  0.01f    // Kd
};

// RPM đặt theo thứ tự:
// 0 = FR, 1 = FL, 2 = BL, 3 = BR
float sharedTargets[4] = {
  0.0f,
  0.0f,
  0.0f,
  0.0f
};

bool sharedEmergencyStop = false;
bool sharedCommandLeaseActive = false;
bool sharedWatchdogTimedOut = false;
unsigned long sharedLastCommandMs = 0;

// Khóa dữ liệu dùng chung Core 0 và Core 1
portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;

// ========================================================
// 4. CẤU HÌNH ĐỘNG CƠ
// ========================================================
struct MotorController {
  MotorController(
    const char *motorName,
    int motorRPWMPin,
    int motorLPWMPin,
    int motorEncoderAPin,
    int motorEncoderBPin
  ) :
    name(motorName),
    rpwmPin(motorRPWMPin),
    lpwmPin(motorLPWMPin),
    encoderAPin(motorEncoderAPin),
    encoderBPin(motorEncoderBPin) {}

  String name;

  int rpwmPin;
  int lpwmPin;

  int encoderAPin;
  int encoderBPin;

  ESP32Encoder encoder;
  amr_drive::VelocityPid pid;

  // Dữ liệu nội bộ Core 1
  float rampedTargetRPM = 0.0f;

  float rawRPM = 0.0f;
  float currentRPM = 0.0f;

  int outputPWM = 0;

  long previousEncoderCount = 0;

  // Phân tích đáp ứng
  bool isAnalyzing = false;
  bool reportValid = false;

  float oldTargetRPM = 0.0f;
  float peakRPM = 0.0f;

  float reportTargetRPM = 0.0f;
  float reportSettlingTime = 0.0f;
  float reportOvershootPct = 0.0f;

  unsigned long stepStartTime = 0;
  unsigned long settledStartTime = 0;
};

MotorController motors[4] = {
  { "FR", 32, 33, 23, 4 },   // Lệnh web FR
  { "FL", 27, 14, 18, 19 },  // Lệnh web FL
  { "BL", 22, 13, 16, 17 },  // Lệnh web BL (rear-left)
  { "BR", 25, 26, 34, 35 }   // Lệnh web BR (rear-right)
};

void attachMotorPWM(int motorIndex) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(
    motors[motorIndex].rpwmPin,
    PWM_FREQ,
    PWM_RES
  );
  ledcAttach(
    motors[motorIndex].lpwmPin,
    PWM_FREQ,
    PWM_RES
  );
#else
  const int rpwmChannel = motorIndex * 2;
  const int lpwmChannel = rpwmChannel + 1;
  ledcSetup(rpwmChannel, PWM_FREQ, PWM_RES);
  ledcSetup(lpwmChannel, PWM_FREQ, PWM_RES);
  ledcAttachPin(motors[motorIndex].rpwmPin, rpwmChannel);
  ledcAttachPin(motors[motorIndex].lpwmPin, lpwmChannel);
#endif
}

void writeMotorPWMPin(
  int motorIndex,
  bool useRPWM,
  int duty
) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(
    useRPWM
      ? motors[motorIndex].rpwmPin
      : motors[motorIndex].lpwmPin,
    duty
  );
#else
  ledcWrite(
    motorIndex * 2 + (useRPWM ? 0 : 1),
    duty
  );
#endif
}

// ========================================================
// 5. DỮ LIỆU GIÁM SÁT GỬI LÊN WEB
// ========================================================
struct TelemetryData {
  float currentRPM[4];
  int outputPWM[4];

  bool reportValid[4];
  float reportTargetRPM[4];
  float settlingTime[4];
  float overshootPct[4];
};

TelemetryData telemetry;

portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;

// ========================================================
// 6. GIAO DIỆN WEB
// Không dùng thư viện biểu đồ ngoài vì ESP32 phát Wi-Fi
// độc lập và có thể không có Internet.
// ========================================================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta
    name="viewport"
    content="width=device-width, initial-scale=1.0"
  >

  <title>AMR Forklift PID Control</title>

  <style>
    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      padding: 16px;
      background: #111827;
      color: #e5e7eb;
      font-family: Arial, Helvetica, sans-serif;
    }

    h1 {
      margin-top: 0;
      font-size: 25px;
      text-align: center;
    }

    h2 {
      margin-top: 0;
      font-size: 19px;
    }

    .container {
      max-width: 1250px;
      margin: auto;
    }

    .status-bar {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-bottom: 14px;
    }

    .badge {
      padding: 8px 13px;
      border-radius: 8px;
      background: #374151;
      font-weight: bold;
    }

    .connected {
      background: #065f46;
    }

    .disconnected {
      background: #991b1b;
    }

    .estop-on {
      background: #b91c1c;
    }

    .estop-off {
      background: #166534;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(
        auto-fit,
        minmax(330px, 1fr)
      );
      gap: 14px;
    }

    .card {
      background: #1f2937;
      border-radius: 12px;
      padding: 16px;
      box-shadow: 0 3px 10px rgba(0, 0, 0, 0.35);
    }

    .parameter-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }

    label {
      display: block;
      margin-bottom: 4px;
      color: #d1d5db;
      font-size: 14px;
    }

    input {
      width: 100%;
      padding: 10px;
      border: 1px solid #4b5563;
      border-radius: 7px;
      background: #111827;
      color: white;
      font-size: 16px;
    }

    button {
      padding: 11px 15px;
      border: none;
      border-radius: 8px;
      cursor: pointer;
      color: white;
      background: #2563eb;
      font-weight: bold;
      font-size: 14px;
    }

    button:hover {
      opacity: 0.88;
    }

    .button-row {
      display: flex;
      flex-wrap: wrap;
      gap: 9px;
      margin-top: 13px;
    }

    .green {
      background: #15803d;
    }

    .orange {
      background: #c2410c;
    }

    .red {
      background: #dc2626;
    }

    .gray {
      background: #4b5563;
    }

    .message {
      margin-top: 12px;
      min-height: 22px;
      font-weight: bold;
    }

    .ok {
      color: #4ade80;
    }

    .error {
      color: #f87171;
    }

    table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 10px;
    }

    th,
    td {
      border-bottom: 1px solid #374151;
      padding: 9px 6px;
      text-align: center;
      font-size: 14px;
    }

    th {
      color: #93c5fd;
    }

    .chart-card {
      margin-top: 14px;
    }

    canvas {
      width: 100%;
      height: 390px;
      display: block;
      background: #0b1220;
      border: 1px solid #374151;
      border-radius: 8px;
    }

    .legend {
      display: flex;
      flex-wrap: wrap;
      gap: 9px 15px;
      margin-top: 10px;
      font-size: 13px;
    }

    .legend-item {
      display: flex;
      align-items: center;
      gap: 6px;
    }

    .legend-color {
      width: 20px;
      height: 4px;
      border-radius: 2px;
    }

    .warning {
      color: #fbbf24;
      font-size: 14px;
      line-height: 1.5;
    }

    @media (max-width: 650px) {
      body {
        padding: 8px;
      }

      .parameter-grid {
        grid-template-columns: 1fr;
      }

      canvas {
        height: 300px;
      }
    }
  </style>
</head>

<body>
<div class="container">

  <h1>AMR Forklift – Điều khiển PID không dây</h1>

  <div class="status-bar">
    <div id="connectionBadge" class="badge disconnected">
      Mất kết nối
    </div>

    <div id="estopBadge" class="badge estop-off">
      ESTOP: OFF
    </div>

    <div id="clientBadge" class="badge">
      Thiết bị kết nối: 0
    </div>

    <div id="uptimeBadge" class="badge">
      Thời gian chạy: 0 s
    </div>
  </div>

  <div class="grid">

    <!-- THÔNG SỐ PID -->
    <div class="card">
      <h2>Thông số điều khiển</h2>

      <div class="parameter-grid">

        <div>
          <label for="accel">
            MAX_ACCEL_RPM_PER_SEC
          </label>
          <input
            id="accel"
            type="number"
            step="1"
            value="250"
          >
        </div>

        <div>
          <label for="alpha">LPF_ALPHA</label>
          <input
            id="alpha"
            type="number"
            step="0.01"
            value="0.4"
          >
        </div>

        <div>
          <label for="kf">Kf</label>
          <input
            id="kf"
            type="number"
            step="0.01"
            value="1.47"
          >
        </div>

        <div>
          <label for="deadband">deadbandPWM</label>
          <input
            id="deadband"
            type="number"
            step="1"
            value="3"
          >
        </div>

        <div>
          <label for="kp">Kp</label>
          <input
            id="kp"
            type="number"
            step="0.01"
            value="0.3"
          >
        </div>

        <div>
          <label for="ki">Ki</label>
          <input
            id="ki"
            type="number"
            step="0.001"
            value="0.02"
          >
        </div>

        <div>
          <label for="kd">Kd</label>
          <input
            id="kd"
            type="number"
            step="0.001"
            value="0.01"
          >
        </div>

      </div>

      <div class="button-row">
        <button onclick="saveParameters()">
          Cập nhật thông số
        </button>

        <button class="gray" onclick="loadParameters()">
          Đọc lại
        </button>
      </div>

      <div id="parameterMessage" class="message"></div>
    </div>

    <!-- ĐIỀU KHIỂN RPM -->
    <div class="card">
      <h2>Điều khiển tốc độ</h2>

      <label for="commonRPM">
        RPM chung cho 4 bánh
      </label>

      <input
        id="commonRPM"
        type="number"
        step="1"
        value="20"
      >

      <div class="button-row">
        <button class="green" onclick="setCommonRPM()">
          Chạy 4 bánh
        </button>

        <button class="orange" onclick="softStop()">
          Dừng mềm
        </button>

        <button class="red" onclick="emergencyStop()">
          DỪNG KHẨN
        </button>

        <button class="gray" onclick="releaseEmergencyStop()">
          Mở khóa ESTOP
        </button>
      </div>

      <hr style="border-color:#374151; margin:18px 0;">

      <div class="parameter-grid">

        <div>
          <label for="rpmFR">FR – Trước phải</label>
          <input id="rpmFR" type="number" step="1" value="0">
        </div>

        <div>
          <label for="rpmFL">FL – Trước trái</label>
          <input id="rpmFL" type="number" step="1" value="0">
        </div>

        <div>
          <label for="rpmBL">BL – Sau trái</label>
          <input id="rpmBL" type="number" step="1" value="0">
        </div>

        <div>
          <label for="rpmBR">BR – Sau phải</label>
          <input id="rpmBR" type="number" step="1" value="0">
        </div>

      </div>

      <div class="button-row">
        <button onclick="setFourRPM()">
          Gửi RPM riêng
        </button>
      </div>

      <div id="rpmMessage" class="message"></div>

      <p class="warning">
        Giới hạn hiện tại: ±54,57 RPM (tương ứng 0,5 m/s).
        Khi thử xe dưới sàn,
        nên bắt đầu khoảng 10–20 RPM và đặt gia tốc thấp.
      </p>
    </div>

  </div>

  <!-- BẢNG DỮ LIỆU -->
  <div class="card" style="margin-top:14px;">
    <h2>Dữ liệu 4 bánh xe</h2>

    <table>
      <thead>
        <tr>
          <th>Bánh</th>
          <th>RPM đặt</th>
          <th>RPM thực</th>
          <th>PWM</th>
          <th>Xác lập</th>
          <th>Vọt lố</th>
        </tr>
      </thead>

      <tbody>
        <tr>
          <td>FR</td>
          <td id="targetFR">0</td>
          <td id="actualFR">0</td>
          <td id="pwmFR">0</td>
          <td id="settlingFR">--</td>
          <td id="overshootFR">--</td>
        </tr>

        <tr>
          <td>FL</td>
          <td id="targetFL">0</td>
          <td id="actualFL">0</td>
          <td id="pwmFL">0</td>
          <td id="settlingFL">--</td>
          <td id="overshootFL">--</td>
        </tr>

        <tr>
          <td>BL</td>
          <td id="targetBL">0</td>
          <td id="actualBL">0</td>
          <td id="pwmBL">0</td>
          <td id="settlingBL">--</td>
          <td id="overshootBL">--</td>
        </tr>

        <tr>
          <td>BR</td>
          <td id="targetBR">0</td>
          <td id="actualBR">0</td>
          <td id="pwmBR">0</td>
          <td id="settlingBR">--</td>
          <td id="overshootBR">--</td>
        </tr>
      </tbody>
    </table>
  </div>

  <!-- ĐỒ THỊ -->
  <div class="card chart-card">
    <h2>Đồ thị RPM thời gian thực</h2>

    <canvas id="rpmChart"></canvas>

    <div class="legend">
      <div class="legend-item">
        <span class="legend-color" style="background:#ffffff;"></span>
        Target FR
      </div>

      <div class="legend-item">
        <span class="legend-color" style="background:#fbbf24;"></span>
        RPM FR
      </div>

      <div class="legend-item">
        <span class="legend-color" style="background:#d1d5db;"></span>
        Target FL
      </div>

      <div class="legend-item">
        <span class="legend-color" style="background:#22c55e;"></span>
        RPM FL
      </div>

      <div class="legend-item">
        <span class="legend-color" style="background:#93c5fd;"></span>
        Target BL
      </div>

      <div class="legend-item">
        <span class="legend-color" style="background:#3b82f6;"></span>
        RPM BL
      </div>

      <div class="legend-item">
        <span class="legend-color" style="background:#f9a8d4;"></span>
        Target BR
      </div>

      <div class="legend-item">
        <span class="legend-color" style="background:#ef4444;"></span>
        RPM BR
      </div>
    </div>

    <div class="button-row">
      <button class="gray" onclick="clearChart()">
        Xóa đồ thị
      </button>
    </div>
  </div>

</div>

<script>
  const wheelNames = ["FR", "FL", "BL", "BR"];

  let parametersLoaded = false;
  let historyData = [];
  let activeRPMCommand = null;
  let commandRefreshInFlight = false;

  const maxHistoryPoints = 200;

  const chartCanvas =
    document.getElementById("rpmChart");

  const chartContext =
    chartCanvas.getContext("2d");

  function setMessage(elementId, text, isError = false) {
    const element = document.getElementById(elementId);

    element.textContent = text;
    element.className =
      "message " + (isError ? "error" : "ok");
  }

  async function sendForm(url, values) {
    const body = new URLSearchParams(values);

    const response = await fetch(url, {
      method: "POST",
      headers: {
        "Content-Type":
          "application/x-www-form-urlencoded"
      },
      body: body.toString()
    });

    const text = await response.text();

    if (!response.ok) {
      throw new Error(text);
    }

    return text;
  }

  async function saveParameters() {
    try {
      const result = await sendForm(
        "/api/params",
        {
          accel:
            document.getElementById("accel").value,

          alpha:
            document.getElementById("alpha").value,

          kf:
            document.getElementById("kf").value,

          deadband:
            document.getElementById("deadband").value,

          kp:
            document.getElementById("kp").value,

          ki:
            document.getElementById("ki").value,

          kd:
            document.getElementById("kd").value
        }
      );

      setMessage(
        "parameterMessage",
        result,
        false
      );

      parametersLoaded = false;
    }
    catch (error) {
      setMessage(
        "parameterMessage",
        error.message,
        true
      );
    }
  }

  async function loadParameters() {
    parametersLoaded = false;

    await updateStatus();

    setMessage(
      "parameterMessage",
      "Đã đọc thông số từ ESP32.",
      false
    );
  }

  async function setCommonRPM() {
    const commonRPM =
      document.getElementById("commonRPM").value;

    const values = {
      fr: commonRPM,
      fl: commonRPM,
      bl: commonRPM,
      br: commonRPM
    };

    try {
      const result = await sendForm(
        "/api/rpm",
        values
      );

      activeRPMCommand = values;
      setMessage("rpmMessage", result, false);
    }
    catch (error) {
      setMessage(
        "rpmMessage",
        error.message,
        true
      );
    }
  }

  async function setFourRPM() {
    const values = {
      fr: document.getElementById("rpmFR").value,
      fl: document.getElementById("rpmFL").value,
      bl: document.getElementById("rpmBL").value,
      br: document.getElementById("rpmBR").value
    };

    try {
      const result = await sendForm(
        "/api/rpm",
        values
      );

      activeRPMCommand = values;
      setMessage("rpmMessage", result, false);
    }
    catch (error) {
      setMessage(
        "rpmMessage",
        error.message,
        true
      );
    }
  }

  async function softStop() {
    activeRPMCommand = null;

    try {
      const response = await fetch(
        "/api/stop",
        { method: "POST" }
      );

      const text = await response.text();

      if (!response.ok) {
        throw new Error(text);
      }

      setMessage("rpmMessage", text, false);
    }
    catch (error) {
      setMessage(
        "rpmMessage",
        error.message,
        true
      );
    }
  }

  async function emergencyStop() {
    activeRPMCommand = null;

    try {
      const response = await fetch(
        "/api/estop",
        { method: "POST" }
      );

      const text = await response.text();

      if (!response.ok) {
        throw new Error(text);
      }

      setMessage("rpmMessage", text, true);
    }
    catch (error) {
      setMessage(
        "rpmMessage",
        error.message,
        true
      );
    }
  }

  async function releaseEmergencyStop() {
    try {
      const response = await fetch(
        "/api/run",
        { method: "POST" }
      );

      const text = await response.text();

      if (!response.ok) {
        throw new Error(text);
      }

      setMessage("rpmMessage", text, false);
    }
    catch (error) {
      setMessage(
        "rpmMessage",
        error.message,
        true
      );
    }
  }

  async function updateStatus() {
    try {
      const response = await fetch(
        "/api/status",
        { cache: "no-store" }
      );

      if (!response.ok) {
        throw new Error("Không đọc được dữ liệu.");
      }

      const data = await response.json();

      const connectionBadge =
        document.getElementById("connectionBadge");

      connectionBadge.textContent =
        "Đã kết nối ESP32";

      connectionBadge.className =
        "badge connected";

      const estopBadge =
        document.getElementById("estopBadge");

      if (data.estop) {
        estopBadge.textContent = "ESTOP: ON";
        estopBadge.className =
          "badge estop-on";
      }
      else {
        estopBadge.textContent = "ESTOP: OFF";
        estopBadge.className =
          "badge estop-off";
      }

      document.getElementById("clientBadge")
        .textContent =
        "Thiết bị kết nối: " + data.clients;

      document.getElementById("uptimeBadge")
        .textContent =
        "Thời gian chạy: " +
        data.uptime +
        " s";

      if (!parametersLoaded) {
        document.getElementById("accel").value =
          data.params.accel;

        document.getElementById("alpha").value =
          data.params.alpha;

        document.getElementById("kf").value =
          data.params.kf;

        document.getElementById("deadband").value =
          data.params.deadband;

        document.getElementById("kp").value =
          data.params.kp;

        document.getElementById("ki").value =
          data.params.ki;

        document.getElementById("kd").value =
          data.params.kd;

        parametersLoaded = true;
      }

      for (let i = 0; i < 4; i++) {
        const wheel = wheelNames[i];

        document.getElementById(
          "target" + wheel
        ).textContent =
          data.target[i].toFixed(1);

        document.getElementById(
          "actual" + wheel
        ).textContent =
          data.rpm[i].toFixed(1);

        document.getElementById(
          "pwm" + wheel
        ).textContent =
          data.pwm[i];

        if (data.reportValid[i]) {
          document.getElementById(
            "settling" + wheel
          ).textContent =
            data.settling[i].toFixed(2) + " s";

          document.getElementById(
            "overshoot" + wheel
          ).textContent =
            data.overshoot[i].toFixed(1) + " %";
        }
        else {
          document.getElementById(
            "settling" + wheel
          ).textContent = "--";

          document.getElementById(
            "overshoot" + wheel
          ).textContent = "--";
        }
      }

      historyData.push({
        target: [
          data.target[0],
          data.target[1],
          data.target[2],
          data.target[3]
        ],

        rpm: [
          data.rpm[0],
          data.rpm[1],
          data.rpm[2],
          data.rpm[3]
        ]
      });

      if (historyData.length > maxHistoryPoints) {
        historyData.shift();
      }

      drawChart();
    }
    catch (error) {
      const connectionBadge =
        document.getElementById("connectionBadge");

      connectionBadge.textContent =
        "Mất kết nối ESP32";

      connectionBadge.className =
        "badge disconnected";
    }
  }

  function clearChart() {
    historyData = [];
    drawChart();
  }

  function resizeCanvas() {
    const pixelRatio =
      window.devicePixelRatio || 1;

    const displayWidth =
      chartCanvas.clientWidth;

    const displayHeight =
      chartCanvas.clientHeight;

    chartCanvas.width =
      displayWidth * pixelRatio;

    chartCanvas.height =
      displayHeight * pixelRatio;

    chartContext.setTransform(
      pixelRatio,
      0,
      0,
      pixelRatio,
      0,
      0
    );
  }

  function drawChart() {
    resizeCanvas();

    const width =
      chartCanvas.clientWidth;

    const height =
      chartCanvas.clientHeight;

    const left = 48;
    const right = 15;
    const top = 18;
    const bottom = 28;

    const plotWidth =
      width - left - right;

    const plotHeight =
      height - top - bottom;

    chartContext.clearRect(
      0,
      0,
      width,
      height
    );

    chartContext.fillStyle = "#0b1220";

    chartContext.fillRect(
      0,
      0,
      width,
      height
    );

    let maximumAbsolute = 20;

    for (const point of historyData) {
      for (let i = 0; i < 4; i++) {
        maximumAbsolute = Math.max(
          maximumAbsolute,
          Math.abs(point.target[i]),
          Math.abs(point.rpm[i])
        );
      }
    }

    maximumAbsolute =
      Math.ceil(maximumAbsolute / 10) * 10;

    const yMin = -maximumAbsolute;
    const yMax = maximumAbsolute;

    function mapY(value) {
      return top +
        (yMax - value) /
        (yMax - yMin) *
        plotHeight;
    }

    // Lưới ngang
    chartContext.strokeStyle = "#273449";
    chartContext.lineWidth = 1;

    chartContext.fillStyle = "#9ca3af";
    chartContext.font = "12px Arial";

    const horizontalLines = 8;

    for (
      let i = 0;
      i <= horizontalLines;
      i++
    ) {
      const y =
        top +
        i / horizontalLines *
        plotHeight;

      const value =
        yMax -
        i / horizontalLines *
        (yMax - yMin);

      chartContext.beginPath();
      chartContext.moveTo(left, y);
      chartContext.lineTo(
        left + plotWidth,
        y
      );
      chartContext.stroke();

      chartContext.fillText(
        value.toFixed(0),
        4,
        y + 4
      );
    }

    // Trục 0 RPM
    chartContext.strokeStyle = "#64748b";
    chartContext.lineWidth = 1.5;

    chartContext.beginPath();
    chartContext.moveTo(left, mapY(0));
    chartContext.lineTo(
      left + plotWidth,
      mapY(0)
    );
    chartContext.stroke();

    if (historyData.length < 2) {
      return;
    }

    function drawLine(
      sourceName,
      wheelIndex,
      lineColor,
      lineWidth
    ) {
      chartContext.strokeStyle = lineColor;
      chartContext.lineWidth = lineWidth;
      chartContext.beginPath();

      for (
        let index = 0;
        index < historyData.length;
        index++
      ) {
        const x =
          left +
          index /
          (maxHistoryPoints - 1) *
          plotWidth;

        const value =
          historyData[index][sourceName][wheelIndex];

        const y = mapY(value);

        if (index === 0) {
          chartContext.moveTo(x, y);
        }
        else {
          chartContext.lineTo(x, y);
        }
      }

      chartContext.stroke();
    }

    // Target nét mảnh
    drawLine("target", 0, "#ffffff", 1);
    drawLine("target", 1, "#d1d5db", 1);
    drawLine("target", 2, "#93c5fd", 1);
    drawLine("target", 3, "#f9a8d4", 1);

    // RPM thực nét đậm
    drawLine("rpm", 0, "#fbbf24", 2);
    drawLine("rpm", 1, "#22c55e", 2);
    drawLine("rpm", 2, "#3b82f6", 2);
    drawLine("rpm", 3, "#ef4444", 2);
  }

  window.addEventListener(
    "resize",
    drawChart
  );

  async function refreshCommandLease() {
    if (
      activeRPMCommand === null ||
      commandRefreshInFlight ||
      document.hidden
    ) {
      return;
    }

    commandRefreshInFlight = true;

    try {
      await sendForm("/api/rpm", activeRPMCommand);
    }
    catch (error) {
      activeRPMCommand = null;
      setMessage("rpmMessage", "Watchdog: " + error.message, true);
    }
    finally {
      commandRefreshInFlight = false;
    }
  }

  document.addEventListener("visibilitychange", () => {
    if (document.hidden && activeRPMCommand !== null) {
      activeRPMCommand = null;
      fetch("/api/stop", { method: "POST", keepalive: true });
    }
  });

  window.addEventListener("beforeunload", () => {
    if (activeRPMCommand !== null) {
      navigator.sendBeacon("/api/stop");
    }
  });

  updateStatus();

  setInterval(
    updateStatus,
    100
  );

  setInterval(
    refreshCommandLease,
    200
  );
</script>
</body>
</html>
)HTML";

// ========================================================
// 7. ĐỌC DỮ LIỆU ĐIỀU KHIỂN DÙNG CHUNG
// ========================================================
void getControlSnapshot(
  ControlParameters &parameters,
  float targets[4],
  bool &emergencyStop
) {
  portENTER_CRITICAL(&controlMux);

  parameters = sharedParams;
  emergencyStop = sharedEmergencyStop;

  for (int i = 0; i < 4; i++) {
    targets[i] = sharedTargets[i];
  }

  portEXIT_CRITICAL(&controlMux);
}

// ========================================================
// 8. CẬP NHẬT RPM ĐẶT
// ========================================================
void setTargetRPM(
  float fr,
  float fl,
  float bl,
  float br
) {
  portENTER_CRITICAL(&controlMux);

  sharedTargets[0] = fr;
  sharedTargets[1] = fl;
  sharedTargets[2] = bl;
  sharedTargets[3] = br;

  sharedLastCommandMs = millis();
  sharedCommandLeaseActive =
    fabsf(fr) > 0.001f ||
    fabsf(fl) > 0.001f ||
    fabsf(bl) > 0.001f ||
    fabsf(br) > 0.001f;
  sharedWatchdogTimedOut = false;

  portEXIT_CRITICAL(&controlMux);
}

// ========================================================
// 9. DỪNG MỀM
// ========================================================
void activateSoftStop() {
  portENTER_CRITICAL(&controlMux);

  for (int i = 0; i < 4; i++) {
    sharedTargets[i] = 0.0f;
  }

  sharedCommandLeaseActive = false;

  portEXIT_CRITICAL(&controlMux);
}

// ========================================================
// 10. DỪNG KHẨN CẤP
// ========================================================
void activateEmergencyStop() {
  portENTER_CRITICAL(&controlMux);

  sharedEmergencyStop = true;

  for (int i = 0; i < 4; i++) {
    sharedTargets[i] = 0.0f;
  }

  sharedCommandLeaseActive = false;

  portEXIT_CRITICAL(&controlMux);
}

// ========================================================
// 11. MỞ KHÓA DỪNG KHẨN
// ========================================================
void releaseEmergencyStop() {
  portENTER_CRITICAL(&controlMux);

  sharedEmergencyStop = false;

  portEXIT_CRITICAL(&controlMux);
}

// ========================================================
// 12. ĐỌC SỐ THỰC TỪ THAM SỐ WEB
// ========================================================
bool readFloatArgument(
  const char *argumentName,
  float &result
) {
  if (!server.hasArg(argumentName)) {
    return false;
  }

  String text = server.arg(argumentName);
  text.trim();

  if (text.length() == 0) {
    return false;
  }

  char *endPointer = nullptr;

  result = strtof(
    text.c_str(),
    &endPointer
  );

  if (endPointer == text.c_str()) {
    return false;
  }

  while (
    *endPointer == ' ' ||
    *endPointer == '\t'
  ) {
    endPointer++;
  }

  if (*endPointer != '\0') {
    return false;
  }

  return isfinite(result);
}

// ========================================================
// 13. KIỂM TRA THÔNG SỐ
// ========================================================
bool validateParameters(
  const ControlParameters &parameters,
  String &errorMessage
) {
  if (
    parameters.maxAccelRPMPerSec < 1.0f ||
    parameters.maxAccelRPMPerSec > 3000.0f
  ) {
    errorMessage =
      "MAX_ACCEL_RPM_PER_SEC phải từ 1 đến 3000.";

    return false;
  }

  if (
    parameters.lpfAlpha <= 0.0f ||
    parameters.lpfAlpha > 1.0f
  ) {
    errorMessage =
      "LPF_ALPHA phải lớn hơn 0 và nhỏ hơn hoặc bằng 1.";

    return false;
  }

  if (
    parameters.Kf < 0.0f ||
    parameters.Kf > 10.0f
  ) {
    errorMessage =
      "Kf phải nằm trong khoảng 0 đến 10.";

    return false;
  }

  if (
    parameters.deadbandPWM < 0 ||
    parameters.deadbandPWM > 255
  ) {
    errorMessage =
      "deadbandPWM phải nằm trong khoảng 0 đến 255.";

    return false;
  }

  if (
    parameters.Kp < 0.0f ||
    parameters.Kp > 20.0f
  ) {
    errorMessage =
      "Kp phải nằm trong khoảng 0 đến 20.";

    return false;
  }

  if (
    parameters.Ki < 0.0f ||
    parameters.Ki > 20.0f
  ) {
    errorMessage =
      "Ki phải nằm trong khoảng 0 đến 20.";

    return false;
  }

  if (
    parameters.Kd < 0.0f ||
    parameters.Kd > 20.0f
  ) {
    errorMessage =
      "Kd phải nằm trong khoảng 0 đến 20.";

    return false;
  }

  return true;
}

// ========================================================
// 14. KIỂM TRA RPM
// ========================================================
bool isRPMValid(float rpm) {
  return (
    isfinite(rpm) &&
    fabsf(rpm) <= MAX_COMMAND_RPM
  );
}

void enforceCommandWatchdog(unsigned long now) {
  portENTER_CRITICAL(&controlMux);

  if (
    sharedCommandLeaseActive &&
    now - sharedLastCommandMs > COMMAND_WATCHDOG_MS
  ) {
    for (int i = 0; i < 4; i++) {
      sharedTargets[i] = 0.0f;
    }

    sharedCommandLeaseActive = false;
    sharedWatchdogTimedOut = true;
  }

  portEXIT_CRITICAL(&controlMux);
}

// ========================================================
// 15. TRANG CHỦ
// ========================================================
void handleRoot() {
  server.sendHeader(
    "Cache-Control",
    "no-store"
  );

  server.send_P(
    200,
    "text/html; charset=utf-8",
    INDEX_HTML
  );
}

// ========================================================
// 16. API ĐỌC TRẠNG THÁI
// ========================================================
void handleStatus() {
  ControlParameters parameters;
  float targets[4];
  bool emergencyStop;

  getControlSnapshot(
    parameters,
    targets,
    emergencyStop
  );

  TelemetryData localTelemetry;

  portENTER_CRITICAL(&telemetryMux);
  localTelemetry = telemetry;
  portEXIT_CRITICAL(&telemetryMux);

  String json;
  json.reserve(1000);

  json += "{";

  json += "\"estop\":";
  json += emergencyStop ? "true" : "false";

  json += ",\"clients\":";
  json += String(WiFi.softAPgetStationNum());

  json += ",\"uptime\":";
  json += String(millis() / 1000UL);

  json += ",\"params\":{";

  json += "\"accel\":";
  json += String(
    parameters.maxAccelRPMPerSec,
    3
  );

  json += ",\"alpha\":";
  json += String(
    parameters.lpfAlpha,
    4
  );

  json += ",\"kf\":";
  json += String(
    parameters.Kf,
    4
  );

  json += ",\"deadband\":";
  json += String(
    parameters.deadbandPWM
  );

  json += ",\"kp\":";
  json += String(
    parameters.Kp,
    5
  );

  json += ",\"ki\":";
  json += String(
    parameters.Ki,
    5
  );

  json += ",\"kd\":";
  json += String(
    parameters.Kd,
    5
  );

  json += "}";

  json += ",\"target\":[";

  for (int i = 0; i < 4; i++) {
    if (i > 0) {
      json += ",";
    }

    json += String(targets[i], 2);
  }

  json += "]";

  json += ",\"rpm\":[";

  for (int i = 0; i < 4; i++) {
    if (i > 0) {
      json += ",";
    }

    json += String(
      localTelemetry.currentRPM[i],
      2
    );
  }

  json += "]";

  json += ",\"pwm\":[";

  for (int i = 0; i < 4; i++) {
    if (i > 0) {
      json += ",";
    }

    json += String(
      localTelemetry.outputPWM[i]
    );
  }

  json += "]";

  json += ",\"reportValid\":[";

  for (int i = 0; i < 4; i++) {
    if (i > 0) {
      json += ",";
    }

    json +=
      localTelemetry.reportValid[i]
      ? "true"
      : "false";
  }

  json += "]";

  json += ",\"settling\":[";

  for (int i = 0; i < 4; i++) {
    if (i > 0) {
      json += ",";
    }

    json += String(
      localTelemetry.settlingTime[i],
      3
    );
  }

  json += "]";

  json += ",\"overshoot\":[";

  for (int i = 0; i < 4; i++) {
    if (i > 0) {
      json += ",";
    }

    json += String(
      localTelemetry.overshootPct[i],
      2
    );
  }

  json += "]";

  json += "}";

  server.sendHeader(
    "Cache-Control",
    "no-store"
  );

  server.send(
    200,
    "application/json; charset=utf-8",
    json
  );
}

// ========================================================
// 17. API CẬP NHẬT THÔNG SỐ PID
// ========================================================
void handleSetParameters() {
  float accel;
  float alpha;
  float kf;
  float deadbandFloat;
  float kp;
  float ki;
  float kd;

  bool validArguments =
    readFloatArgument("accel", accel) &&
    readFloatArgument("alpha", alpha) &&
    readFloatArgument("kf", kf) &&
    readFloatArgument(
      "deadband",
      deadbandFloat
    ) &&
    readFloatArgument("kp", kp) &&
    readFloatArgument("ki", ki) &&
    readFloatArgument("kd", kd);

  if (!validArguments) {
    server.send(
      400,
      "text/plain; charset=utf-8",
      "Dữ liệu thông số không hợp lệ."
    );

    return;
  }

  int deadband =
    (int)lroundf(deadbandFloat);

  if (
    fabsf(deadbandFloat - deadband) >
    0.001f
  ) {
    server.send(
      400,
      "text/plain; charset=utf-8",
      "deadbandPWM phải là số nguyên."
    );

    return;
  }

  ControlParameters newParameters = {
    accel,
    alpha,
    kf,
    deadband,
    kp,
    ki,
    kd
  };

  String errorMessage;

  if (
    !validateParameters(
      newParameters,
      errorMessage
    )
  ) {
    server.send(
      400,
      "text/plain; charset=utf-8",
      errorMessage
    );

    return;
  }

  portENTER_CRITICAL(&controlMux);
  sharedParams = newParameters;
  portEXIT_CRITICAL(&controlMux);

  server.send(
    200,
    "text/plain; charset=utf-8",
    "Đã cập nhật thông số PID."
  );
}

// ========================================================
// 18. API CẬP NHẬT RPM
// ========================================================
void handleSetRPM() {
  float fr;
  float fl;
  float bl;
  float br;

  bool validArguments =
    readFloatArgument("fr", fr) &&
    readFloatArgument("fl", fl) &&
    readFloatArgument("bl", bl) &&
    readFloatArgument("br", br);

  if (!validArguments) {
    server.send(
      400,
      "text/plain; charset=utf-8",
      "Giá trị RPM không hợp lệ."
    );

    return;
  }

  if (
    !isRPMValid(fr) ||
    !isRPMValid(fl) ||
    !isRPMValid(bl) ||
    !isRPMValid(br)
  ) {
    server.send(
      400,
      "text/plain; charset=utf-8",
      "RPM vượt giới hạn ±54,57 RPM (0,5 m/s)."
    );

    return;
  }

  bool emergencyStop;

  portENTER_CRITICAL(&controlMux);
  emergencyStop = sharedEmergencyStop;
  portEXIT_CRITICAL(&controlMux);

  if (emergencyStop) {
    server.send(
      409,
      "text/plain; charset=utf-8",
      "ESTOP đang bật. Hãy mở khóa ESTOP trước."
    );

    return;
  }

  setTargetRPM(
    fr,
    fl,
    bl,
    br
  );

  server.send(
    200,
    "text/plain; charset=utf-8",
    "Đã gửi RPM cho 4 bánh."
  );
}

// ========================================================
// 19. API DỪNG MỀM
// ========================================================
void handleSoftStop() {
  activateSoftStop();

  server.send(
    200,
    "text/plain; charset=utf-8",
    "Đã đặt RPM về 0 theo gia tốc giảm."
  );
}

// ========================================================
// 20. API DỪNG KHẨN
// ========================================================
void handleEmergencyStop() {
  activateEmergencyStop();

  server.send(
    200,
    "text/plain; charset=utf-8",
    "ESTOP đã bật. PWM được cắt ngay."
  );
}

// ========================================================
// 21. API MỞ KHÓA ESTOP
// ========================================================
void handleRun() {
  releaseEmergencyStop();

  server.send(
    200,
    "text/plain; charset=utf-8",
    "Đã mở khóa ESTOP. RPM hiện vẫn bằng 0."
  );
}

// ========================================================
// 22. CẤU HÌNH CÁC ĐƯỜNG DẪN WEB
// ========================================================
void configureWebServer() {
  server.on(
    "/",
    HTTP_GET,
    handleRoot
  );

  server.on(
    "/api/status",
    HTTP_GET,
    handleStatus
  );

  server.on(
    "/api/params",
    HTTP_POST,
    handleSetParameters
  );

  server.on(
    "/api/rpm",
    HTTP_POST,
    handleSetRPM
  );

  server.on(
    "/api/stop",
    HTTP_POST,
    handleSoftStop
  );

  server.on(
    "/api/estop",
    HTTP_POST,
    handleEmergencyStop
  );

  server.on(
    "/api/run",
    HTTP_POST,
    handleRun
  );

  // Hỗ trợ một số điện thoại tự kiểm tra captive portal
  server.on(
    "/generate_204",
    HTTP_GET,
    handleRoot
  );

  server.on(
    "/hotspot-detect.html",
    HTTP_GET,
    handleRoot
  );

  server.on(
    "/connecttest.txt",
    HTTP_GET,
    handleRoot
  );

  server.onNotFound(
    []() {
      server.sendHeader(
        "Location",
        "/",
        true
      );

      server.send(
        302,
        "text/plain",
        ""
      );
    }
  );
}

// ========================================================
// TASK CORE 0:
// WI-FI, WEB SERVER, NHẬN LỆNH VÀ GỬI DỮ LIỆU
// ========================================================
void communicationTask(void *pvParameters) {
  for (;;) {
    server.handleClient();

    // Core 0 xử lý Web liên tục nhưng vẫn nhường CPU
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ========================================================
// TASK CORE 1:
// ENCODER, PID VÀ PWM
// ========================================================
void motorControlTask(void *pvParameters) {
  TickType_t lastWakeTime =
    xTaskGetTickCount();

  const TickType_t taskFrequency =
    pdMS_TO_TICKS(PID_INTERVAL_MS);

  for (;;) {
    unsigned long currentMillis =
      millis();

    enforceCommandWatchdog(currentMillis);

    ControlParameters parameters;
    float targets[4];
    bool emergencyStop;

    // Lấy một bản sao đồng nhất tại đầu chu kỳ PID
    getControlSnapshot(
      parameters,
      targets,
      emergencyStop
    );

    float maximumRampStep =
      parameters.maxAccelRPMPerSec *
      (PID_INTERVAL_MS / 1000.0f);

    for (int i = 0; i < 4; i++) {
      MotorController &motor = motors[i];

      float targetRPM = targets[i];

      // ==================================================
      // 1. PHÁT HIỆN THAY ĐỔI RPM MỤC TIÊU
      // ==================================================
      if (
        !emergencyStop &&
        fabsf(targetRPM - motor.oldTargetRPM) >
          0.001f &&
        fabsf(targetRPM) > 0.001f
      ) {
        motor.isAnalyzing = true;
        motor.reportValid = false;

        motor.peakRPM = 0.0f;

        motor.stepStartTime =
          currentMillis;

        motor.settledStartTime = 0;

        motor.oldTargetRPM =
          targetRPM;
      }

      // ==================================================
      // 2. SOFT START / SOFT STOP
      // ==================================================
      if (!emergencyStop) {
        if (
          motor.rampedTargetRPM <
          targetRPM
        ) {
          motor.rampedTargetRPM +=
            maximumRampStep;

          if (
            motor.rampedTargetRPM >
            targetRPM
          ) {
            motor.rampedTargetRPM =
              targetRPM;
          }
        }
        else if (
          motor.rampedTargetRPM >
          targetRPM
        ) {
          motor.rampedTargetRPM -=
            maximumRampStep;

          if (
            motor.rampedTargetRPM <
            targetRPM
          ) {
            motor.rampedTargetRPM =
              targetRPM;
          }
        }
      }

      // ==================================================
      // 3. ĐỌC ENCODER
      // ==================================================
      long currentEncoderCount =
        motor.encoder.getCount();

      long deltaEncoder =
        currentEncoderCount -
        motor.previousEncoderCount;

      motor.previousEncoderCount =
        currentEncoderCount;

      motor.rawRPM =
        (
          (float)deltaEncoder *
          (1000.0f / PID_INTERVAL_MS) *
          60.0f
        ) /
        PULSES_PER_REV;

      // ==================================================
      // 4. LỌC THÔNG THẤP LPF
      // ==================================================
      motor.currentRPM =
        (
          parameters.lpfAlpha *
          motor.rawRPM
        ) +
        (
          (1.0f - parameters.lpfAlpha) *
          motor.currentRPM
        );

      // ==================================================
      // 5. DỪNG KHẨN CẤP
      // ==================================================
      if (emergencyStop) {
        motor.rampedTargetRPM = 0.0f;

        motor.pid.reset(motor.currentRPM);
        motor.outputPWM = 0;

        motor.oldTargetRPM = 0.0f;

        motor.isAnalyzing = false;
        motor.settledStartTime = 0;
      }

      // ==================================================
      // 6. PID + FEEDFORWARD
      // ==================================================
      else {
        if (
          fabsf(targetRPM) < 0.001f &&
          fabsf(motor.rampedTargetRPM) <
            0.001f
        ) {
          motor.pid.reset(motor.currentRPM);
          motor.outputPWM = 0;

          motor.oldTargetRPM = 0.0f;
          motor.isAnalyzing = false;
        }
        else {
          const amr_drive::PidConfig pidConfig = {
            parameters.Kp,
            parameters.Ki,
            parameters.Kd,
            parameters.Kf,
            static_cast<int16_t>(parameters.deadbandPWM),
            255.0f,
            255
          };

          motor.outputPWM = motor.pid.update(
            motor.rampedTargetRPM,
            motor.currentRPM,
            pidConfig
          );
        }
      }

      // ==================================================
      // 7. XUẤT PWM BTS7960
      // ==================================================
      if (motor.outputPWM > 0) {
        writeMotorPWMPin(
          i,
          true,
          motor.outputPWM
        );

        writeMotorPWMPin(
          i,
          false,
          0
        );
      }
      else if (motor.outputPWM < 0) {
        writeMotorPWMPin(
          i,
          true,
          0
        );

        writeMotorPWMPin(
          i,
          false,
          -motor.outputPWM
        );
      }
      else {
        writeMotorPWMPin(
          i,
          true,
          0
        );

        writeMotorPWMPin(
          i,
          false,
          0
        );
      }

      // ==================================================
      // 8. PHÂN TÍCH STEP RESPONSE
      // ==================================================
      if (
        !emergencyStop &&
        motor.isAnalyzing
      ) {
        if (
          fabsf(motor.currentRPM) >
          fabsf(motor.peakRPM)
        ) {
          motor.peakRPM =
            motor.currentRPM;
        }

        float errorBand =
          fmaxf(
            fabsf(targetRPM * 0.05f),
            3.0f
          );

        bool rampFinished =
          fabsf(
            motor.rampedTargetRPM -
            targetRPM
          ) < 0.001f;

        bool insideErrorBand =
          fabsf(
            targetRPM -
            motor.currentRPM
          ) <= errorBand;

        if (
          rampFinished &&
          insideErrorBand
        ) {
          if (
            motor.settledStartTime == 0
          ) {
            motor.settledStartTime =
              currentMillis;
          }
          else if (
            currentMillis -
            motor.settledStartTime >
            400
          ) {
            float overshootRPM =
              fabsf(motor.peakRPM) -
              fabsf(targetRPM);

            motor.reportTargetRPM =
              targetRPM;

            motor.reportSettlingTime =
              (
                motor.settledStartTime -
                motor.stepStartTime
              ) /
              1000.0f;

            if (
              overshootRPM > 0.0f &&
              fabsf(targetRPM) > 0.001f
            ) {
              motor.reportOvershootPct =
                (
                  overshootRPM /
                  fabsf(targetRPM)
                ) *
                100.0f;
            }
            else {
              motor.reportOvershootPct =
                0.0f;
            }

            motor.reportValid = true;
            motor.isAnalyzing = false;
          }
        }
        else {
          motor.settledStartTime = 0;
        }
      }
    }

    // ====================================================
    // 9. CẬP NHẬT TELEMETRY CHO CORE 0
    // ====================================================
    portENTER_CRITICAL(&telemetryMux);

    for (int i = 0; i < 4; i++) {
      telemetry.currentRPM[i] =
        motors[i].currentRPM;

      telemetry.outputPWM[i] =
        motors[i].outputPWM;

      telemetry.reportValid[i] =
        motors[i].reportValid;

      telemetry.reportTargetRPM[i] =
        motors[i].reportTargetRPM;

      telemetry.settlingTime[i] =
        motors[i].reportSettlingTime;

      telemetry.overshootPct[i] =
        motors[i].reportOvershootPct;
    }

    portEXIT_CRITICAL(&telemetryMux);

    // Khóa chu kỳ điều khiển đúng 20 ms
    vTaskDelayUntil(
      &lastWakeTime,
      taskFrequency
    );
  }
}

// ========================================================
// SETUP
// ========================================================
void setup() {
  // Không cần Serial và không cần dây USB khi vận hành

  // ------------------------------------------------------
  // 1. Khởi tạo encoder và PWM
  // ------------------------------------------------------
  for (int i = 0; i < 4; i++) {
    motors[i].encoder.attachHalfQuad(
      motors[i].encoderAPin,
      motors[i].encoderBPin
    );

    motors[i].encoder.clearCount();

    motors[i].encoder.setFilter(1023);

    motors[i].previousEncoderCount =
      motors[i].encoder.getCount();

    attachMotorPWM(i);
    writeMotorPWMPin(i, true, 0);
    writeMotorPWMPin(i, false, 0);
  }

  // ------------------------------------------------------
  // 2. Khởi tạo Wi-Fi Access Point
  // ------------------------------------------------------
  WiFi.mode(WIFI_AP);

  WiFi.softAPConfig(
    localIP,
    gateway,
    subnet
  );

  WiFi.softAP(
    WIFI_NAME,
    WIFI_PASSWORD
  );

  // ------------------------------------------------------
  // 3. Khởi tạo Web Server
  // ------------------------------------------------------
  configureWebServer();

  server.begin();

  // ------------------------------------------------------
  // 4. Core 0: Wi-Fi và Web Server
  // ------------------------------------------------------
  xTaskCreatePinnedToCore(
    communicationTask,
    "Communication_Task",
    10000,
    NULL,
    1,
    NULL,
    0
  );

  // ------------------------------------------------------
  // 5. Core 1: Encoder, PID và PWM
  // ------------------------------------------------------
  xTaskCreatePinnedToCore(
    motorControlTask,
    "Motor_Control_Task",
    8192,
    NULL,
    2,
    NULL,
    1
  );

  // Xóa task setup
  vTaskDelete(NULL);
}

// Không sử dụng loop
void loop() {
}

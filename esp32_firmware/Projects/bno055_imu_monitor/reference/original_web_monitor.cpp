#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// ========================================================
// 1. CẤU HÌNH WI-FI RIÊNG CHO ESP32 IMU
// ========================================================
// ESP32 điều khiển động cơ đang dùng 192.168.4.1.
// ESP32 IMU dùng 192.168.5.1 để tránh nhầm địa chỉ.

const char *AP_SSID = "BNO055_IMU_MONITOR";
const char *AP_PASSWORD = "12345678";

const uint8_t AP_CHANNEL = 6;
const uint8_t AP_MAX_CLIENTS = 2;

IPAddress localIP(192, 168, 5, 1);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// ========================================================
// 2. CẤU HÌNH BNO055
// ========================================================
const uint8_t BNO055_ADDRESS = 0x28;

// Nếu không tìm thấy cảm biến, thử đổi 0x28 thành 0x29.
Adafruit_BNO055 bno =
  Adafruit_BNO055(55, BNO055_ADDRESS, &Wire);

const int SDA_PIN = 21;
const int SCL_PIN = 22;

const int IMU_INTERVAL_MS = 40;  // 25 Hz

bool bnoAvailable = false;

// ========================================================
// 3. QUATERNION OFFSET
// Q_relative = inverse(Q_offset) × Q_current
// ========================================================
float inv_qw = 1.0f;
float inv_qx = 0.0f;
float inv_qy = 0.0f;
float inv_qz = 0.0f;

// ========================================================
// 4. DỮ LIỆU IMU DÙNG CHUNG GIỮA HAI CORE
// ========================================================
struct IMUData {
  bool sensorOK;

  // Quaternion tương đối
  float qx;
  float qy;
  float qz;
  float qw;

  // Góc Euler, đơn vị độ
  float roll;
  float pitch;
  float yaw;

  // Gia tốc, đơn vị m/s²
  float ax;
  float ay;
  float az;

  // Calibration từ 0 đến 3
  uint8_t calibSystem;
  uint8_t calibGyro;
  uint8_t calibAccel;
  uint8_t calibMag;

  uint32_t sampleCount;
  uint32_t lastUpdateMs;
};

IMUData sharedIMU = {
  false,
  0.0f, 0.0f, 0.0f, 1.0f,
  0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f,
  0, 0, 0, 0,
  0,
  0
};

portMUX_TYPE imuMux = portMUX_INITIALIZER_UNLOCKED;

// Cờ yêu cầu đặt lại zero từ Web
bool zeroRequested = false;
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;

// ========================================================
// 5. GIAO DIỆN WEB
// Không sử dụng thư viện JavaScript bên ngoài,
// nên vẫn hoạt động khi Wi-Fi không có Internet.
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

  <title>BNO055 IMU Monitor</title>

  <style>
    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      padding: 16px;
      background: #0f172a;
      color: #e5e7eb;
      font-family: Arial, Helvetica, sans-serif;
    }

    .container {
      max-width: 1300px;
      margin: auto;
    }

    h1 {
      text-align: center;
      margin: 6px 0 18px;
      font-size: 26px;
    }

    h2 {
      margin-top: 0;
      font-size: 20px;
    }

    .status-row {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-bottom: 15px;
    }

    .badge {
      background: #334155;
      padding: 9px 14px;
      border-radius: 8px;
      font-weight: bold;
    }

    .connected {
      background: #166534;
    }

    .disconnected {
      background: #991b1b;
    }

    .warning {
      background: #9a3412;
    }

    .grid {
      display: grid;
      grid-template-columns: repeat(
        auto-fit,
        minmax(330px, 1fr)
      );
      gap: 15px;
    }

    .card {
      background: #1e293b;
      border-radius: 13px;
      padding: 16px;
      box-shadow: 0 4px 14px rgba(0, 0, 0, 0.35);
    }

    .axis-values {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
    }

    .axis-box {
      background: #0f172a;
      border: 1px solid #334155;
      border-radius: 10px;
      padding: 14px 8px;
      text-align: center;
    }

    .axis-name {
      font-size: 17px;
      font-weight: bold;
    }

    .axis-number {
      margin-top: 7px;
      font-size: 25px;
      font-weight: bold;
    }

    .axis-unit {
      margin-top: 3px;
      color: #94a3b8;
      font-size: 13px;
    }

    .x-axis {
      color: #f87171;
    }

    .y-axis {
      color: #4ade80;
    }

    .z-axis {
      color: #60a5fa;
    }

    canvas {
      display: block;
      width: 100%;
      background: #07101f;
      border: 1px solid #334155;
      border-radius: 9px;
    }

    #axisCanvas {
      height: 370px;
    }

    .chart {
      height: 330px;
    }

    .button-row {
      display: flex;
      flex-wrap: wrap;
      gap: 9px;
      margin-top: 14px;
    }

    button {
      border: none;
      border-radius: 8px;
      padding: 11px 15px;
      color: white;
      background: #2563eb;
      cursor: pointer;
      font-weight: bold;
      font-size: 14px;
    }

    button:hover {
      opacity: 0.86;
    }

    .orange {
      background: #c2410c;
    }

    .gray {
      background: #475569;
    }

    .message {
      margin-top: 11px;
      min-height: 21px;
      font-weight: bold;
    }

    .ok-message {
      color: #4ade80;
    }

    .error-message {
      color: #f87171;
    }

    table {
      width: 100%;
      border-collapse: collapse;
    }

    th,
    td {
      padding: 10px 5px;
      border-bottom: 1px solid #334155;
      text-align: center;
    }

    th {
      color: #93c5fd;
    }

    .calibration {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 9px;
      margin-top: 13px;
    }

    .cal-box {
      padding: 11px 5px;
      text-align: center;
      background: #0f172a;
      border-radius: 8px;
    }

    .cal-value {
      margin-top: 5px;
      font-size: 24px;
      font-weight: bold;
    }

    .legend {
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      margin-top: 9px;
      font-size: 14px;
    }

    .legend-item {
      display: flex;
      align-items: center;
      gap: 6px;
    }

    .line {
      width: 22px;
      height: 4px;
      border-radius: 3px;
    }

    .line-x {
      background: #f87171;
    }

    .line-y {
      background: #4ade80;
    }

    .line-z {
      background: #60a5fa;
    }

    .full-width {
      margin-top: 15px;
    }

    .note {
      margin-top: 12px;
      color: #fbbf24;
      line-height: 1.5;
      font-size: 14px;
    }

    @media (max-width: 650px) {
      body {
        padding: 8px;
      }

      .axis-values {
        grid-template-columns: 1fr;
      }

      #axisCanvas {
        height: 300px;
      }

      .chart {
        height: 280px;
      }
    }
  </style>
</head>

<body>
<div class="container">

  <h1>BNO055 – Giám sát IMU không dây</h1>

  <div class="status-row">
    <div id="connectionBadge" class="badge disconnected">
      Mất kết nối
    </div>

    <div id="sensorBadge" class="badge warning">
      BNO055: Chưa sẵn sàng
    </div>

    <div id="clientBadge" class="badge">
      Laptop kết nối: 0
    </div>

    <div id="sampleBadge" class="badge">
      Mẫu: 0
    </div>

    <div id="ageBadge" class="badge">
      Tuổi dữ liệu: -- ms
    </div>
  </div>

  <div class="grid">

    <!-- GIÁ TRỊ BA TRỤC -->
    <div class="card">
      <h2>Góc tương đối ba trục</h2>

      <div class="axis-values">

        <div class="axis-box">
          <div class="axis-name x-axis">
            X – Roll
          </div>

          <div id="rollValue" class="axis-number x-axis">
            0.00
          </div>

          <div class="axis-unit">độ</div>
        </div>

        <div class="axis-box">
          <div class="axis-name y-axis">
            Y – Pitch
          </div>

          <div id="pitchValue" class="axis-number y-axis">
            0.00
          </div>

          <div class="axis-unit">độ</div>
        </div>

        <div class="axis-box">
          <div class="axis-name z-axis">
            Z – Yaw
          </div>

          <div id="yawValue" class="axis-number z-axis">
            0.00
          </div>

          <div class="axis-unit">độ</div>
        </div>

      </div>

      <div class="button-row">
        <button class="orange" onclick="setZero()">
          Đặt vị trí hiện tại làm ZERO
        </button>

        <button class="gray" onclick="clearCharts()">
          Xóa đồ thị
        </button>
      </div>

      <div id="commandMessage" class="message"></div>

      <p class="note">
        Sau khi nhấn ZERO, vị trí hiện tại của cảm biến được xem
        là Roll = Pitch = Yaw = 0°.
      </p>
    </div>

    <!-- MÔ HÌNH BA TRỤC -->
    <div class="card">
      <h2>Mô hình trục X, Y, Z</h2>

      <canvas id="axisCanvas"></canvas>

      <div class="legend">
        <div class="legend-item">
          <span class="line line-x"></span>
          Trục X
        </div>

        <div class="legend-item">
          <span class="line line-y"></span>
          Trục Y
        </div>

        <div class="legend-item">
          <span class="line line-z"></span>
          Trục Z
        </div>
      </div>
    </div>

  </div>

  <!-- GIA TỐC VÀ QUATERNION -->
  <div class="grid full-width">

    <div class="card">
      <h2>Gia tốc ba trục</h2>

      <table>
        <thead>
          <tr>
            <th>Trục</th>
            <th>Gia tốc</th>
            <th>Đơn vị</th>
          </tr>
        </thead>

        <tbody>
          <tr>
            <td class="x-axis">Ax</td>
            <td id="axValue">0.000</td>
            <td>m/s²</td>
          </tr>

          <tr>
            <td class="y-axis">Ay</td>
            <td id="ayValue">0.000</td>
            <td>m/s²</td>
          </tr>

          <tr>
            <td class="z-axis">Az</td>
            <td id="azValue">0.000</td>
            <td>m/s²</td>
          </tr>
        </tbody>
      </table>
    </div>

    <div class="card">
      <h2>Quaternion tương đối</h2>

      <table>
        <thead>
          <tr>
            <th>qx</th>
            <th>qy</th>
            <th>qz</th>
            <th>qw</th>
          </tr>
        </thead>

        <tbody>
          <tr>
            <td id="qxValue">0.0000</td>
            <td id="qyValue">0.0000</td>
            <td id="qzValue">0.0000</td>
            <td id="qwValue">1.0000</td>
          </tr>
        </tbody>
      </table>

      <h2 style="margin-top:18px;">
        Calibration
      </h2>

      <div class="calibration">
        <div class="cal-box">
          <div>SYS</div>
          <div id="calSys" class="cal-value">0</div>
        </div>

        <div class="cal-box">
          <div>GYRO</div>
          <div id="calGyro" class="cal-value">0</div>
        </div>

        <div class="cal-box">
          <div>ACC</div>
          <div id="calAccel" class="cal-value">0</div>
        </div>

        <div class="cal-box">
          <div>MAG</div>
          <div id="calMag" class="cal-value">0</div>
        </div>
      </div>

      <p class="note">
        Calibration có giá trị từ 0 đến 3.
        Giá trị 3 là mức hiệu chuẩn cao nhất.
      </p>
    </div>

  </div>

  <!-- ĐỒ THỊ GÓC -->
  <div class="card full-width">
    <h2>Đồ thị góc X, Y, Z theo thời gian</h2>

    <canvas id="angleChart" class="chart"></canvas>

    <div class="legend">
      <div class="legend-item">
        <span class="line line-x"></span>
        X – Roll
      </div>

      <div class="legend-item">
        <span class="line line-y"></span>
        Y – Pitch
      </div>

      <div class="legend-item">
        <span class="line line-z"></span>
        Z – Yaw
      </div>
    </div>
  </div>

  <!-- ĐỒ THỊ GIA TỐC -->
  <div class="card full-width">
    <h2>Đồ thị gia tốc X, Y, Z theo thời gian</h2>

    <canvas id="accelChart" class="chart"></canvas>

    <div class="legend">
      <div class="legend-item">
        <span class="line line-x"></span>
        Ax
      </div>

      <div class="legend-item">
        <span class="line line-y"></span>
        Ay
      </div>

      <div class="legend-item">
        <span class="line line-z"></span>
        Az
      </div>
    </div>
  </div>

</div>

<script>
  const axisCanvas =
    document.getElementById("axisCanvas");

  const axisContext =
    axisCanvas.getContext("2d");

  const angleCanvas =
    document.getElementById("angleChart");

  const angleContext =
    angleCanvas.getContext("2d");

  const accelCanvas =
    document.getElementById("accelChart");

  const accelContext =
    accelCanvas.getContext("2d");

  const maximumHistoryPoints = 250;

  let angleHistory = [];
  let accelHistory = [];

  let lastSampleNumber = -1;
  let communicationBusy = false;

  let currentQuaternion = {
    x: 0,
    y: 0,
    z: 0,
    w: 1
  };

  function resizeCanvas(canvas, context) {
    const ratio =
      window.devicePixelRatio || 1;

    const width =
      canvas.clientWidth;

    const height =
      canvas.clientHeight;

    if (
      canvas.width !== width * ratio ||
      canvas.height !== height * ratio
    ) {
      canvas.width = width * ratio;
      canvas.height = height * ratio;

      context.setTransform(
        ratio,
        0,
        0,
        ratio,
        0,
        0
      );
    }

    return {
      width: width,
      height: height
    };
  }

  function setMessage(text, isError) {
    const message =
      document.getElementById("commandMessage");

    message.textContent = text;

    message.className =
      "message " +
      (isError
        ? "error-message"
        : "ok-message");
  }

  async function setZero() {
    try {
      const response = await fetch(
        "/api/zero",
        {
          method: "POST",
          cache: "no-store"
        }
      );

      const text = await response.text();

      if (!response.ok) {
        throw new Error(text);
      }

      clearCharts();

      setMessage(
        "Đã gửi yêu cầu đặt lại mốc ZERO.",
        false
      );
    }
    catch (error) {
      setMessage(
        error.message,
        true
      );
    }
  }

  function clearCharts() {
    angleHistory = [];
    accelHistory = [];

    drawHistoryChart(
      angleCanvas,
      angleContext,
      angleHistory,
      "Góc",
      "°",
      10
    );

    drawHistoryChart(
      accelCanvas,
      accelContext,
      accelHistory,
      "Gia tốc",
      "m/s²",
      2
    );
  }

  function rotateVectorByQuaternion(vector, quaternion) {
    const x = quaternion.x;
    const y = quaternion.y;
    const z = quaternion.z;
    const w = quaternion.w;

    const vx = vector[0];
    const vy = vector[1];
    const vz = vector[2];

    const matrix00 =
      1 - 2 * (y * y + z * z);

    const matrix01 =
      2 * (x * y - z * w);

    const matrix02 =
      2 * (x * z + y * w);

    const matrix10 =
      2 * (x * y + z * w);

    const matrix11 =
      1 - 2 * (x * x + z * z);

    const matrix12 =
      2 * (y * z - x * w);

    const matrix20 =
      2 * (x * z - y * w);

    const matrix21 =
      2 * (y * z + x * w);

    const matrix22 =
      1 - 2 * (x * x + y * y);

    return [
      matrix00 * vx +
        matrix01 * vy +
        matrix02 * vz,

      matrix10 * vx +
        matrix11 * vy +
        matrix12 * vz,

      matrix20 * vx +
        matrix21 * vy +
        matrix22 * vz
    ];
  }

  function drawArrow(
    context,
    startX,
    startY,
    endX,
    endY,
    color,
    label,
    lineWidth
  ) {
    const headLength = 12;

    const angle =
      Math.atan2(
        endY - startY,
        endX - startX
      );

    context.strokeStyle = color;
    context.fillStyle = color;
    context.lineWidth = lineWidth;

    context.beginPath();
    context.moveTo(startX, startY);
    context.lineTo(endX, endY);
    context.stroke();

    context.beginPath();
    context.moveTo(endX, endY);

    context.lineTo(
      endX -
        headLength *
        Math.cos(angle - Math.PI / 6),

      endY -
        headLength *
        Math.sin(angle - Math.PI / 6)
    );

    context.lineTo(
      endX -
        headLength *
        Math.cos(angle + Math.PI / 6),

      endY -
        headLength *
        Math.sin(angle + Math.PI / 6)
    );

    context.closePath();
    context.fill();

    context.font = "bold 17px Arial";

    context.fillText(
      label,
      endX + 8,
      endY - 7
    );
  }

  function drawAxisModel() {
    const size =
      resizeCanvas(
        axisCanvas,
        axisContext
      );

    const width = size.width;
    const height = size.height;

    axisContext.clearRect(
      0,
      0,
      width,
      height
    );

    axisContext.fillStyle = "#07101f";

    axisContext.fillRect(
      0,
      0,
      width,
      height
    );

    const centerX = width * 0.5;
    const centerY = height * 0.54;

    const scale =
      Math.min(width, height) * 0.34;

    function project(vector) {
      const viewX =
        0.86 * vector[0] -
        0.55 * vector[1];

      const viewY =
        0.34 * vector[0] +
        0.34 * vector[1] -
        0.90 * vector[2];

      return {
        x: centerX + viewX * scale,
        y: centerY + viewY * scale
      };
    }

    // Trục tham chiếu cố định
    const fixedAxes = [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1]
    ];

    axisContext.globalAlpha = 0.22;

    const fixedColors = [
      "#f87171",
      "#4ade80",
      "#60a5fa"
    ];

    for (let i = 0; i < 3; i++) {
      const point =
        project(fixedAxes[i]);

      drawArrow(
        axisContext,
        centerX,
        centerY,
        point.x,
        point.y,
        fixedColors[i],
        "",
        2
      );
    }

    axisContext.globalAlpha = 1;

    const rotatedX =
      rotateVectorByQuaternion(
        [1, 0, 0],
        currentQuaternion
      );

    const rotatedY =
      rotateVectorByQuaternion(
        [0, 1, 0],
        currentQuaternion
      );

    const rotatedZ =
      rotateVectorByQuaternion(
        [0, 0, 1],
        currentQuaternion
      );

    const pointX = project(rotatedX);
    const pointY = project(rotatedY);
    const pointZ = project(rotatedZ);

    drawArrow(
      axisContext,
      centerX,
      centerY,
      pointX.x,
      pointX.y,
      "#f87171",
      "X",
      5
    );

    drawArrow(
      axisContext,
      centerX,
      centerY,
      pointY.x,
      pointY.y,
      "#4ade80",
      "Y",
      5
    );

    drawArrow(
      axisContext,
      centerX,
      centerY,
      pointZ.x,
      pointZ.y,
      "#60a5fa",
      "Z",
      5
    );

    // Điểm tâm
    axisContext.fillStyle = "#ffffff";
    axisContext.beginPath();
    axisContext.arc(
      centerX,
      centerY,
      6,
      0,
      Math.PI * 2
    );
    axisContext.fill();
  }

  function drawHistoryChart(
    canvas,
    context,
    history,
    title,
    unit,
    minimumRange
  ) {
    const size =
      resizeCanvas(canvas, context);

    const width = size.width;
    const height = size.height;

    const left = 55;
    const right = 16;
    const top = 18;
    const bottom = 30;

    const plotWidth =
      width - left - right;

    const plotHeight =
      height - top - bottom;

    context.clearRect(
      0,
      0,
      width,
      height
    );

    context.fillStyle = "#07101f";

    context.fillRect(
      0,
      0,
      width,
      height
    );

    let maximumAbsolute = minimumRange;

    for (const point of history) {
      maximumAbsolute = Math.max(
        maximumAbsolute,
        Math.abs(point.x),
        Math.abs(point.y),
        Math.abs(point.z)
      );
    }

    maximumAbsolute =
      Math.ceil(maximumAbsolute / 5) * 5;

    const yMaximum = maximumAbsolute;
    const yMinimum = -maximumAbsolute;

    function mapY(value) {
      return (
        top +
        (yMaximum - value) /
        (yMaximum - yMinimum) *
        plotHeight
      );
    }

    context.strokeStyle = "#26364f";
    context.lineWidth = 1;

    context.fillStyle = "#94a3b8";
    context.font = "12px Arial";

    const gridLines = 8;

    for (
      let index = 0;
      index <= gridLines;
      index++
    ) {
      const y =
        top +
        index / gridLines *
        plotHeight;

      const value =
        yMaximum -
        index / gridLines *
        (yMaximum - yMinimum);

      context.beginPath();
      context.moveTo(left, y);
      context.lineTo(
        left + plotWidth,
        y
      );
      context.stroke();

      context.fillText(
        value.toFixed(0),
        5,
        y + 4
      );
    }

    // Đường zero
    context.strokeStyle = "#94a3b8";
    context.lineWidth = 1.5;

    context.beginPath();
    context.moveTo(left, mapY(0));
    context.lineTo(
      left + plotWidth,
      mapY(0)
    );
    context.stroke();

    context.fillStyle = "#cbd5e1";

    context.fillText(
      title + " (" + unit + ")",
      left,
      13
    );

    if (history.length < 2) {
      return;
    }

    function drawLine(key, color) {
      context.strokeStyle = color;
      context.lineWidth = 2;
      context.beginPath();

      for (
        let index = 0;
        index < history.length;
        index++
      ) {
        const x =
          left +
          index /
          (maximumHistoryPoints - 1) *
          plotWidth;

        const y =
          mapY(history[index][key]);

        if (index === 0) {
          context.moveTo(x, y);
        }
        else {
          context.lineTo(x, y);
        }
      }

      context.stroke();
    }

    drawLine("x", "#f87171");
    drawLine("y", "#4ade80");
    drawLine("z", "#60a5fa");
  }

  function updateCalibrationColor(elementId, value) {
    const element =
      document.getElementById(elementId);

    if (value >= 3) {
      element.style.color = "#4ade80";
    }
    else if (value >= 1) {
      element.style.color = "#fbbf24";
    }
    else {
      element.style.color = "#f87171";
    }
  }

  async function updateIMU() {
    if (communicationBusy) {
      return;
    }

    communicationBusy = true;

    try {
      const response = await fetch(
        "/api/imu",
        {
          cache: "no-store"
        }
      );

      if (!response.ok) {
        throw new Error(
          "ESP32 không phản hồi."
        );
      }

      const data = await response.json();

      const connectionBadge =
        document.getElementById(
          "connectionBadge"
        );

      connectionBadge.textContent =
        "Đã kết nối ESP32 IMU";

      connectionBadge.className =
        "badge connected";

      const sensorBadge =
        document.getElementById(
          "sensorBadge"
        );

      if (data.ok) {
        sensorBadge.textContent =
          "BNO055: Hoạt động";

        sensorBadge.className =
          "badge connected";
      }
      else {
        sensorBadge.textContent =
          "BNO055: Không tìm thấy";

        sensorBadge.className =
          "badge warning";
      }

      document.getElementById(
        "clientBadge"
      ).textContent =
        "Laptop kết nối: " +
        data.clients;

      document.getElementById(
        "sampleBadge"
      ).textContent =
        "Mẫu: " +
        data.samples;

      document.getElementById(
        "ageBadge"
      ).textContent =
        "Tuổi dữ liệu: " +
        data.age +
        " ms";

      document.getElementById(
        "rollValue"
      ).textContent =
        data.angle[0].toFixed(2);

      document.getElementById(
        "pitchValue"
      ).textContent =
        data.angle[1].toFixed(2);

      document.getElementById(
        "yawValue"
      ).textContent =
        data.angle[2].toFixed(2);

      document.getElementById(
        "axValue"
      ).textContent =
        data.accel[0].toFixed(3);

      document.getElementById(
        "ayValue"
      ).textContent =
        data.accel[1].toFixed(3);

      document.getElementById(
        "azValue"
      ).textContent =
        data.accel[2].toFixed(3);

      document.getElementById(
        "qxValue"
      ).textContent =
        data.q[0].toFixed(4);

      document.getElementById(
        "qyValue"
      ).textContent =
        data.q[1].toFixed(4);

      document.getElementById(
        "qzValue"
      ).textContent =
        data.q[2].toFixed(4);

      document.getElementById(
        "qwValue"
      ).textContent =
        data.q[3].toFixed(4);

      document.getElementById(
        "calSys"
      ).textContent =
        data.cal[0];

      document.getElementById(
        "calGyro"
      ).textContent =
        data.cal[1];

      document.getElementById(
        "calAccel"
      ).textContent =
        data.cal[2];

      document.getElementById(
        "calMag"
      ).textContent =
        data.cal[3];

      updateCalibrationColor(
        "calSys",
        data.cal[0]
      );

      updateCalibrationColor(
        "calGyro",
        data.cal[1]
      );

      updateCalibrationColor(
        "calAccel",
        data.cal[2]
      );

      updateCalibrationColor(
        "calMag",
        data.cal[3]
      );

      currentQuaternion = {
        x: data.q[0],
        y: data.q[1],
        z: data.q[2],
        w: data.q[3]
      };

      drawAxisModel();

      // Chỉ thêm dữ liệu khi có mẫu IMU mới
      if (
        data.ok &&
        data.samples !== lastSampleNumber
      ) {
        lastSampleNumber = data.samples;

        angleHistory.push({
          x: data.angle[0],
          y: data.angle[1],
          z: data.angle[2]
        });

        accelHistory.push({
          x: data.accel[0],
          y: data.accel[1],
          z: data.accel[2]
        });

        if (
          angleHistory.length >
          maximumHistoryPoints
        ) {
          angleHistory.shift();
        }

        if (
          accelHistory.length >
          maximumHistoryPoints
        ) {
          accelHistory.shift();
        }

        drawHistoryChart(
          angleCanvas,
          angleContext,
          angleHistory,
          "Góc",
          "°",
          10
        );

        drawHistoryChart(
          accelCanvas,
          accelContext,
          accelHistory,
          "Gia tốc",
          "m/s²",
          2
        );
      }
    }
    catch (error) {
      const connectionBadge =
        document.getElementById(
          "connectionBadge"
        );

      connectionBadge.textContent =
        "Mất kết nối ESP32 IMU";

      connectionBadge.className =
        "badge disconnected";
    }
    finally {
      communicationBusy = false;
    }
  }

  window.addEventListener(
    "resize",
    function() {
      drawAxisModel();

      drawHistoryChart(
        angleCanvas,
        angleContext,
        angleHistory,
        "Góc",
        "°",
        10
      );

      drawHistoryChart(
        accelCanvas,
        accelContext,
        accelHistory,
        "Gia tốc",
        "m/s²",
        2
      );
    }
  );

  drawAxisModel();
  clearCharts();

  // Giao diện đọc khoảng 20 lần mỗi giây.
  // IMU cập nhật 25 lần mỗi giây.
  setInterval(updateIMU, 50);

  updateIMU();
</script>
</body>
</html>
)HTML";

// ========================================================
// 6. CHUẨN HÓA QUATERNION
// ========================================================
void normalizeQuaternion(
  float &qw,
  float &qx,
  float &qy,
  float &qz) {
  float magnitude = sqrtf(
    qw * qw + qx * qx + qy * qy + qz * qz);

  if (magnitude < 0.000001f) {
    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;
    return;
  }

  qw /= magnitude;
  qx /= magnitude;
  qy /= magnitude;
  qz /= magnitude;
}

// ========================================================
// 7. ĐẶT QUATERNION HIỆN TẠI LÀM OFFSET
// ========================================================
void setQuaternionOffset(
  const imu::Quaternion &quaternion) {
  float qw = quaternion.w();
  float qx = quaternion.x();
  float qy = quaternion.y();
  float qz = quaternion.z();

  normalizeQuaternion(
    qw,
    qx,
    qy,
    qz);

  // Nghịch đảo của quaternion đơn vị
  inv_qw = qw;
  inv_qx = -qx;
  inv_qy = -qy;
  inv_qz = -qz;
}

// ========================================================
// 8. KIỂM TRA YÊU CẦU ĐẶT LẠI ZERO
// ========================================================
bool takeZeroRequest() {
  bool request;

  portENTER_CRITICAL(&commandMux);

  request = zeroRequested;
  zeroRequested = false;

  portEXIT_CRITICAL(&commandMux);

  return request;
}

// ========================================================
// 9. CHUYỂN QUATERNION SANG ROLL, PITCH, YAW
// ========================================================
void quaternionToEuler(
  float qw,
  float qx,
  float qy,
  float qz,
  float &roll,
  float &pitch,
  float &yaw) {
  // Roll quanh trục X
  float sinRoll =
    2.0f * (qw * qx + qy * qz);

  float cosRoll =
    1.0f - 2.0f * (qx * qx + qy * qy);

  roll =
    atan2f(sinRoll, cosRoll);

  // Pitch quanh trục Y
  float sinPitch =
    2.0f * (qw * qy - qz * qx);

  sinPitch =
    constrain(
      sinPitch,
      -1.0f,
      1.0f);

  pitch =
    asinf(sinPitch);

  // Yaw quanh trục Z
  float sinYaw =
    2.0f * (qw * qz + qx * qy);

  float cosYaw =
    1.0f - 2.0f * (qy * qy + qz * qz);

  yaw =
    atan2f(sinYaw, cosYaw);

  const float RAD_TO_DEGREE =
    180.0f / PI;

  roll *= RAD_TO_DEGREE;
  pitch *= RAD_TO_DEGREE;
  yaw *= RAD_TO_DEGREE;
}

// ========================================================
// 10. SAO CHÉP DỮ LIỆU IMU AN TOÀN
// ========================================================
IMUData getIMUSnapshot() {
  IMUData data;

  portENTER_CRITICAL(&imuMux);
  data = sharedIMU;
  portEXIT_CRITICAL(&imuMux);

  return data;
}

// ========================================================
// 11. TRANG CHỦ
// ========================================================
void handleRoot() {
  server.sendHeader(
    "Cache-Control",
    "no-store");

  server.send_P(
    200,
    "text/html; charset=utf-8",
    INDEX_HTML);
}

// ========================================================
// 12. API GỬI DỮ LIỆU IMU
// ========================================================
void handleIMUData() {
  IMUData data =
    getIMUSnapshot();

  uint32_t dataAge = 0;

  if (data.lastUpdateMs > 0) {
    dataAge =
      millis() - data.lastUpdateMs;
  }

  String json;
  json.reserve(550);

  json += "{";

  json += "\"ok\":";
  json += data.sensorOK
            ? "true"
            : "false";

  json += ",\"clients\":";
  json += String(
    WiFi.softAPgetStationNum());

  json += ",\"uptime\":";
  json += String(
    millis() / 1000UL);

  json += ",\"age\":";
  json += String(dataAge);

  json += ",\"samples\":";
  json += String(data.sampleCount);

  json += ",\"q\":[";

  json += String(data.qx, 6);
  json += ",";
  json += String(data.qy, 6);
  json += ",";
  json += String(data.qz, 6);
  json += ",";
  json += String(data.qw, 6);

  json += "]";

  json += ",\"angle\":[";

  json += String(data.roll, 3);
  json += ",";
  json += String(data.pitch, 3);
  json += ",";
  json += String(data.yaw, 3);

  json += "]";

  json += ",\"accel\":[";

  json += String(data.ax, 4);
  json += ",";
  json += String(data.ay, 4);
  json += ",";
  json += String(data.az, 4);

  json += "]";

  json += ",\"cal\":[";

  json += String(data.calibSystem);
  json += ",";
  json += String(data.calibGyro);
  json += ",";
  json += String(data.calibAccel);
  json += ",";
  json += String(data.calibMag);

  json += "]";

  json += "}";

  server.sendHeader(
    "Cache-Control",
    "no-store");

  server.send(
    200,
    "application/json; charset=utf-8",
    json);
}

// ========================================================
// 13. API YÊU CẦU ĐẶT LẠI ZERO
// ========================================================
void handleSetZero() {
  IMUData data =
    getIMUSnapshot();

  if (!data.sensorOK) {
    server.send(
      503,
      "text/plain; charset=utf-8",
      "BNO055 chưa sẵn sàng.");

    return;
  }

  portENTER_CRITICAL(&commandMux);
  zeroRequested = true;
  portEXIT_CRITICAL(&commandMux);

  server.send(
    200,
    "text/plain; charset=utf-8",
    "Đã yêu cầu đặt lại mốc ZERO.");
}

// ========================================================
// 14. CẤU HÌNH WEB SERVER
// ========================================================
void configureWebServer() {
  server.on(
    "/",
    HTTP_GET,
    handleRoot);

  server.on(
    "/api/imu",
    HTTP_GET,
    handleIMUData);

  server.on(
    "/api/zero",
    HTTP_POST,
    handleSetZero);

  // Hỗ trợ một số hệ điều hành tự kiểm tra Internet
  server.on(
    "/generate_204",
    HTTP_GET,
    handleRoot);

  server.on(
    "/hotspot-detect.html",
    HTTP_GET,
    handleRoot);

  server.on(
    "/connecttest.txt",
    HTTP_GET,
    handleRoot);

  server.onNotFound(
    []() {
      server.sendHeader(
        "Location",
        "/",
        true);

      server.send(
        302,
        "text/plain",
        "");
    });
}

// ========================================================
// 15. KHỞI TẠO BNO055
// ========================================================
bool initializeBNO055() {
  if (!bno.begin()) {
    return false;
  }

  bno.setExtCrystalUse(true);

  // Chờ chế độ NDOF ổn định
  vTaskDelay(
    pdMS_TO_TICKS(1000));

  imu::Quaternion initialQuaternion =
    bno.getQuat();

  setQuaternionOffset(
    initialQuaternion);

  return true;
}

// ========================================================
// TASK CORE 1:
// ĐỌC BNO055 VÀ TÍNH TOÁN QUATERNION
// ========================================================
void TaskIMU(void *pvParameters) {
  TickType_t lastWakeTime =
    xTaskGetTickCount();

  const TickType_t frequency =
    pdMS_TO_TICKS(IMU_INTERVAL_MS);

  unsigned long lastRetryTime = 0;

  for (;;) {
    // Nếu chưa tìm thấy cảm biến, thử lại mỗi 2 giây
    if (!bnoAvailable) {
      unsigned long currentTime =
        millis();

      if (
        currentTime - lastRetryTime >= 2000) {
        lastRetryTime = currentTime;

        bnoAvailable =
          initializeBNO055();

        portENTER_CRITICAL(&imuMux);
        sharedIMU.sensorOK =
          bnoAvailable;
        portEXIT_CRITICAL(&imuMux);
      }

      vTaskDelay(
        pdMS_TO_TICKS(100));

      continue;
    }

    // ----------------------------------------
    // 1. Đọc quaternion hiện tại
    // ----------------------------------------
    imu::Quaternion currentQuaternion =
      bno.getQuat();

    // ----------------------------------------
    // 2. Đặt lại zero nếu Web yêu cầu
    // ----------------------------------------
    if (takeZeroRequest()) {
      setQuaternionOffset(
        currentQuaternion);
    }

    float current_qw =
      currentQuaternion.w();

    float current_qx =
      currentQuaternion.x();

    float current_qy =
      currentQuaternion.y();

    float current_qz =
      currentQuaternion.z();

    normalizeQuaternion(
      current_qw,
      current_qx,
      current_qy,
      current_qz);

    // ----------------------------------------
    // 3. Q_relative =
    //    inverse(Q_offset) × Q_current
    // ----------------------------------------
    float relative_qw =
      inv_qw * current_qw - inv_qx * current_qx - inv_qy * current_qy - inv_qz * current_qz;

    float relative_qx =
      inv_qw * current_qx + inv_qx * current_qw + inv_qy * current_qz - inv_qz * current_qy;

    float relative_qy =
      inv_qw * current_qy - inv_qx * current_qz + inv_qy * current_qw + inv_qz * current_qx;

    float relative_qz =
      inv_qw * current_qz + inv_qx * current_qy - inv_qy * current_qx + inv_qz * current_qw;

    normalizeQuaternion(
      relative_qw,
      relative_qx,
      relative_qy,
      relative_qz);

    // ----------------------------------------
    // 4. Chuyển sang Roll, Pitch, Yaw
    // ----------------------------------------
    float roll;
    float pitch;
    float yaw;

    quaternionToEuler(
      relative_qw,
      relative_qx,
      relative_qy,
      relative_qz,
      roll,
      pitch,
      yaw);

    // ----------------------------------------
    // 5. Đọc gia tốc
    // ----------------------------------------
    sensors_event_t accelData;

    bno.getEvent(
      &accelData,
      Adafruit_BNO055::VECTOR_ACCELEROMETER);

    // ----------------------------------------
    // 6. Đọc calibration
    // ----------------------------------------
    uint8_t systemCalibration;
    uint8_t gyroCalibration;
    uint8_t accelCalibration;
    uint8_t magCalibration;

    bno.getCalibration(
      &systemCalibration,
      &gyroCalibration,
      &accelCalibration,
      &magCalibration);

    // ----------------------------------------
    // 7. Ghi dữ liệu dùng chung cho Core 0
    // ----------------------------------------
    portENTER_CRITICAL(&imuMux);

    sharedIMU.sensorOK = true;

    sharedIMU.qx = relative_qx;
    sharedIMU.qy = relative_qy;
    sharedIMU.qz = relative_qz;
    sharedIMU.qw = relative_qw;

    sharedIMU.roll = roll;
    sharedIMU.pitch = pitch;
    sharedIMU.yaw = yaw;

    sharedIMU.ax =
      accelData.acceleration.x;

    sharedIMU.ay =
      accelData.acceleration.y;

    sharedIMU.az =
      accelData.acceleration.z;

    sharedIMU.calibSystem =
      systemCalibration;

    sharedIMU.calibGyro =
      gyroCalibration;

    sharedIMU.calibAccel =
      accelCalibration;

    sharedIMU.calibMag =
      magCalibration;

    sharedIMU.sampleCount++;
    sharedIMU.lastUpdateMs =
      millis();

    portEXIT_CRITICAL(&imuMux);

    // Chu kỳ chính xác khoảng 40 ms
    vTaskDelayUntil(
      &lastWakeTime,
      frequency);
  }
}

// ========================================================
// TASK CORE 0:
// WI-FI VÀ WEB SERVER
// ========================================================
void TaskWebServer(void *pvParameters) {
  for (;;) {
    server.handleClient();

    vTaskDelay(
      pdMS_TO_TICKS(2));
  }
}

// ========================================================
// SETUP
// ========================================================
void setup() {
  // Serial chỉ dùng để kiểm tra khi nạp code.
  // Khi vận hành không cần cắm USB.
  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println("======================================");
  Serial.println("ESP32 BNO055 IMU WEB MONITOR");
  Serial.println("======================================");

  // ----------------------------------------
  // 1. Khởi tạo I2C
  // ----------------------------------------
  Wire.begin(
    SDA_PIN,
    SCL_PIN);

  Wire.setClock(400000);

  // ----------------------------------------
  // 2. Khởi tạo BNO055
  // ----------------------------------------
  Serial.println(
    "Dang khoi tao BNO055...");

  bnoAvailable = bno.begin();

  if (bnoAvailable) {
    Serial.println(
      "Da tim thay BNO055.");

    bno.setExtCrystalUse(true);

    Serial.println(
      "Cho BNO055 on dinh trong 5 giay...");

    for (int i = 0; i < 125; i++) {
      delay(40);
    }

    imu::Quaternion initialQuaternion =
      bno.getQuat();

    setQuaternionOffset(
      initialQuaternion);

    portENTER_CRITICAL(&imuMux);
    sharedIMU.sensorOK = true;
    portEXIT_CRITICAL(&imuMux);

    Serial.println(
      "Da khoa moc ZERO ban dau.");
  } else {
    Serial.println(
      "Khong tim thay BNO055.");

    Serial.println(
      "Web van khoi dong va se tu thu ket noi lai.");
  }

  // ----------------------------------------
  // 3. Khởi tạo Wi-Fi Access Point
  // ----------------------------------------
  WiFi.mode(WIFI_AP);

  // Tắt chế độ ngủ Wi-Fi để giảm độ trễ Web
  WiFi.setSleep(false);

  WiFi.softAPConfig(
    localIP,
    gateway,
    subnet);

  bool accessPointStarted =
    WiFi.softAP(
      AP_SSID,
      AP_PASSWORD,
      AP_CHANNEL,
      false,
      AP_MAX_CLIENTS);

  if (accessPointStarted) {
    Serial.println(
      "Wi-Fi IMU da khoi dong.");

    Serial.print("SSID: ");
    Serial.println(AP_SSID);

    Serial.print("Dia chi Web: ");
    Serial.println(
      WiFi.softAPIP());
  } else {
    Serial.println(
      "Loi khoi dong Wi-Fi Access Point.");
  }

  // ----------------------------------------
  // 4. Khởi tạo Web Server
  // ----------------------------------------
  configureWebServer();
  server.begin();

  // ----------------------------------------
  // 5. Tạo task
  // ----------------------------------------
  // Core 0: Wi-Fi và Web
  xTaskCreatePinnedToCore(
    TaskWebServer,
    "Web_Server_Task",
    8192,
    NULL,
    1,
    NULL,
    0);

  // Core 1: BNO055 và tính toán IMU
  xTaskCreatePinnedToCore(
    TaskIMU,
    "IMU_Task",
    6144,
    NULL,
    2,
    NULL,
    1);
}

void loop() {
  // Không xử lý trong loop.
  vTaskDelay(
    pdMS_TO_TICKS(1000));
}

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>

namespace {

constexpr char kApSsid[] = "BNO055_IMU_MONITOR";
constexpr char kApPassword[] = "12345678";  // Bench only; change before use.
constexpr uint8_t kApChannel = 6;
constexpr uint8_t kMaxClients = 2;
constexpr uint8_t kBnoAddress = 0x28;
constexpr uint8_t kSdaPin = 21;
constexpr uint8_t kSclPin = 22;
constexpr uint32_t kSamplePeriodMs = 40;  // 25 Hz
constexpr uint32_t kRetryPeriodMs = 2000;
constexpr uint8_t kProtocolVersion = 1;

IPAddress localIp(192, 168, 5, 1);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 0);
WebServer server(80);
Adafruit_BNO055 bno(55, kBnoAddress, &Wire);

struct ImuData {
  bool sensor_ok = false;
  float qx = 0.0F;
  float qy = 0.0F;
  float qz = 0.0F;
  float qw = 1.0F;
  float gx = 0.0F;
  float gy = 0.0F;
  float gz = 0.0F;
  float ax = 0.0F;
  float ay = 0.0F;
  float az = 0.0F;
  float roll = 0.0F;
  float pitch = 0.0F;
  float yaw = 0.0F;
  uint8_t cal_system = 0;
  uint8_t cal_gyro = 0;
  uint8_t cal_accel = 0;
  uint8_t cal_mag = 0;
  uint32_t sequence = 0;
  uint32_t sample_ms = 0;
};

ImuData imuData;
portMUX_TYPE imuMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;
bool zeroRequested = false;

float invQw = 1.0F;
float invQx = 0.0F;
float invQy = 0.0F;
float invQz = 0.0F;

const char kIndexHtml[] PROGMEM = R"HTML(
<!doctype html><html lang="vi"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>BNO055 IMU Monitor</title><style>
body{font-family:system-ui;background:#0f172a;color:#e2e8f0;margin:0;padding:20px}
main{max-width:900px;margin:auto}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:12px}
.card{background:#1e293b;border-radius:12px;padding:16px}h1{font-size:24px}.value{font:600 18px monospace;line-height:1.7}
button{background:#2563eb;color:white;border:0;border-radius:8px;padding:10px 16px}.bad{color:#f87171}.ok{color:#4ade80}
</style></head><body><main><h1>BNO055 IMU Monitor</h1>
<p id="status">Đang kết nối…</p><div class="grid">
<section class="card"><h2>Quaternion</h2><div id="q" class="value"></div></section>
<section class="card"><h2>Góc (độ)</h2><div id="angle" class="value"></div></section>
<section class="card"><h2>Gyro (rad/s)</h2><div id="gyro" class="value"></div></section>
<section class="card"><h2>Gia tốc (m/s²)</h2><div id="accel" class="value"></div></section>
<section class="card"><h2>Calibration SYS/G/A/M</h2><div id="cal" class="value"></div></section>
</div><p><button onclick="zero()">Đặt lại ZERO</button></p></main><script>
const fmt=(a,n=3)=>a.map(v=>Number(v).toFixed(n)).join(' &nbsp; ');
async function update(){try{const r=await fetch('/api/imu',{cache:'no-store'});const d=await r.json();
status.textContent=d.ok?'BNO055 OK · mẫu '+d.sequence:'Không tìm thấy BNO055';status.className=d.ok?'ok':'bad';
q.innerHTML='x y z w<br>'+fmt(d.q,5);angle.innerHTML='roll pitch yaw<br>'+fmt(d.angle);
gyro.innerHTML='x y z<br>'+fmt(d.gyro);accel.innerHTML='x y z<br>'+fmt(d.accel);cal.textContent=d.cal.join(' / ');
}catch(e){status.textContent='Mất kết nối web';status.className='bad'}}
async function zero(){await fetch('/api/zero',{method:'POST'});await update()}setInterval(update,200);update();
</script></body></html>)HTML";

void normalize(float &w, float &x, float &y, float &z) {
  const float norm = sqrtf(w * w + x * x + y * y + z * z);
  if (!isfinite(norm) || norm < 1.0e-6F) {
    w = 1.0F;
    x = y = z = 0.0F;
    return;
  }
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;
}

void setZero(const imu::Quaternion &q) {
  float w = q.w();
  float x = q.x();
  float y = q.y();
  float z = q.z();
  normalize(w, x, y, z);
  invQw = w;
  invQx = -x;
  invQy = -y;
  invQz = -z;
}

void toEuler(float w, float x, float y, float z, float &roll, float &pitch,
             float &yaw) {
  roll = atan2f(2.0F * (w * x + y * z),
                1.0F - 2.0F * (x * x + y * y));
  const float sinPitch = 2.0F * (w * y - z * x);
  pitch = fabsf(sinPitch) >= 1.0F ? copysignf(PI / 2.0F, sinPitch)
                                  : asinf(sinPitch);
  yaw = atan2f(2.0F * (w * z + x * y),
               1.0F - 2.0F * (y * y + z * z));
  constexpr float kRadToDeg = 180.0F / PI;
  roll *= kRadToDeg;
  pitch *= kRadToDeg;
  yaw *= kRadToDeg;
}

ImuData snapshot() {
  portENTER_CRITICAL(&imuMux);
  const ImuData copy = imuData;
  portEXIT_CRITICAL(&imuMux);
  return copy;
}

void appendFloat(String &json, float value, unsigned int digits = 6) {
  json += isfinite(value) ? String(value, digits) : "null";
}

void handleImu() {
  const ImuData d = snapshot();
  String json;
  json.reserve(420);
  json += "{\"ok\":";
  json += d.sensor_ok ? "true" : "false";
  json += ",\"sequence\":" + String(d.sequence);
  json += ",\"age_ms\":" + String(millis() - d.sample_ms);
  json += ",\"q\":[";
  appendFloat(json, d.qx); json += ','; appendFloat(json, d.qy); json += ',';
  appendFloat(json, d.qz); json += ','; appendFloat(json, d.qw); json += ']';
  json += ",\"angle\":[";
  appendFloat(json, d.roll, 3); json += ','; appendFloat(json, d.pitch, 3);
  json += ','; appendFloat(json, d.yaw, 3); json += ']';
  json += ",\"gyro\":[";
  appendFloat(json, d.gx); json += ','; appendFloat(json, d.gy); json += ',';
  appendFloat(json, d.gz); json += ']';
  json += ",\"accel\":[";
  appendFloat(json, d.ax); json += ','; appendFloat(json, d.ay); json += ',';
  appendFloat(json, d.az); json += ']';
  json += ",\"cal\":[" + String(d.cal_system) + ',' + String(d.cal_gyro) +
          ',' + String(d.cal_accel) + ',' + String(d.cal_mag) + "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json; charset=utf-8", json);
}

void handleZero() {
  if (!snapshot().sensor_ok) {
    server.send(503, "text/plain", "BNO055 not ready");
    return;
  }
  portENTER_CRITICAL(&commandMux);
  zeroRequested = true;
  portEXIT_CRITICAL(&commandMux);
  server.send(202, "text/plain", "ZERO requested");
}

bool takeZeroRequest() {
  portENTER_CRITICAL(&commandMux);
  const bool requested = zeroRequested;
  zeroRequested = false;
  portEXIT_CRITICAL(&commandMux);
  return requested;
}

bool initializeSensor() {
  if (!bno.begin()) {
    return false;
  }
  bno.setExtCrystalUse(true);
  delay(1000);
  setZero(bno.getQuat());
  return true;
}

void publishSerial(const ImuData &d) {
  // Tagged protocol keeps boot/debug text distinguishable from sensor data:
  // IMU,version,sequence,sample_ms,qx,qy,qz,qw,gx,gy,gz,ax,ay,az,cal...
  Serial.printf(
      "IMU,%u,%lu,%lu,%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%u,%u\n",
      kProtocolVersion, static_cast<unsigned long>(d.sequence),
      static_cast<unsigned long>(d.sample_ms), d.qx, d.qy, d.qz, d.qw,
      d.gx, d.gy, d.gz, d.ax, d.ay, d.az, d.cal_system, d.cal_gyro,
      d.cal_accel, d.cal_mag);
}

void imuTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastRetryMs = 0;
  for (;;) {
    if (!snapshot().sensor_ok) {
      const uint32_t now = millis();
      if (now - lastRetryMs >= kRetryPeriodMs) {
        lastRetryMs = now;
        const bool ok = initializeSensor();
        portENTER_CRITICAL(&imuMux);
        imuData.sensor_ok = ok;
        portEXIT_CRITICAL(&imuMux);
        Serial.println(ok ? "STATUS:BNO055_READY" : "STATUS:BNO055_NOT_FOUND");
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      lastWake = xTaskGetTickCount();
      continue;
    }

    const imu::Quaternion current = bno.getQuat();
    if (takeZeroRequest()) {
      setZero(current);
      Serial.println("STATUS:ZERO_APPLIED");
    }
    float cw = current.w(), cx = current.x(), cy = current.y(), cz = current.z();
    normalize(cw, cx, cy, cz);
    float rw = invQw * cw - invQx * cx - invQy * cy - invQz * cz;
    float rx = invQw * cx + invQx * cw + invQy * cz - invQz * cy;
    float ry = invQw * cy - invQx * cz + invQy * cw + invQz * cx;
    float rz = invQw * cz + invQx * cy - invQy * cx + invQz * cw;
    normalize(rw, rx, ry, rz);

    sensors_event_t accel;
    sensors_event_t gyro;
    bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
    ImuData next;
    next.sensor_ok = true;
    next.qx = rx; next.qy = ry; next.qz = rz; next.qw = rw;
    next.gx = gyro.gyro.x; next.gy = gyro.gyro.y; next.gz = gyro.gyro.z;
    next.ax = accel.acceleration.x; next.ay = accel.acceleration.y;
    next.az = accel.acceleration.z;
    toEuler(rw, rx, ry, rz, next.roll, next.pitch, next.yaw);
    bno.getCalibration(&next.cal_system, &next.cal_gyro, &next.cal_accel,
                       &next.cal_mag);
    next.sample_ms = millis();
    portENTER_CRITICAL(&imuMux);
    next.sequence = imuData.sequence + 1;
    imuData = next;
    portEXIT_CRITICAL(&imuMux);
    publishSerial(next);
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kSamplePeriodMs));
  }
}

void webTask(void *) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("BOOT,BNO055_IMU_MONITOR,1");
  Wire.begin(kSdaPin, kSclPin);
  Wire.setClock(400000);

  const bool ready = initializeSensor();
  portENTER_CRITICAL(&imuMux);
  imuData.sensor_ok = ready;
  portEXIT_CRITICAL(&imuMux);
  Serial.println(ready ? "STATUS:BNO055_READY" : "STATUS:BNO055_NOT_FOUND");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(localIp, gateway, subnet);
  WiFi.softAP(kApSsid, kApPassword, kApChannel, false, kMaxClients);
  server.on("/", HTTP_GET,
            []() { server.send_P(200, "text/html; charset=utf-8", kIndexHtml); });
  server.on("/api/imu", HTTP_GET, handleImu);
  server.on("/api/zero", HTTP_POST, handleZero);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  xTaskCreatePinnedToCore(webTask, "imu_web", 6144, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(imuTask, "imu_read", 6144, nullptr, 2, nullptr, 1);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }

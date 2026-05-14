#include <Arduino.h>
#include <M5Atom.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Kalman.h>
#include "driver/twai.h"

// ATOM Matrix + SN65HVD230/VP230
#define CAN_TX_PIN GPIO_NUM_32
#define CAN_RX_PIN GPIO_NUM_26

// ID, パスワードは任意
static const char* AP_SSID = "ATOM-RS05-ID";
static const char* AP_PASS = "pass1234";

static const uint8_t MOTOR_ID = 0x7E;
static const uint8_t MASTER_ID = 0xFD;

static const uint8_t COMM_RUN_EN = 0x03;
static const uint8_t COMM_RUN_DIS = 0x04;
static const uint8_t COMM_READ_ONE = 0x11;
static const uint8_t COMM_WRITE_ONE = 0x12;

static const uint16_t IDX_RUN_MODE = 0x7005;
static const uint16_t IDX_SPD_REF = 0x700A;
static const uint16_t IDX_MECH_VEL = 0x701B;
static const uint16_t IDX_LIMIT_CUR = 0x7018;
static const uint16_t IDX_ACC_RAD = 0x7022;

static const float DEFAULT_LIMIT_CUR = 8.0f;    // A
static const float DEFAULT_ACC_RAD = 500.0f;    // rad/s^2
static const float SPEED_LIMIT = 50.0f;         // rad/s
static const float GAIN_LIMIT = 1.0f;
static const float WHEEL_SPEED_SIGN = 1.0f;
static const float MOTOR_ENABLE_ANGLE_DEG = 1.0f;
static const float MOTOR_DISABLE_ANGLE_DEG = 30.0f;
static const uint32_t MOTOR_CONFIG_DELAY_MS = 1000;
static const uint32_t CONTROL_PERIOD_US = 4000;
static const uint32_t LED_PERIOD_MS = 20;       // 50 Hz

static const uint8_t MPU6886_ADDR = 0x68;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

static const float INDICATOR_RANGE_DEG = 20.0f;
static const float LEVEL_DEG = 1.0f;
static const float RED_LIMIT_DEG = 15.0f;
static const CRGB COLOR_LEVEL = CRGB(0x00, 0xff, 0x00);
static const CRGB COLOR_RED_ZONE = CRGB(0xff, 0x00, 0x00);
static const CRGB COLOR_OTHER = CRGB(0x00, 0x00, 0xff);
static const CRGB COLOR_OFF = CRGB(0x00, 0x00, 0x00);

static twai_general_config_t g_config =
  TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

WebServer server(80);
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

struct ImuRaw {
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
};

struct ControlState {
  float targetSpeed;
  float sentSpeed;
  float wheelSpeed;
  float angleY;
  float gyroY;
  float kp;
  float kd;
  float kw;
  bool enabled;
  bool canOk;
  bool imuOk;
  bool wheelOk;
  uint32_t txOk;
  uint32_t txFail;
  uint32_t rxOk;
  uint32_t lastTxMs;
  uint32_t lastRxMs;
};

static ControlState state = {};
static Kalman kalmanY;
static TaskHandle_t webTaskHandle = nullptr;
static TaskHandle_t controlTaskHandle = nullptr;

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ATOM RS05 Tilt</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    body { margin: 0; background: #101318; color: #eef2f8; }
    main { max-width: 620px; margin: 0 auto; padding: 22px 14px 34px; }
    h1 { margin: 0 0 14px; font-size: 24px; }
    .panel { border: 1px solid #2c3440; border-radius: 8px; padding: 16px; background: #171b22; }
    .label { color: #9ea9bb; font-size: 13px; margin-top: 12px; }
    .angle { font: 800 52px/1 ui-monospace, SFMono-Regular, Consolas, monospace; margin: 4px 0 8px; }
    .speed { font: 800 44px/1 ui-monospace, SFMono-Regular, Consolas, monospace; margin: 4px 0 8px; }
    .unit { color: #9ea9bb; font-size: 18px; }
    .bar { position: relative; height: 14px; margin: 12px 0 20px; border-radius: 7px; background: linear-gradient(90deg, #ff2a2a 0 12.5%, #006dff 12.5% 47.5%, #00d866 47.5% 52.5%, #006dff 52.5% 87.5%, #ff2a2a 87.5% 100%); }
    .marker { position: absolute; top: -5px; left: 50%; width: 8px; height: 24px; border-radius: 4px; background: #eef2f8; box-shadow: 0 0 0 2px #101318; transform: translateX(-50%); }
    input[type=range] { width: 100%; accent-color: #4cc9f0; }
    input[type=number] { box-sizing: border-box; width: 100%; padding: 12px; border-radius: 6px; border: 1px solid #3a4352; background: #0f1217; color: #eef2f8; font: 700 20px ui-monospace, SFMono-Regular, Consolas, monospace; }
    .row { display: flex; gap: 10px; align-items: center; margin-top: 10px; }
    .row > * { flex: 1; }
    button { height: 46px; border: 0; border-radius: 6px; color: #071018; font-weight: 800; font-size: 16px; }
    .enable { background: #34d399; }
    .disable { background: #f97373; }
    .zero { background: #e5e7eb; }
    .status { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-top: 14px; color: #9ea9bb; font-size: 13px; }
    .status b { display: block; margin-top: 3px; color: #eef2f8; font: 700 18px ui-monospace, SFMono-Regular, Consolas, monospace; }
  </style>
</head>
<body>
  <main>
    <h1>ATOM Matrix Y Tilt + RobStride 05</h1>
    <section class="panel">
      <div class="label">Y Tilt</div>
      <div><span id="angleY" class="angle">0.0</span><span class="unit"> deg</span></div>
      <div class="bar"><span id="tiltMarker" class="marker"></span></div>

      <div class="label">PD Speed Command</div>
      <div><span id="speedText" class="speed">0.00</span><span class="unit"> rad/s</span></div>

      <div class="label">Kp</div>
      <input id="kpRange" type="range" min="0" max="1" step="0.05" value="0">
      <div class="row">
        <input id="kpInput" type="number" min="0" max="1" step="0.05" value="0">
      </div>

      <div class="label">Kd</div>
      <input id="kdRange" type="range" min="0" max="1" step="0.05" value="0">
      <div class="row">
        <input id="kdInput" type="number" min="0" max="1" step="0.05" value="0">
      </div>

      <div class="label">Kw</div>
      <input id="kwRange" type="range" min="0" max="1" step="0.05" value="0">
      <div class="row">
        <input id="kwInput" type="number" min="0" max="1" step="0.05" value="0">
      </div>

      <div class="status">
        <div>Motor<b id="enabled">-</b></div>
        <div>Sent<b id="sent">-</b></div>
        <div>Wheel<b id="wheel">-</b></div>
        <div>GyroY<b id="gyroY">-</b></div>
        <div>Gate<b id="gate">-</b></div>
        <div>CAN<b id="can">-</b></div>
        <div>IMU<b id="imu">-</b></div>
        <div>Wheel FB<b id="wheelFb">-</b></div>
        <div>TX/RX<b id="tx">-</b></div>
        <div>Last TX<b id="lastTx">-</b></div>
      </div>
    </section>
  </main>
  <script>
    const kpRange = document.getElementById("kpRange");
    const kpInput = document.getElementById("kpInput");
    const kdRange = document.getElementById("kdRange");
    const kdInput = document.getElementById("kdInput");
    const kwRange = document.getElementById("kwRange");
    const kwInput = document.getElementById("kwInput");
    const speedText = document.getElementById("speedText");
    const marker = document.getElementById("tiltMarker");
    let timer = 0;

    function clampGain(v) { return Math.max(0, Math.min(1, Number(v) || 0)); }
    function clampKw(v) { return Math.max(0, Math.min(1, Number(v) || 0)); }
    function showGain(kind, v) {
      if (kind === "kp") {
        const s = clampGain(v).toFixed(2);
        kpRange.value = s;
        kpInput.value = s;
      } else if (kind === "kd") {
        const s = clampGain(v).toFixed(2);
        kdRange.value = s;
        kdInput.value = s;
      } else {
        const s = clampKw(v).toFixed(2);
        kwRange.value = s;
        kwInput.value = s;
      }
    }
    async function command(kp, kd, kw) {
      const params = new URLSearchParams();
      if (kp !== null) params.set("kp", clampGain(kp).toFixed(3));
      if (kd !== null) params.set("kd", clampGain(kd).toFixed(3));
      if (kw !== null) params.set("kw", clampKw(kw).toFixed(3));
      await fetch("/api/cmd?" + params.toString(), { cache: "no-store" });
    }
    function scheduleGain(kind, v) {
      showGain(kind, v);
      clearTimeout(timer);
      timer = setTimeout(() => {
        command(kind === "kp" ? v : null, kind === "kd" ? v : null, kind === "kw" ? v : null).catch(() => {});
      }, 40);
    }
    kpRange.addEventListener("input", () => scheduleGain("kp", kpRange.value));
    kpInput.addEventListener("change", () => scheduleGain("kp", kpInput.value));
    kdRange.addEventListener("input", () => scheduleGain("kd", kdRange.value));
    kdInput.addEventListener("change", () => scheduleGain("kd", kdInput.value));
    kwRange.addEventListener("input", () => scheduleGain("kw", kwRange.value));
    kwInput.addEventListener("change", () => scheduleGain("kw", kwInput.value));

    async function poll() {
      try {
        const res = await fetch("/api/state", { cache: "no-store" });
        const s = await res.json();
        const angleY = Number(s.angleY);
        document.getElementById("angleY").textContent = angleY.toFixed(1);
        marker.style.left = Math.max(0, Math.min(100, ((20 - angleY) / 40) * 100)) + "%";
        marker.style.background = s.indicatorColor || "#eef2f8";
        document.getElementById("enabled").textContent = s.enabled ? "ON" : "OFF";
        speedText.textContent = Number(s.target).toFixed(2);
        document.getElementById("sent").textContent = Number(s.sent).toFixed(2);
        document.getElementById("wheel").textContent = Number(s.wheelSpeed).toFixed(2);
        document.getElementById("gyroY").textContent = Number(s.gyroY).toFixed(1);
        document.getElementById("gate").textContent = Math.abs(angleY) <= 1.0 ? "ENABLE" : (Math.abs(angleY) >= 30.0 ? "DISABLE" : "KEEP");
        document.getElementById("can").textContent = s.canOk ? "OK" : "ERR";
        document.getElementById("imu").textContent = s.imuOk ? "OK" : "ERR";
        document.getElementById("wheelFb").textContent = s.wheelOk ? "OK" : "WAIT";
        document.getElementById("tx").textContent = s.txOk + "/" + s.rxOk;
        document.getElementById("lastTx").textContent = s.lastTxMs + " ms";
        if (document.activeElement !== kpInput && document.activeElement !== kpRange) showGain("kp", s.kp);
        if (document.activeElement !== kdInput && document.activeElement !== kdRange) showGain("kd", s.kd);
        if (document.activeElement !== kwInput && document.activeElement !== kwRange) showGain("kw", s.kw);
      } catch (e) {}
    }
    showGain("kp", 0);
    showGain("kd", 0);
    showGain("kw", 0);
    setInterval(poll, 100);
    poll();
  </script>
</body>
</html>
)HTML";

static inline uint32_t buildExtId(uint8_t mode, uint16_t data16, uint8_t id8) {
  return ((uint32_t)(mode & 0x1F) << 24) | ((uint32_t)data16 << 8) | id8;
}

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static void floatToLe(float v, uint8_t* b) {
  uint32_t u;
  memcpy(&u, &v, sizeof(u));
  b[0] = (uint8_t)u;
  b[1] = (uint8_t)(u >> 8);
  b[2] = (uint8_t)(u >> 16);
  b[3] = (uint8_t)(u >> 24);
}

static bool canSend(const twai_message_t& tx, TickType_t timeoutTicks = pdMS_TO_TICKS(1)) {
  return twai_transmit((twai_message_t*)&tx, timeoutTicks) == ESP_OK;
}

static bool writeParamU8(uint16_t index, uint8_t val) {
  twai_message_t tx = {};
  tx.identifier = buildExtId(COMM_WRITE_ONE, ((uint16_t)MASTER_ID << 8), MOTOR_ID);
  tx.flags = TWAI_MSG_FLAG_EXTD;
  tx.data_length_code = 8;
  tx.data[0] = (uint8_t)(index & 0xFF);
  tx.data[1] = (uint8_t)(index >> 8);
  tx.data[4] = val;
  return canSend(tx, pdMS_TO_TICKS(5));
}

static bool writeParamFloat(uint16_t index, float val) {
  twai_message_t tx = {};
  tx.identifier = buildExtId(COMM_WRITE_ONE, ((uint16_t)MASTER_ID << 8), MOTOR_ID);
  tx.flags = TWAI_MSG_FLAG_EXTD;
  tx.data_length_code = 8;
  tx.data[0] = (uint8_t)(index & 0xFF);
  tx.data[1] = (uint8_t)(index >> 8);
  floatToLe(val, &tx.data[4]);
  return canSend(tx);
}

static bool sendRun(uint8_t command) {
  twai_message_t tx = {};
  tx.identifier = buildExtId(command, ((uint16_t)MASTER_ID << 8), MOTOR_ID);
  tx.flags = TWAI_MSG_FLAG_EXTD | TWAI_MSG_FLAG_SS;
  tx.data_length_code = 8;
  return canSend(tx, pdMS_TO_TICKS(5));
}

static bool requestWheelSpeed() {
  twai_message_t tx = {};
  tx.identifier = buildExtId(COMM_READ_ONE, ((uint16_t)MASTER_ID << 8), MOTOR_ID);
  tx.flags = TWAI_MSG_FLAG_EXTD;
  tx.data_length_code = 8;
  tx.data[0] = (uint8_t)(IDX_MECH_VEL & 0xFF);
  tx.data[1] = (uint8_t)(IDX_MECH_VEL >> 8);
  return canSend(tx);
}

static bool parseWheelSpeed(const twai_message_t& rx, float& wheelSpeed) {
  if ((rx.flags & TWAI_MSG_FLAG_EXTD) == 0 || rx.data_length_code != 8) {
    return false;
  }

  const uint8_t mode = (rx.identifier >> 24) & 0x1F;
  if (mode != COMM_READ_ONE) {
    return false;
  }

  const uint16_t index = (uint16_t)rx.data[0] | ((uint16_t)rx.data[1] << 8);
  if (index != IDX_MECH_VEL) {
    return false;
  }

  uint32_t raw = (uint32_t)rx.data[4]
    | ((uint32_t)rx.data[5] << 8)
    | ((uint32_t)rx.data[6] << 16)
    | ((uint32_t)rx.data[7] << 24);
  float value;
  memcpy(&value, &raw, sizeof(value));
  wheelSpeed = WHEEL_SPEED_SIGN * value;
  return true;
}

static void updateWheelFeedback() {
  twai_message_t rx = {};
  float wheelSpeed = 0.0f;
  bool gotWheel = false;

  while (twai_receive(&rx, 0) == ESP_OK) {
    if (parseWheelSpeed(rx, wheelSpeed)) {
      gotWheel = true;
    }
  }

  if (gotWheel) {
    portENTER_CRITICAL(&stateMux);
    state.wheelSpeed = wheelSpeed;
    state.wheelOk = true;
    state.rxOk++;
    state.lastRxMs = millis();
    portEXIT_CRITICAL(&stateMux);
  }
}

static CRGB indicatorColor(float angle) {
  const float absAngle = fabsf(angle);
  if (absAngle <= LEVEL_DEG) {
    return COLOR_LEVEL;
  }
  if (absAngle >= RED_LIMIT_DEG) {
    return COLOR_RED_ZONE;
  }
  return COLOR_OTHER;
}

static const char* indicatorColorHex(float angle) {
  const float absAngle = fabsf(angle);
  if (absAngle <= LEVEL_DEG) {
    return "#00ff00";
  }
  if (absAngle >= RED_LIMIT_DEG) {
    return "#ff0000";
  }
  return "#0000ff";
}

static void setMatrixColor(CRGB color) {
  for (uint8_t i = 0; i < 25; i++) {
    M5.dis.drawpix(i, color);
  }
}

static void drawTiltIndicator(float angle, bool imuOk) {
  if (!imuOk) {
    const bool blink = ((millis() / 250) % 2) == 0;
    setMatrixColor(blink ? CRGB(0x30, 0x00, 0x00) : COLOR_OFF);
    return;
  }

  const float clampedAngle = clampf(angle, -INDICATOR_RANGE_DEG, INDICATOR_RANGE_DEG);
  const int markerX = 4 - constrain((int)(((clampedAngle + INDICATOR_RANGE_DEG) / (INDICATOR_RANGE_DEG * 2.0f) * 4.0f) + 0.5f), 0, 4);
  const CRGB markerColor = indicatorColor(angle);

  for (uint8_t y = 0; y < 5; y++) {
    for (uint8_t x = 0; x < 5; x++) {
      M5.dis.drawpix((uint8_t)(y * 5 + x), x == markerX ? markerColor : COLOR_OFF);
    }
  }
}

static int16_t readInt16() {
  const uint8_t hi = Wire.read();
  const uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

static bool writeImuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6886_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static void configureImuRange() {
  writeImuRegister(REG_GYRO_CONFIG, 0x00);
  writeImuRegister(REG_ACCEL_CONFIG, 0x00);
}

static bool readRawImu(ImuRaw& imu) {
  Wire.beginTransmission(MPU6886_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU6886_ADDR, (uint8_t)14) != 14) {
    return false;
  }

  imu.ax = readInt16();
  imu.ay = readInt16();
  imu.az = -readInt16();
  readInt16();
  imu.gx = readInt16();
  imu.gy = readInt16();
  imu.gz = readInt16();
  return true;
}

static float accelAngleYDeg(const ImuRaw& imu) {
  return atan2f((float)imu.ax, (float)imu.az) * 180.0f / PI;
}

static float gyroYDegPerSec(const ImuRaw& imu) {
  return (float)imu.gy / 131.0f;
}

static void updateTilt(uint32_t nowMicros) {
  static bool kalmanReady = false;
  static uint32_t lastMicros = 0;

  ImuRaw imu;
  if (!readRawImu(imu)) {
    portENTER_CRITICAL(&stateMux);
    state.imuOk = false;
    portEXIT_CRITICAL(&stateMux);
    return;
  }

  const float accelY = accelAngleYDeg(imu);
  const float rawGyroY = gyroYDegPerSec(imu);
  float angleY = accelY;
  float filteredGyroY = 0.0f;

  if (!kalmanReady || lastMicros == 0) {
    kalmanY.setAngle(accelY);
    kalmanReady = true;
  } else {
    float dt = (nowMicros - lastMicros) / 1000000.0f;
    if (dt <= 0.0f || dt > 0.2f) {
      dt = CONTROL_PERIOD_US / 1000000.0f;
    }
    angleY = kalmanY.getAngle(accelY, rawGyroY, dt);
    filteredGyroY = kalmanY.getRate();
  }

  lastMicros = nowMicros;

  portENTER_CRITICAL(&stateMux);
  state.angleY = angleY;
  state.gyroY = filteredGyroY;
  state.imuOk = true;
  portEXIT_CRITICAL(&stateMux);
}

static void configureMotor() {
  writeParamU8(IDX_RUN_MODE, 2);
  delay(10);
  writeParamFloat(IDX_LIMIT_CUR, DEFAULT_LIMIT_CUR);
  delay(2);
  writeParamFloat(IDX_ACC_RAD, DEFAULT_ACC_RAD);
  delay(2);
  writeParamFloat(IDX_SPD_REF, 0.0f);
}

static void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleCommand() {
  const bool hasKp = server.hasArg("kp");
  const bool hasKd = server.hasArg("kd");
  const bool hasKw = server.hasArg("kw");
  const float kp = hasKp ? clampf(server.arg("kp").toFloat(), 0.0f, GAIN_LIMIT) : 0.0f;
  const float kd = hasKd ? clampf(server.arg("kd").toFloat(), 0.0f, GAIN_LIMIT) : 0.0f;
  const float kw = hasKw ? clampf(server.arg("kw").toFloat(), 0.0f, GAIN_LIMIT) : 0.0f;

  portENTER_CRITICAL(&stateMux);
  if (hasKp) {
    state.kp = kp;
  }
  if (hasKd) {
    state.kd = kd;
  }
  if (hasKw) {
    state.kw = kw;
  }
  portEXIT_CRITICAL(&stateMux);

  server.send(200, "application/json", "{\"ok\":true}");
}

static void handleState() {
  ControlState s;
  portENTER_CRITICAL(&stateMux);
  s = state;
  portEXIT_CRITICAL(&stateMux);

  char json[512];
  snprintf(
    json,
    sizeof(json),
    "{\"target\":%.3f,\"sent\":%.3f,\"wheelSpeed\":%.3f,\"angleY\":%.3f,\"gyroY\":%.3f,\"kp\":%.3f,\"kd\":%.3f,\"kw\":%.3f,\"indicatorColor\":\"%s\",\"enabled\":%s,\"canOk\":%s,\"imuOk\":%s,\"wheelOk\":%s,\"txOk\":%lu,\"txFail\":%lu,\"rxOk\":%lu,\"lastTxMs\":%lu,\"lastRxMs\":%lu}",
    s.targetSpeed,
    s.sentSpeed,
    s.wheelSpeed,
    s.angleY,
    s.gyroY,
    s.kp,
    s.kd,
    s.kw,
    indicatorColorHex(s.angleY),
    s.enabled ? "true" : "false",
    s.canOk ? "true" : "false",
    s.imuOk ? "true" : "false",
    s.wheelOk ? "true" : "false",
    (unsigned long)s.txOk,
    (unsigned long)s.txFail,
    (unsigned long)s.rxOk,
    (unsigned long)s.lastTxMs,
    (unsigned long)s.lastRxMs
  );
  server.send(200, "application/json", json);
}

static void webTask(void*) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/cmd", HTTP_GET, handleCommand);
  server.on("/api/state", HTTP_GET, handleState);
  server.begin();

  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static void controlTask(void*) {
  vTaskDelay(pdMS_TO_TICKS(MOTOR_CONFIG_DELAY_MS));
  configureMotor();

  uint32_t next = micros();
  uint32_t lastLedMs = 0;
  bool wasEnabled = false;

  for (;;) {
    const uint32_t nowUs = micros();
    updateTilt(nowUs);

    float angleY;
    float gyroY;
    float wheelSpeed;
    float kp;
    float kd;
    float kw;
    bool imuOk;
    bool wheelOk;
    bool enabled;
    portENTER_CRITICAL(&stateMux);
    angleY = state.angleY;
    gyroY = state.gyroY;
    wheelSpeed = state.wheelSpeed;
    kp = state.kp;
    kd = state.kd;
    kw = state.kw;
    imuOk = state.imuOk;
    if (!imuOk || fabsf(angleY) >= MOTOR_DISABLE_ANGLE_DEG) {
      enabled = false;
    } else if (fabsf(angleY) <= MOTOR_ENABLE_ANGLE_DEG) {
      enabled = true;
    } else {
      enabled = state.enabled;
    }
    state.enabled = enabled;
    portEXIT_CRITICAL(&stateMux);

    if (enabled && !wasEnabled) {
      sendRun(COMM_RUN_EN);
    } else if (!enabled && wasEnabled) {
      writeParamFloat(IDX_SPD_REF, 0.0f);
      sendRun(COMM_RUN_DIS);
    }
    wasEnabled = enabled;

    updateWheelFeedback();

    if (requestWheelSpeed()) {
      portENTER_CRITICAL(&stateMux);
      state.canOk = true;
      portEXIT_CRITICAL(&stateMux);
    }

    const uint32_t nowMsForWheel = millis();
    portENTER_CRITICAL(&stateMux);
    wheelSpeed = state.wheelSpeed;
    if (state.lastRxMs == 0 || nowMsForWheel - state.lastRxMs > 100) {
      state.wheelOk = false;
    }
    wheelOk = state.wheelOk;
    portEXIT_CRITICAL(&stateMux);

    const float wheelTerm = wheelOk ? wheelSpeed : 0.0f;
    const float feedback = kp * angleY + kd * gyroY - kw * wheelTerm;
    const float target = enabled ? clampf(-feedback, -SPEED_LIMIT, SPEED_LIMIT) : 0.0f;
    const float cmd = target;
    const bool ok = writeParamFloat(IDX_SPD_REF, cmd);

    portENTER_CRITICAL(&stateMux);
    state.targetSpeed = target;
    state.sentSpeed = cmd;
    state.canOk = ok;
    if (ok) {
      state.txOk++;
      state.lastTxMs = millis();
    } else {
      state.txFail++;
    }
    portEXIT_CRITICAL(&stateMux);

    const uint32_t nowMs = millis();
    if (nowMs - lastLedMs >= LED_PERIOD_MS) {
      ControlState s;
      portENTER_CRITICAL(&stateMux);
      s = state;
      portEXIT_CRITICAL(&stateMux);
      drawTiltIndicator(s.angleY, s.imuOk);
      lastLedMs = nowMs;
    }

    next += CONTROL_PERIOD_US;
    const int32_t waitUs = (int32_t)(next - micros());
    if (waitUs > 0) {
      delayMicroseconds((uint32_t)waitUs);
    } else {
      next = micros();
      taskYIELD();
    }
  }
}

void setup() {
  Serial.begin(115200);
  M5.begin(true, false, true);
  M5.dis.setBrightness(20);
  Wire.begin(25, 21);
  M5.IMU.Init();
  configureImuRange();
  setMatrixColor(CRGB(0x14, 0x14, 0x00));

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    setMatrixColor(CRGB(0xff, 0x00, 0x00));
    Serial.println("TWAI driver install failed");
    return;
  }
  if (twai_start() != ESP_OK) {
    setMatrixColor(CRGB(0xff, 0x00, 0x00));
    Serial.println("TWAI start failed");
    return;
  }

  portENTER_CRITICAL(&stateMux);
  state.targetSpeed = 0.0f;
  state.sentSpeed = 0.0f;
  state.wheelSpeed = 0.0f;
  state.angleY = 0.0f;
  state.gyroY = 0.0f;
  state.kp = 0.8f;
  state.kd = 0.3f;
  state.kw = 0.8f;
  state.enabled = false;
  state.canOk = true;
  state.imuOk = false;
  state.wheelOk = false;
  portEXIT_CRITICAL(&stateMux);

  xTaskCreatePinnedToCore(webTask, "web", 6144, nullptr, 1, &webTaskHandle, 0);
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 3, &controlTaskHandle, 1);

  Serial.println("Connect WiFi: ATOM-RS05-Tilt / atom1234");
  Serial.println("Open: http://192.168.4.1/");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "DFRobot_AXP313A.h"
#include "model_data.h"
#include <Arduino_JSON.h>
#include <Preferences.h>
#include <time.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

#define CONFIG_PORTAL_TIMEOUT 180

#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3  // Has PSRAM
#include "camera_pins.h"

// ─── TFLite Micro includes ────────────────────────────────────────────────────
#include <tflm_esp32.h>
#include <eloquent_tinyml.h>

// ─── Model specifications ─────────────────────────────────────────────────────
#define INPUT_WIDTH      96
#define INPUT_HEIGHT     96
#define NUMBER_OF_INPUTS (INPUT_WIDTH * INPUT_HEIGHT)  // 9216 — grayscale pixels
#define NUMBER_OF_OUTPUTS 1                        // 0 = No-Fall, 1 = Fall
#define ARENA_SIZE       (100 * 1024)                  // bytes — increase if inference crashes

// ─── Detection thresholds ─────────────────────────────────────────────────────
#define FALL_CONFIDENCE_THRESHOLD 0.85f  // Minimum confidence to trigger alert
#define INFERENCE_INTERVAL_MS     2000   // Run inference every 2 s

// ─── Debug stream toggle ──────────────────────────────────────────────────────
// Camera can only be in one pixel format at a time. Grayscale is required for
// inference, so the JPEG debug stream is disabled by default. Flip to true,
// reflash, and check :81/stream to verify camera framing — then flip back.
#define DEBUG_STREAM_ENABLED false

// find the ip by running 'ipconfig' on terminal
// ensure server is running on the same network as the ESP (mobile hotspot works)
String SERVER_REGISTER_URL = "https://server-fall-detection-app.onrender.com/api/v1/devices";
String SERVER_ALERT_URL    = "https://server-fall-detection-app.onrender.com/api/v1/events";

Preferences prefs;
String apiKey   = "";
String deviceId = "";

// ─── Globals ──────────────────────────────────────────────────────────────────
DFRobot_AXP313A axp;
Eloquent::TF::Sequential<10, ARENA_SIZE> tf;

static float    input_buffer[NUMBER_OF_INPUTS];
static uint32_t last_inference_ms = 0;

void startCameraServer();
void setupLedFlash(int pin);
void runInference();
bool captureAndPreprocess();
void setupTime();
void setupApiKey(const char* userEmail);
String registerDevice(const char* userEmail);
bool sendFallAlert();
String getDeviceId();

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  deviceId = getDeviceId();
  Serial.println("Device ID: " + deviceId);

  // ── Power management (must happen BEFORE esp_camera_init) ──────────────────
  while (axp.begin() != 0) {
    Serial.println("AXP313A init failed, retrying...");
    delay(1000);
  }
  axp.enableCameraPower(axp.eOV2640);

  // ── Camera configuration ───────────────────────────────────────────────────
  camera_config_t config;
  config.ledc_channel  = LEDC_CHANNEL_0;
  config.ledc_timer    = LEDC_TIMER_0;
  config.pin_d0        = Y2_GPIO_NUM;
  config.pin_d1        = Y3_GPIO_NUM;
  config.pin_d2        = Y4_GPIO_NUM;
  config.pin_d3        = Y5_GPIO_NUM;
  config.pin_d4        = Y6_GPIO_NUM;
  config.pin_d5        = Y7_GPIO_NUM;
  config.pin_d6        = Y8_GPIO_NUM;
  config.pin_d7        = Y9_GPIO_NUM;
  config.pin_xclk      = XCLK_GPIO_NUM;
  config.pin_pclk      = PCLK_GPIO_NUM;
  config.pin_vsync     = VSYNC_GPIO_NUM;
  config.pin_href      = HREF_GPIO_NUM;
  config.pin_sccb_sda  = SIOD_GPIO_NUM;
  config.pin_sccb_scl  = SIOC_GPIO_NUM;
  config.pin_pwdn      = PWDN_GPIO_NUM;
  config.pin_reset     = RESET_GPIO_NUM;
  config.xclk_freq_hz  = 20000000;
  config.grab_mode     = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location   = CAMERA_FB_IN_PSRAM;
  config.fb_count      = 1;

  // Camera runs in ONE format for the whole session — grayscale for inference,
  // or JPEG if you're temporarily debugging the live stream (see DEBUG_STREAM_ENABLED).
  if (DEBUG_STREAM_ENABLED) {
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
  } else {
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size   = FRAMESIZE_96X96;
  }

  if (psramFound()) {
    if (DEBUG_STREAM_ENABLED) {
      config.jpeg_quality = 10;
    }
    config.fb_count  = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  // ── WiFi ───────────────────────────────────────────────────────────────────
  WiFiManager wm;
  WiFiManagerParameter custom_email("email", "your email", "mari@gmail.com", 254);
  wm.addParameter(&custom_email);
  Serial.println("Stored SSID: " + wm.getWiFiSSID());
  Serial.println("Stored pass length: " + String(wm.getWiFiPass().length()));
  wm.setConnectTimeout(20);
  wm.setDebugOutput(true);
  wm.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT);

  bool res = wm.autoConnect("AutoConnectAP", "password");

  if (!res) {
    Serial.println("Failed to connect");
    delay(1000);
    ESP.restart();
  }

  Serial.println("WiFi connected — IP: " + WiFi.localIP().toString());

  const char* userEmail = custom_email.getValue();
  Serial.print("User email: ");
  Serial.println(userEmail);

  setupTime();              // sync clock for reliable timestamps
  setupApiKey(userEmail);   // load or register API key

  // ── TFLite model init ──────────────────────────────────────────────────────
  while (!tf.begin(fall_detection_model_tflite).isOk()) {
    Serial.println("TFLite model load failed: " + tf.exception.toString());
    delay(1000);
  }
  Serial.println("TFLite model loaded successfully");

  // ── Camera web server (debug only — see DEBUG_STREAM_ENABLED) ─────────────
  if (DEBUG_STREAM_ENABLED) {
    startCameraServer();
    Serial.print("Camera stream: http://");
    Serial.print(WiFi.localIP());
    Serial.println(":81/stream");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  if (now - last_inference_ms >= INFERENCE_INTERVAL_MS) {
    last_inference_ms = now;
    runInference();
  }

  // Keep WiFi alive / handle reconnects
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected — attempting reconnect...");
    WiFi.reconnect();
  }

  delay(10);
}

// ─────────────────────────────────────────────────────────────────────────────
// Capture a GRAYSCALE frame, normalise to [0,1], fill buffer
// ─────────────────────────────────────────────────────────────────────────────
bool captureAndPreprocess() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }

  if (fb->format != PIXFORMAT_GRAYSCALE) {
    esp_camera_fb_return(fb);
    Serial.println("Warning: frame not grayscale — check pixel_format config");
    return false;
  }

  if (fb->len < (size_t)NUMBER_OF_INPUTS) {
    esp_camera_fb_return(fb);
    Serial.println("Frame too small for model input");
    return false;
  }

  // Normalise uint8 [0,255] → float [0.0, 1.0]
  for (int i = 0; i < NUMBER_OF_INPUTS; i++) {
    input_buffer[i] = fb->buf[i] / 255.0f;
  }

  esp_camera_fb_return(fb);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Run TFLite inference and trigger alert if fall detected
// ─────────────────────────────────────────────────────────────────────────────
void runInference() {
  if (!captureAndPreprocess()) return;

  if (!tf.predict(input_buffer).isOk()) {
    Serial.println("Inference error: " + tf.exception.toString());
    return;
  }

  float prob_fall = tf.output(0);  // single sigmoid output: probability of "fall"

  Serial.printf("[Inference] Fall probability: %.2f\n", prob_fall);

  if (prob_fall >= FALL_CONFIDENCE_THRESHOLD) {
    Serial.println(">>> FALL DETECTED — sending alert");
    sendFallAlert();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Syncs internal clock for reliable timestamps
// ─────────────────────────────────────────────────────────────────────────────
void setupTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");  // UTC, no DST

  Serial.print("Waiting for NTP time sync");
  time_t now = time(nullptr);
  while (now < 1700000000) {  // arbitrary "sane" epoch threshold (Nov 2023+)
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println();
  Serial.println("Time synced: " + String(now));
}

// ─────────────────────────────────────────────────────────────────────────────
// Returns a stable hex string for this device's unique MAC-derived ID
// ─────────────────────────────────────────────────────────────────────────────
String getDeviceId() {
  uint64_t mac = ESP.getEfuseMac();
  char idStr[17];
  // Format as a 12-character hex MAC string (consistent across all uses)
  snprintf(idStr, sizeof(idStr), "%04X%08X",
           (uint16_t)(mac >> 32), (uint32_t)mac);
  return String(idStr);
}

// ─────────────────────────────────────────────────────────────────────────────
// Retrieve saved API key (gets it from server if not saved)
// ─────────────────────────────────────────────────────────────────────────────
void setupApiKey(const char* userEmail) {
  prefs.begin("device", false);  // namespace "device", read-write mode
  apiKey = prefs.getString("api_key", "");  // "" = default if not found

  if (apiKey == "") {
    Serial.println("No API key found — registering with server...");
    apiKey = registerDevice(userEmail);

    if (apiKey != "") {
      prefs.putString("api_key", apiKey);
      Serial.println("API key saved: " + apiKey);
    } else {
      Serial.println("Registration failed; will retry next boot");
    }
  } else {
    Serial.println("Loaded existing API key: " + apiKey);
  }

  prefs.end();
}

// ─────────────────────────────────────────────────────────────────────────────
// Registers ESP32 in the server and returns its unique API key
// ─────────────────────────────────────────────────────────────────────────────
String registerDevice(const char* userEmail) {
  HTTPClient http;
  http.begin(SERVER_REGISTER_URL);
  http.addHeader("Content-Type", "application/json");

  JSONVar payloadObj;
  payloadObj["deviceId"]  = deviceId;
  payloadObj["userEmail"] = userEmail;
  String payload = JSON.stringify(payloadObj);

  int httpCode = http.POST(payload);
  String result = "";

  if (httpCode == 201 || httpCode == 409) {  // created or already exists
    String response = http.getString();
    Serial.println(response);
    JSONVar doc = JSON.parse(response);

    if (JSON.typeof(doc) == "undefined") {
      Serial.println("JSON parse failed");
    } else {
      result = String((const char*) doc["apiKey"]);
    }
  } else {
    Serial.printf("Registration HTTP error: %d\n", httpCode);
  }

  http.end();
  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Send HTTP POST alert to server
// ─────────────────────────────────────────────────────────────────────────────
bool sendFallAlert() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi is not connected.");
    return false;
  }

  if (apiKey == "") {
    Serial.println("No API key available.");
    return false;
  }

  JSONVar payloadObj;
  payloadObj["deviceId"]   = deviceId;
  payloadObj["timestamp"]  = (long) time(nullptr);
  String payload = JSON.stringify(payloadObj);

  HTTPClient http;
  http.begin(SERVER_ALERT_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-api-key", apiKey);

  int httpCode = http.POST(payload);
  bool success = false;

  if (httpCode == 201) {
    String response = http.getString();
    Serial.println(response);
    success = true;
  } else {
    String response = http.getString();
    Serial.printf("Alert failed — HTTP %d\n", httpCode);
    Serial.println(response);
  }

  http.end();
  return success;
}

#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "DFRobot_AXP313A.h"
#include <Arduino_JSON.h>
#include <Preferences.h>
#include <time.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

#define CONFIG_PORTAL_TIMEOUT 0

#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3  // Has PSRAM
#include "camera_pins.h"

// Server-side CNN+LSTM plan:
// The ESP32-S3 captures 160x160 color frames, JPEG-encodes them, and uploads them.
// The server should keep a rolling frame buffer per device and run the fall model.
#define FRAME_UPLOAD_INTERVAL_MS 500
#define FRAME_WIDTH 160
#define FRAME_HEIGHT 160
#define FRAME_JPEG_QUALITY 80
#define FRAME_UPLOAD_ERROR_BACKOFF_MS 5000

#define DEBUG_STREAM_ENABLED false

String SERVER_REGISTER_URL = "https://server-fall-detection-app-w7lr.onrender.com/api/v1/devices";
String SERVER_FRAME_URL    = "https://server-fall-detection-app-w7lr.onrender.com/api/v1/frames";

Preferences prefs;
String apiKey   = "";
String deviceId = "";

DFRobot_AXP313A axp;
static uint32_t last_frame_upload_ms = 0;
static uint32_t frame_upload_backoff_until_ms = 0;
static uint32_t frame_sequence = 0;

void startCameraServer();
void setupLedFlash(int pin);
void setupTime();
void setupApiKey(const char* userEmail);
String registerDevice(const char* userEmail);
bool uploadFrame();
String getDeviceId();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  deviceId = getDeviceId();
  Serial.println("Device ID: " + deviceId);

  while (axp.begin() != 0) {
    Serial.println("AXP313A init failed, retrying...");
    delay(1000);
  }
  axp.enableCameraPower(axp.eOV2640);

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
  config.pixel_format  = PIXFORMAT_RGB565;
  config.frame_size    = FRAMESIZE_160X160;
  config.jpeg_quality  = 12;
  config.grab_mode     = CAMERA_GRAB_LATEST;
  config.fb_location   = CAMERA_FB_IN_PSRAM;
  config.fb_count      = 2;

  if (!psramFound()) {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.fb_count = 1;
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

// WIFI CONNECTION -------------------------------------------------------------------

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFiManager wm;
  WiFiManagerParameter custom_email("email", "your email", "mari@gmail.com", 254);
  wm.addParameter(&custom_email);
  Serial.println("Stored SSID: " + wm.getWiFiSSID());
  Serial.println("Stored pass length: " + String(wm.getWiFiPass().length()));
  wm.setConnectTimeout(20);
  wm.setDebugOutput(true);
  wm.setConfigPortalTimeout(CONFIG_PORTAL_TIMEOUT);

  Serial.println("Starting WiFi setup portal if no saved network connects.");
  Serial.println("Setup AP: BYOP-Camera-Setup");
  Serial.println("Setup page: http://192.168.4.1");
  bool res = wm.autoConnect("BYOP-Camera-Setup");

  if (!res) {
    Serial.println("Failed to connect");
    Serial.println("WiFi setup portal closed without credentials. Restart the board to try again.");
    while (true) delay(1000);
  }

  Serial.println("WiFi connected. IP: " + WiFi.localIP().toString());

  const char* userEmail = custom_email.getValue();
  Serial.print("User email: ");
  Serial.println(userEmail);

  setupTime();
  setupApiKey(userEmail);

//-----------------------------------------------------------------------------------------------

  if (DEBUG_STREAM_ENABLED) {
    startCameraServer();
    Serial.print("Camera stream: http://");
    Serial.print(WiFi.localIP());
    Serial.println(":81/stream");
  }

  Serial.printf("Uploading JPEG frames to server every %d ms\n", FRAME_UPLOAD_INTERVAL_MS);
}

void loop() {
  uint32_t now = millis();

  if (now >= frame_upload_backoff_until_ms &&
      now - last_frame_upload_ms >= FRAME_UPLOAD_INTERVAL_MS) {
    last_frame_upload_ms = now;
    uploadFrame();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting reconnect...");
    WiFi.reconnect();
  }

  delay(10);
}

bool uploadFrame() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Skipping frame upload: WiFi is not connected.");
    return false;
  }

  if (apiKey == "") {
    Serial.println("Skipping frame upload: no API key available.");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }

  if (fb->format != PIXFORMAT_RGB565) {
    esp_camera_fb_return(fb);
    Serial.println("Frame is not RGB565; check camera pixel_format config.");
    return false;
  }

  uint8_t *jpg_buf = nullptr;
  size_t jpg_len = 0;
  bool converted = fmt2jpg(fb->buf, fb->len, FRAME_WIDTH, FRAME_HEIGHT,
                           PIXFORMAT_RGB565, FRAME_JPEG_QUALITY,
                           &jpg_buf, &jpg_len);

  esp_camera_fb_return(fb);

  if (!converted || !jpg_buf || jpg_len == 0) {
    Serial.println("JPEG conversion failed.");
    if (jpg_buf) free(jpg_buf);
    return false;
  }

  uint32_t sequence = frame_sequence++;
  time_t timestamp = time(nullptr);

  HTTPClient http;
  http.setTimeout(5000);
  http.begin(SERVER_FRAME_URL);
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("x-api-key", apiKey);
  http.addHeader("x-device-id", deviceId);
  http.addHeader("x-frame-sequence", String(sequence));
  http.addHeader("x-frame-timestamp", String((long)timestamp));

  int httpCode = http.POST(jpg_buf, jpg_len);
  bool success = (httpCode >= 200 && httpCode < 300);

  if (success) {
    Serial.printf("Uploaded frame %lu (%u bytes)\n", (unsigned long)sequence, jpg_len);
    frame_upload_backoff_until_ms = 0;
  } else if (httpCode == 404) {
    Serial.println("Frame upload failed. HTTP 404");
    Serial.println("The ESP32 is connected, but the server does not have POST /api/v1/frames.");
    Serial.println("Add that backend route or change SERVER_FRAME_URL to an existing frame endpoint.");
    frame_upload_backoff_until_ms = millis() + FRAME_UPLOAD_ERROR_BACKOFF_MS;
  } else {
    String response = http.getString();
    Serial.printf("Frame upload failed. HTTP %d\n", httpCode);
    Serial.println(response);
    frame_upload_backoff_until_ms = millis() + FRAME_UPLOAD_ERROR_BACKOFF_MS;
  }

  http.end();
  free(jpg_buf);
  return success;
}

void setupTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  Serial.print("Waiting for NTP time sync");
  time_t now = time(nullptr);
  while (now < 1700000000) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println();
  Serial.println("Time synced: " + String(now));
}

String getDeviceId() {
  uint64_t mac = ESP.getEfuseMac();
  char idStr[17];
  snprintf(idStr, sizeof(idStr), "%04X%08X",
           (uint16_t)(mac >> 32), (uint32_t)mac);
  return String(idStr);
}

void setupApiKey(const char* userEmail) {
  prefs.begin("device", false);
  apiKey = prefs.getString("api_key", "");

  if (apiKey == "") {
    Serial.println("No API key found. Registering with server...");
    apiKey = registerDevice(userEmail);

    if (apiKey != "") {
      prefs.putString("api_key", apiKey);
      Serial.println("API key saved.");
    } else {
      Serial.println("Registration failed; will retry next boot.");
    }
  } else {
    Serial.println("Loaded existing API key.");
  }

  prefs.end();
}

String registerDevice(const char* userEmail) {
  HTTPClient http;
  http.setTimeout(10000);
  http.begin(SERVER_REGISTER_URL);
  http.addHeader("Content-Type", "application/json");

  JSONVar payloadObj;
  payloadObj["deviceId"]  = deviceId;
  payloadObj["userEmail"] = userEmail;
  String payload = JSON.stringify(payloadObj);

  int httpCode = http.POST(payload);
  String result = "";

  if (httpCode == 201 || httpCode == 409) {
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
    Serial.println(http.getString());
  }

  http.end();
  return result;
}

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "DFRobot_AXP313A.h"
#include "model_data.h"

#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3  // Has PSRAM
#include "camera_pins.h"

// ─── TFLite Micro includes ────────────────────────────────────────────────────
#include <tflm_esp32.h>
#include <eloquent_tinyml.h>

// ─── Model specifications ─────────────────────────────────────────────────────
#define INPUT_WIDTH      96
#define INPUT_HEIGHT     96
#define NUMBER_OF_INPUTS (INPUT_WIDTH * INPUT_HEIGHT)  // 9216 — grayscale pixels
#define NUMBER_OF_OUTPUTS 2                            // 0 = No-Fall, 1 = Fall
#define ARENA_SIZE       64                            // KB — increase if inference crashes

// ─── Detection thresholds ─────────────────────────────────────────────────────
#define FALL_CONFIDENCE_THRESHOLD 0.85f  // Minimum confidence to trigger alert
#define INFERENCE_INTERVAL_MS     500    // Run inference every 500 ms

// ─── Android app endpoint — change to your phone's IP on the same WiFi ────────
// If using a backend/server, replace with your server URL
#define ANDROID_ALERT_URL "http://192.168.x.x:8080/fall-alert"

// ─── WiFi credentials ─────────────────────────────────────────────────────────
const char *ssid     = "namie";
const char *password = "mariguima";

// ─── Globals ──────────────────────────────────────────────────────────────────
DFRobot_AXP313A axp;
Eloquent::TF::Sequential<10, ARENA_SIZE> tf;

static float    input_buffer[NUMBER_OF_INPUTS];
static uint32_t last_inference_ms = 0;

void startCameraServer();
void setupLedFlash(int pin);
void runInference();
bool captureAndPreprocess();
void sendFallAlert(float confidence);

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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

  // Use GRAYSCALE for inference (matches 96x96 model input)
  // The camera server will still serve JPEG — we switch format per-task below
  config.pixel_format  = PIXFORMAT_JPEG;
  config.frame_size    = FRAMESIZE_UXGA;
  config.jpeg_quality  = 12;

  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count     = 2;
    config.grab_mode    = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size  = FRAMESIZE_SVGA;
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
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
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
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected — IP: " + WiFi.localIP().toString());

  // ── TFLite model init ──────────────────────────────────────────────────────
  while (!tf.begin(fall_detection_model_tflite).isOk()) {
    Serial.println("TFLite model load failed: " + tf.exception.toString());
    delay(1000);
  }
  Serial.println("TFLite model loaded successfully");

  // ── Camera web server (runs in its own FreeRTOS task) ─────────────────────
  startCameraServer();
  Serial.print("Camera stream: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/stream");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  if (now - last_inference_ms >= INFERENCE_INTERVAL_MS) {
    last_inference_ms = now;
    runInference();
  }

  delay(10);
}

// ─────────────────────────────────────────────────────────────────────────────
// Capture a GRAYSCALE frame, resize to 96×96, normalise to [0,1], fill buffer
// ─────────────────────────────────────────────────────────────────────────────
bool captureAndPreprocess() {
  // Temporarily switch to GRAYSCALE for inference capture
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_96X96);  // 96×96 native — no software resize needed

  // Re-init pixel format to GRAYSCALE just for this capture
  // (The web server uses JPEG; we restore below)
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }

  if (fb->format != PIXFORMAT_GRAYSCALE) {
    // If the frame isn't grayscale (e.g. still JPEG), skip this cycle
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

  float prob_no_fall = tf.output(0);
  float prob_fall    = tf.output(1);

  Serial.printf("[Inference] No-Fall: %.2f  |  Fall: %.2f\n", prob_no_fall, prob_fall);

  if (prob_fall >= FALL_CONFIDENCE_THRESHOLD) {
    Serial.println(">>> FALL DETECTED — sending alert");
    sendFallAlert(prob_fall);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Send HTTP POST alert to Android app
// ─────────────────────────────────────────────────────────────────────────────
void sendFallAlert(float confidence) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected — alert not sent");
    return;
  }

  HTTPClient http;
  http.begin(ANDROID_ALERT_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(3000);  // 3 s timeout — don't block inference loop

  // Build JSON payload
  char payload[128];
  snprintf(payload, sizeof(payload),
           "{\"event\":\"fall\",\"confidence\":%.2f,\"device\":\"esp32s3\"}",
           confidence);

  int httpCode = http.POST(payload);

  if (httpCode > 0) {
    Serial.printf("Alert sent — HTTP %d\n", httpCode);
  } else {
    Serial.printf("Alert failed — error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}

#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include "esp_camera.h"

// WiFi credentials
const char* ssid = "Galaxy M31FECA";
const char* password = "mgachamp";

// Server URL
const char* serverURL = "https://383a-34-74-88-123.ngrok-free.app/";
// GPS setuphttp://192.168.148.93:5000;http://172.28.0.12:5000
TinyGPSPlus gps;
HardwareSerial GPSserial(1);

// Ultrasonic Sensor pins
#define TRIG_PIN 12
#define ECHO_PIN 13

#define GPS_RX_PIN 2   // RX for Neo6M GPS
#define GPS_TX_PIN 4 
#define CAMERA_MODEL_AI_THINKER 
#include "camera_pins.h"



void setup() {
  Serial.begin(115200);

  // WiFi setup
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");

  // GPS setup
  GPSserial.begin(9600, SERIAL_8N1, 4, 2);

  // Ultrasonic sensor setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Camera setup
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }
}
              // Use a single frame buffer


void loop() {
  // Get distance
  long duration, distanceCm;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distanceCm = duration * 0.034 / 2;

  // Read GPS data
  while (GPSserial.available() > 0) {
    gps.encode(GPSserial.read());
  }
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();

  // Capture camera image
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Initialize HTTP client
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverURL);
    http.addHeader("Content-Type", "application/json");

    // Create JSON payload
    String jsonPayload;
    jsonPayload += "{";
    jsonPayload += "\"latitude\": " + String(latitude, 6) + ",";
    jsonPayload += "\"longitude\": " + String(longitude, 6) + ",";
    jsonPayload += "\"distance\": " + String(distanceCm) + ",";
    jsonPayload += "\"image\": \"";

    // Encode image in Base64
    for (int i = 0; i < fb->len; i++) {
      char base64Chars[3];
      sprintf(base64Chars, "%02x", fb->buf[i]);
      jsonPayload += String(base64Chars);
    }

    jsonPayload += "\"}";

    // Send POST request
    int httpResponseCode = http.POST(jsonPayload);
    if (httpResponseCode > 0) {
      Serial.printf("Data sent successfully, code: %d\n", httpResponseCode);
    } else {
      Serial.printf("Failed to send data, error: %s\n", http.errorToString(httpResponseCode).c_str());
    }

    // End connection
    http.end();
  }

  // Release camera frame buffer
  esp_camera_fb_return(fb);

  delay(5000);  // Delay before sending the next data
}

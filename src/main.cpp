#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include <SPIFFS.h>
#include <base64.h>

#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define FILE_PHOTO "/photo.jpg"
#define pressButton 2

#include "camera_pins.h"

// const char* ssid = "estacionaqui_pi";
// const char* password = "modular123";
const char* ssid = "Avelino-2.4G";
const char* password = "avelino1461";
const char* camId = "sbc-golden-001";

// const char* serverName = "http://192.168.0.10:5000/api/imgdata";
const char* serverName = "http://192.168.15.160:5000/api/imgdata";

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;
unsigned int spotNumber = 0;

WiFiClient wifiClient; // do the WiFi instantiation thing

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  Serial.println("init setup");
  pinMode(pressButton, INPUT);

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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  WiFi.begin(ssid, password);
  Serial.println("Connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Print ESP32 Local IP Address
  Serial.print("IP Address: http://");
  Serial.println(WiFi.localIP());

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }

}

bool checkPhoto( fs::FS &fs ) {
  File f_pic = fs.open( FILE_PHOTO );
  unsigned int pic_sz = f_pic.size();
  return ( pic_sz > 100 );
}

String capturePhotoSaveSpiffs() {
  camera_fb_t * pic = NULL; // pointer
  bool ok = 0; // Boolean indicating if the picture has been taken correctly
  String encoded = "";

  do {
    // Take a photo with the camera
    Serial.println("Taking a photo...");

    pic = esp_camera_fb_get();
    if (!pic) {
      Serial.println("Camera capture failed");
      continue;
    }

    size_t pic_len;
    pic_len = pic->len;

    encoded = base64::encode(pic->buf, pic_len);

    esp_camera_fb_return(pic);

    if (encoded.length() > 0) {
      ok = 1;
    }
  } while ( !ok );

  return encoded;
}

void loop() {
  if ((millis() - lastTime) > timerDelay) {
    //Check WiFi connection status
    if(WiFi.status() == WL_CONNECTED) {
      WiFiClient client;
      HTTPClient http;

      while(digitalRead(pressButton)==HIGH){
        delay(100);

        if(digitalRead(pressButton)==HIGH){
          String img_base64 = capturePhotoSaveSpiffs();
          spotNumber = 1;

          // Your Domain name with URL path or IP address with path
          http.begin(client, serverName);

          http.addHeader("Content-Type", "application/json");
          String httpMessage = String("{\"img_data\":\"") + img_base64 + String("\",") +
          String("\"cam_id\":\"") + String(camId) + String("\",") +
          String("\"spot_number\":\"") + String(spotNumber) + String("\"}");

          Serial.println("sending message to server");

          int httpResponseCode = http.POST(httpMessage);

          Serial.println(httpResponseCode);
        }
      }
    }
  }
}

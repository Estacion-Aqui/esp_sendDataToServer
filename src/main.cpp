#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include <ultrasonic.h>
#include <SPIFFS.h>
#include <base64.h>
#include <PubSubClient.h>

#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define FILE_PHOTO "/photo.jpg"
#define BLUE_LED 12
#define GREEN_LED 13
#define RED_LED 15
#define FLASH_LED 4
#define TRIG_PIN 14
#define ECHO_PIN 2

#include "camera_pins.h"

// const char* ssid = "EstacionAquiWifi";
// const char* password = "modular123";
const char* ssid = "Avelino-2.4G";
const char* password = "avelino1461";
const char* camId = "sbc-golden-011";
const char* deviceId = "cam011";
const char* entityId = "urn:ngsi-ld:ParkingSpot:sbc:golden:011";

const char* serverAddress = "http://192.168.80.196:5000/api/imgdata";
const char* helixAddress = "34.151.219.62";
const int mqttPort = 1883;

boolean spotFree;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastLoopTime = 0;
unsigned long timerDelay = 2000;
unsigned int spotNumber = 1;
unsigned int sensorDistance = 400;

WiFiClient wifiClient; // do the WiFi instantiation client
PubSubClient mqttClient(wifiClient); // do the MQTT instantiation client
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);

void mqttConnect();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void helixCreateEntity();

void configInitCamera() {
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
  s->set_contrast(s, 0);       // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 300);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  } else {
    s->set_brightness(s, 0);     // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  }

  s->set_framesize(s, FRAMESIZE_QVGA);
}

void configInitPins() {
  pinMode(FLASH_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  spotFree = true;
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  digitalWrite(GREEN_LED, HIGH);

  Serial.println("Init setup");
  configInitPins();

  Serial.print("Initializing the camera module...");
  configInitCamera();
  Serial.println("Ok!");

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

  helixCreateEntity();
  mqttConnect();

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }

}

void helixCreateEntity() {
  HTTPClient http;

  Serial.println("Creating " + String(deviceId) + " entity...");

  String bodyRequest = "{\"devices\": [{" +
  String("\"device_id\": \"" + String(deviceId) + "\",") +
  String("\"entity_name\": \"" + String(entityId) + "\",") +
  String("\"entity_type\": \"ParkingSpot\",") +
  String("\"protocol\": \"PDI-IoTA-UltraLight\",") +
  String("\"transport\": \"MQTT\",") +
  String("\"commands\": [") +
  String("{\"name\": \"free\", \"type\": \"command\"}, {\"name\": \"reserved\", \"type\": \"command\"}, {\"name\": \"filled\", \"type\": \"command\"}") +
  String("],") +
  String("\"attributes\": [") +
  String("{\"object_id\": \"c\", \"name\": \"category\", \"type\": \"string\"},") +
  String("{\"object_id\": \"cp\", \"name\": \"current_plate\", \"type\": \"string\"},") +
  String("{\"object_id\": \"l\", \"name\": \"location\", \"type\": \"Point\"},") +
  String("{\"object_id\": \"n\", \"name\": \"name\", \"type\": \"string\"},") +
  String("{\"object_id\": \"s\", \"name\": \"status\", \"type\": \"string\"}") +
  String("]}]}");

  String helixUrl = "http://" + String(helixAddress) + ":4041/iot/devices";

  http.begin(wifiClient, helixUrl);

  http.addHeader("Content-Type", "application/json");
  http.addHeader("fiware-service", "helixiot");
  http.addHeader("fiware-servicepath", "/");

  int response = http.POST(bodyRequest);

  if (response < 0) {
    Serial.println("request error - " + response);
  }

  if (response != HTTP_CODE_OK) {
    Serial.println("Dispositivo criado com sucesso");
  } else {
    Serial.println("Falha na criacao do dispositivo");
  }

  http.end();
}

void mqttConnect() {
  String topic = String("/iot/" + String(deviceId) + "/cmd");

  Serial.println("Connecting to Helix....");

  mqttClient.setServer(helixAddress, mqttPort);
  mqttClient.setCallback(mqttCallback);

  while(!mqttClient.connected()) {
    if (mqttClient.connect("helix")) {
      Serial.println("Connected to Helix");
    } else {
      Serial.print("failed to connect: ");
      Serial.println(mqttClient.state());
    }
  }

  mqttClient.subscribe(topic.c_str());
}

void turnBlueLed() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
}

void turnOrangeLed() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
}

void turnRedLed() {
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
}

void turnGreenLed() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, LOW);
}

bool checkPhoto( fs::FS &fs ) {
  File f_pic = fs.open( FILE_PHOTO );
  unsigned int pic_sz = f_pic.size();
  return ( pic_sz > 100 );
}

String capturePhoto() {
  camera_fb_t * picture = NULL; // pointer to the photo
  bool picOk = 0;
  String encoded = "";

  do {
    Serial.println("Taking a photo...");

    digitalWrite(FLASH_LED, HIGH); // Turn on flash
    delay (1000);
    picture = esp_camera_fb_get(); // take the the picture and save in the buffer
    delay (1000);
    digitalWrite(FLASH_LED, LOW); // Turn off flash

    if (!picture) {
      Serial.println("Camera capture failed");
      continue;
    }

    size_t pic_length = picture->len;
    encoded = base64::encode(picture->buf, pic_length); // encode pic ion base64 format

    esp_camera_fb_return(picture); // clean the buffer for the camera reuse

    if (encoded.length() > 0) {
      picOk = 1;
    }
  } while ( !picOk );

  return encoded;
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
  payload[length] = '\0';
  String strMsg = String((char*) payload);

  int charPos = strMsg.lastIndexOf("@");
  String command = strMsg.substring(charPos + 1);
  command.replace("|", "");

  Serial.println("Received payload:");
  Serial.println(strMsg);
  Serial.println(command);
  Serial.println("..................");

  if (command == "reserved") {
    turnOrangeLed();
  } else if (command == "filled") {
    turnRedLed();
  } else if (command == "free") {
    turnGreenLed();
  }
}

boolean readSensor() {
  sensorDistance = ultrasonic.read();

  Serial.println("Distance in cm: " + String(sensorDistance));

  if (sensorDistance < 170) {
    delay(5000);

    unsigned int newSensorDistance = ultrasonic.read();
    delay(50);

    if (newSensorDistance >= sensorDistance - 2 && newSensorDistance <= sensorDistance + 2 ) {
      return true;
    }
  }

  return false;
}

int sendToRaspberry(String httpMessage) {
  HTTPClient http;
  int responseCode;
  int retries = 0;

  do {
    http.begin(wifiClient, serverAddress);

    http.addHeader("Content-Type", "application/json");

    responseCode = http.POST(httpMessage);

    Serial.println(responseCode);

    retries++;

    if(retries > 3)
      return 0;
  } while(responseCode != 200);

  return responseCode;
}

void sendPicture() {
  String img_base64 = capturePhoto();
  spotNumber = 1;

  String httpMessage = String("{\"img_data\":\"") + img_base64 + String("\",") +
      String("\"cam_id\":\"") + String(camId) + String("\",") +
      String("\"spot_status\":\"filled\",") +
      String("\"spot_number\":\"") + String(spotNumber) + String("\"}");

  Serial.println("sending picture to server");
  int responseCode = sendToRaspberry(httpMessage);

  if (responseCode == 200) {
    spotFree = false;
  }
}

void loop() {
  if(!mqttClient.connected()) {
    mqttConnect();
  }

  mqttClient.loop();

  if ((millis() - lastLoopTime) > timerDelay) {
    //Check WiFi connection status
    if(WiFi.status() == WL_CONNECTED) {
      boolean possuiCarro = readSensor();

      if(possuiCarro && spotFree) {
        turnBlueLed();

        sendPicture();
      } else if (!possuiCarro && !spotFree) {
        spotFree = true;

        String httpMessage = String("{\"img_data\":\"\",") +
          String("\"cam_id\":\"") + String(camId) + String("\",") +
          String("\"spot_status\":\"free\",") +
          String("\"spot_number\":\"") + String(spotNumber) + String("\"}");

        sendToRaspberry(httpMessage);
        Serial.println("spot is free");
      }
    }

    lastLoopTime = millis();
  }
}

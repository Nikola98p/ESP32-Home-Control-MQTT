#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include "DHT.h"
#include "creds.h"
#include "style.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RELAY_TOPIC "homeassistant/switch/ESP32-W1/state"
#define SWITCH_TOPIC "homeassistant/switch/ESP32-W1/set"
#define HA_STAT_TOPIC "homeassistant/status"
#define BRIGHTNESS_TOPIC "homeassistant/light/ESP32-W1/brightness/set"
#define POT_TOPIC "homeassistant/light/ESP32-W1/brightness/state"
#define TEMPERATURE_TOPIC "homeassistant/sensor/ESP32-W1/temperature"
#define HUMIDITY_TOPIC "homeassistant/sensor/ESP32-W1/humidity"
#define IP_ADDRESS_TOPIC "homeassistant/sensor/ESP32-W1/ESP_IP_Address/state"

#define DHTPIN 9
#define DHTTYPE DHT21  // AM2301

const int relay_pin = 13;          // Relay is connected to GPIO27
const int LED_PIN = 12;             // Analog control of pin
const int POTENTIOMETER_PIN = 10;  // Reads potentiometer value
const int button_pin = 18;         // Controls relay
const int NUM_SAMPLES = 10;        // Number of samples for averaging pot value

const unsigned long interval = 180000;                 // Interval between measurements in milliseconds
const unsigned long SwitchDebounceUpdateInterval = 1;  // Switch Debounce
const unsigned long PotUpdateInterval = 100;           // Pot value read delay
const unsigned long keep_alive = 840000;           // MQTT Keep alive function
unsigned long previousMillis = 0;
unsigned long buttonMillis = 0;
unsigned long potMillis = 0;
unsigned long keep_alive_milis = 0;

bool first_boot;
bool keep_msg_alive = false;
// unsigned long lastIPAddressUpdate = 0;

const char *host = "esp32_moja_soba";

//Setup states
int relayState = LOW;
// int ledState = LOW;
int lastButtonState = LOW;
int lastPotValue = 0;

WebServer server(80);

DHT dht(DHTPIN, DHTTYPE);
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

void KeepMeAlive(void *parameter) {
  while (true) {
    unsigned long currentMillis = millis();

    // Check if relay state has changed
    if (currentMillis - previousMillis >= 840000 ) { // 14 minutes
      previousMillis = currentMillis;

      keep_msg_alive = true;

      MqttHomeAssistantDiscovery();
      
      // Send relay state message
      String relayStateMessage = (relayState == HIGH) ? "ON" : "OFF";
      mqttClient.beginMessage(RELAY_TOPIC);
      mqttClient.print(relayStateMessage);
      mqttClient.endMessage();
    }

    // Check if brightness value has changed
    if (currentMillis - potMillis >= 840000 ) { // 14 minutes
      int potValue = averagePotValue();
      if (abs(potValue - lastPotValue) > 90) {
        potMillis = currentMillis;
        int brightnessValue = map(potValue, 0, 4095, 0, 255);
        mqttClient.beginMessage(BRIGHTNESS_TOPIC);
        mqttClient.print(String(brightnessValue));
        mqttClient.endMessage();
        lastPotValue = potValue;
      }
    }
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(relay_pin, OUTPUT);
  pinMode(button_pin, INPUT_PULLDOWN);
  digitalWrite(relay_pin, relayState);

  pinMode(LED_PIN, OUTPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);

  digitalWrite(relay_pin, LOW);

  first_boot = true;

  xTaskCreatePinnedToCore(
    connectToWiFi,   /* Function to implement the task */
    "ConnectToWiFi", /* Name of the task */
    10000,           /* Stack size in words */
    NULL,            /* Task input parameter */
    1,               /* Priority of the task */
    NULL,            /* Task handle. */
    0);              /* Core where the task should run */

  xTaskCreatePinnedToCore(
    connectToMQTTBroker,   /* Function to implement the task */
    "ConnectToMQTTBroker", /* Name of the task */
    10000,                 /* Stack size in words */
    NULL,                  /* Task input parameter */
    1,                     /* Priority of the task */
    NULL,                  /* Task handle. */
    0);                    /* Core where the task should run */

  xTaskCreatePinnedToCore(
    handleMQTTMessages,   /* Function to implement the task */
    "HandleMQTTMessages", /* Name of the task */
    10000,                /* Stack size in words */
    NULL,                 /* Task input parameter */
    1,                    /* Priority of the task */
    NULL,                 /* Task handle. */
    0);                   /* Core where the task should run */

  xTaskCreatePinnedToCore(
    handleWebServer,   /* Function to implement the task */
    "HandleWebServer", /* Name of the task */
    10000,             /* Stack size in words */
    NULL,              /* Task input parameter */
    1,                 /* Priority of the task */
    NULL,              /* Task handle. */
    0);                /* Core where the task should run */

  xTaskCreatePinnedToCore(
    readSensors,   /* Function to implement the task */
    "ReadSensors", /* Name of the task */
    10000,         /* Stack size in words */
    NULL,          /* Task input parameter */
    1,             /* Priority of the task */
    NULL,          /* Task handle. */
    0);            /* Core where the task should run */

  xTaskCreatePinnedToCore(
    KeepMeAlive,    /* Function to implement the task */
    "KeepMeAlive",  /* Name of the task */
    10000,                /* Stack size in words */
    NULL,                 /* Task input parameter */
    1,                    /* Priority of the task */
    NULL,                 /* Task handle. */
    0                     /* Core where the task should run */
  );
}

void loop() {
  // Empty because FreeRTOS tasks are used
}

void connectToWiFi(void *parameter) {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.print("Connected to the network, your IP: ");
  Serial.println(WiFi.localIP());

  vTaskDelete(NULL);
}

void connectToMQTTBroker(void *parameter) {
  mqttClient.setId("test123");
  mqttClient.setUsernamePassword(MQTT_USER, MQTT_PASS);

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(MQTT_HOST);

  while (!mqttClient.connect(MQTT_HOST, MQTT_PORT)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    delay(100);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  mqttClient.onMessage(onMqttMessage);

  Serial.print("Subscribing to topic: ");
  Serial.println(SWITCH_TOPIC);
  Serial.println(HA_STAT_TOPIC);
  Serial.println(BRIGHTNESS_TOPIC);
  Serial.println();

  mqttClient.subscribe(SWITCH_TOPIC);
  mqttClient.subscribe(HA_STAT_TOPIC);
  mqttClient.subscribe(BRIGHTNESS_TOPIC);

  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) {  //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on(
    "/update", HTTP_POST, []() {
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    },
    []() {
      HTTPUpload &upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {  //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        /* flashing firmware to ESP*/
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {  //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
      }
    });

  server.begin();

  dht.begin();

  MqttHomeAssistantDiscovery();  // Initial discovery

  vTaskDelete(NULL);
}

void handleMQTTMessages(void *parameter) {
  while (true) {
    mqttClient.poll();
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay 100ms
  }
}

void handleWebServer(void *parameter) {
  while (true) {
    server.handleClient();
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay 100ms
  }
}

void readSensors(void *parameter) {
  while (true) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval || keep_msg_alive == true) {
      previousMillis = currentMillis;
      float h = dht.readHumidity();
      float t = dht.readTemperature();

      // Measure temperature and humidity
      if (!isnan(t) && !isnan(h)) {
        // Update Home Assistant with the state attribute
        mqttClient.beginMessage(TEMPERATURE_TOPIC);
        mqttClient.print(t);
        mqttClient.endMessage();

        mqttClient.beginMessage(HUMIDITY_TOPIC);
        mqttClient.print(h);
        mqttClient.endMessage();

        keep_msg_alive = false;
      }
    }

    int buttonState = digitalRead(button_pin);
      if (buttonState != lastButtonState) {
        unsigned long currentMillis1 = millis();
        if (currentMillis1 - buttonMillis >= SwitchDebounceUpdateInterval) {
          buttonMillis = currentMillis1;
          if (buttonState != lastButtonState) {
            relayState = !relayState;
            digitalWrite(relay_pin, relayState);

            // Send the correct state to RELAY_TOPIC
            String relayStateMessage = (relayState == HIGH) ? "ON" : "OFF";
            mqttClient.beginMessage(RELAY_TOPIC);
            mqttClient.print(relayStateMessage);
            mqttClient.endMessage();

            Serial.println("Physical switch toggled");
          }
        }
      }

    lastButtonState = buttonState;

    int potValue = averagePotValue();
    if (abs(potValue - lastPotValue) > 90) {
      unsigned long currentMillis2 = millis();
      if (currentMillis2 - potMillis >= PotUpdateInterval) {
        potMillis = currentMillis2;
        int brightnessValue = map(potValue, 0, 4095, 0, 255);
        mqttClient.beginMessage(BRIGHTNESS_TOPIC);
        mqttClient.print(String(brightnessValue));
        mqttClient.endMessage();
        lastPotValue = potValue;
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay 100ms
  }
}

void onMqttMessage(int messageSize) {
  String receivedTopic = mqttClient.messageTopic();

  String message = mqttClient.readString();

  if (receivedTopic.equals(HA_STAT_TOPIC)) {
    if (message == "online") {
      MqttHomeAssistantDiscovery();
    }
  } else if (receivedTopic.equals(SWITCH_TOPIC)) {
    if (message == "ON") {
      relayState = HIGH;
      digitalWrite(relay_pin, relayState);

      // Update Home Assistant switch status
      mqttClient.beginMessage(RELAY_TOPIC);
      mqttClient.print("ON");
      mqttClient.endMessage();
    } else if (message == "OFF") {
      relayState = LOW;
      digitalWrite(relay_pin, relayState);

      // Update Home Assistant switch status
      mqttClient.beginMessage(RELAY_TOPIC);
      mqttClient.print("OFF");
      mqttClient.endMessage();
    }
  } else if (receivedTopic.equals(BRIGHTNESS_TOPIC)) {
    int brightnessValue = message.toInt();
    analogWrite(LED_PIN, brightnessValue);
  }
}

void MqttHomeAssistantDiscovery() {
  String discoveryTopic;
  String payload;

  if (mqttClient.connected()) {
    Serial.println("SEND HOME ASSISTANT DISCOVERY!!!");

    // JSON buffer and object
    StaticJsonDocument<256> jsonBuffer;
    JsonObject root = jsonBuffer.to<JsonObject>();

    // Discovery for relay control
    discoveryTopic = "homeassistant/switch/ESP32-W1/ESP_Switch/config";
    root["name"] = "Esp32 Relay";
    root["unique_id"] = "esp_relay";
    root["command_topic"] = SWITCH_TOPIC;
    root["state_topic"] = RELAY_TOPIC;
    root["icon"] = "mdi:globe-light";

    serializeJson(root, payload);
    mqttClient.beginMessage(discoveryTopic.c_str());
    mqttClient.print(payload.c_str());
    mqttClient.endMessage();

    // Discovery for LED control
    discoveryTopic = "homeassistant/light/ESP32-W1/ESP_Brightness/config";

    root["name"] = "Brightness light";
    root["unique_id"] = "brightness_light";
    root["state_topic"] = BRIGHTNESS_TOPIC;
    root["command_topic"] = POT_TOPIC;
    root["icon"] = "mdi:knob";

    serializeJson(root, payload);
    mqttClient.beginMessage(discoveryTopic.c_str());
    mqttClient.print(payload.c_str());
    mqttClient.endMessage();

    // Discovery for temperature sensor
    String temperatureDiscoveryTopic = "homeassistant/sensor/ESP32-W1/ESP_Temperature/config";
    String temperaturePayload;
    root.clear();
    root["name"] = "Temperature Sensor";
    root["unique_id"] = "temperature_sensor";
    root["state_topic"] = TEMPERATURE_TOPIC;
    root["unit_of_measurement"] = "°C";
    serializeJson(root, temperaturePayload);
    mqttClient.beginMessage(temperatureDiscoveryTopic.c_str());
    mqttClient.print(temperaturePayload.c_str());
    mqttClient.endMessage();

    // Discovery for humidity sensor
    String humidityDiscoveryTopic = "homeassistant/sensor/ESP32-W1/ESP_Humidity/config";
    String humidityPayload;
    root.clear();
    root["name"] = "Humidity Sensor";
    root["unique_id"] = "humidity_sensor";
    root["state_topic"] = HUMIDITY_TOPIC;
    root["unit_of_measurement"] = "%";
    serializeJson(root, humidityPayload);
    mqttClient.beginMessage(humidityDiscoveryTopic.c_str());
    mqttClient.print(humidityPayload.c_str());
    mqttClient.endMessage();
  }
}

int averagePotValue() {
  int total = 0;
  for (int i = 0; i < NUM_SAMPLES; ++i) {
    total += analogRead(POTENTIOMETER_PIN);
    delay(5);
  }
  return total / NUM_SAMPLES;
}

# ESP32 Home Automation Readme

This project implements a home automation system using an ESP32 microcontroller. It connects to Wi-Fi, communicates with Home Assistant Mosquito MQTT broker, and controls relay and 12V LED. The system also reads sensor data like temperature and humidity, and provides an OTA Interface.

## Features

- Connects to Wi-Fi network
- Communicates with an MQTT broker for remote control and monitoring
- Provides a web server for firmware updates
- Reads temperature and humidity sensor data
- Controls relay and LED brightness based on MQTT messages (or local control via button and potenciometer)
- Implements Home Assistant auto-discovery for easy integration

## Dependencies

- [WiFi.h](https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi)
- [ArduinoMqttClient.h](https://github.com/arduino-libraries/ArduinoMqttClient)
- [ArduinoJson.h](https://arduinojson.org/)
- [WebServer.h](https://github.com/espressif/arduino-esp32/tree/master/libraries/WebServer)
- [ESPmDNS.h](https://github.com/espressif/arduino-esp32/tree/master/libraries/ESPmDNS)
- [Update.h](https://github.com/espressif/arduino-esp32/tree/master/libraries/Update)
- [DHT.h](https://github.com/adafruit/DHT-sensor-library)
- [creds.h](#) (Custom file for storing credentials)
- [style.h](#) (Custom file for defining UI styles)
- [FreeRTOS.h](https://github.com/espressif/arduino-esp32/tree/master/libraries/FreeRTOS)
- [Task.h](https://github.com/espressif/arduino-esp32/tree/master/libraries/FreeRTOS/src)

## Setup

1. Include the necessary libraries and headers in your Arduino IDE.
2. Update `creds.h` with your Wi-Fi SSID, password, MQTT broker credentials, and other sensitive information.
3. Update `style.h` with perfered password for login, default user is admin and password is admin.
4. Adjust pin configurations and settings in the code according to your hardware setup.
5. Upload the code to your ESP32 device.

## Usage

- Once the device is powered on, it will attempt to connect to the specified Wi-Fi network and HA MQTT broker.
- The system provides MQTT topics for controlling and monitoring devices such as relays and LEDs.
- Sensor data for temperature and humidity is published to MQTT topics periodically.
- Users can access the web server hosted on the ESP32 for firmware updates and remote control.

## Tasks

The code uses FreeRTOS tasks for multitasking. Each task handles a specific aspect of the system:

1. `connectToWiFi`: Connects the ESP32 to the Wi-Fi network.
2. `connectToMQTTBroker`: Connects the ESP32 to the MQTT broker and initializes MQTT subscriptions.
3. `handleMQTTMessages`: Handles incoming MQTT messages and updates device states accordingly.
4. `handleWebServer`: Handles HTTP requests from the web interface.
5. `readSensors`: Reads sensor data and publishes it to MQTT topics.
6. `KeepMeAlive`: Ensures the MQTT connection stays alive and periodically sends updates.

## Contributions

Contributions to this project are welcome! Feel free to fork the repository, make improvements, and submit pull requests.

## License

This project is licensed under the [MIT License](LICENSE).

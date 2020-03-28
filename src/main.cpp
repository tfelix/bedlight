#include <Arduino.h>

#define OTA_CORE_ID 0
#define SENSOR_CORE_ID 0

#define MQTT_HOST IPAddress(192, 168, 178, 220)
#define MQTT_PORT 1883

#include "mode.h"
#include "wifi/wifi.h"
// #include "led/led.h"
#include "mqtt/mqtt.h"
#include "sensor/sensor.h"

void setup()
{
  Serial.begin(115200);
  delay(50);
  Serial.println("Booting");

  setupOta();
  setupMqtt();
  // setupLed();
  setupSensor();
}

void loop()
{
  // This runs on core 1

  // Core 0 WLAN, OTA, MEAS + FFT
  // Core 1 Stripe, CMD

  // loopLed();
}
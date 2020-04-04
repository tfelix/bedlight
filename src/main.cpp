#include <Arduino.h>

#include "mode.h"
#include "ota/ota.h"
#include "led/led.h"
#include "mqtt/mqtt.h"
#include "sensor/sensor.h"

void setup()
{
  Serial.begin(115200);
  delay(50);
  Serial.println("Booting");

  setupOta();
  setupMqtt();
  setupLed();
  setupSensor();
}

void loop()
{
  // This runs on core 1
  loopLed();
}
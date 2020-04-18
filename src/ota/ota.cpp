#include "ota.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "config.h"
#include "credentials.h"
#include "sensor.h"
#include "mode.h"

TimerHandle_t wifiReconnectTimer;

void connectToWifi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    isWifiConnected = true;

    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection, try reconnecting");

    isWifiConnected = false;

    delay(3000);
    xTimerStart(wifiReconnectTimer, 0);
    break;
  default:
    // no op
    break;
  }
}

void onOtaStart()
{
  // Disable LEDs to prevent flashing during update
  isUpdating = true;

  String type;
  if (ArduinoOTA.getCommand() == U_FLASH)
  {
    type = "sketch";
  }
  else
  {
    // U_SPIFFS
    type = "filesystem";
  }

  // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  Serial.println("Start updating " + type);
}

void onOtaError(ota_error_t error)
{
  Serial.printf("OTA: Error[%u]", error);
  if (error == OTA_AUTH_ERROR)
    Serial.println("OTA: Auth Failed");
  else if (error == OTA_BEGIN_ERROR)
    Serial.println("OTA: Begin Failed");
  else if (error == OTA_CONNECT_ERROR)
    Serial.println("OTA: Connect Failed");
  else if (error == OTA_RECEIVE_ERROR)
    Serial.println("OTA: Receive Failed");
  else if (error == OTA_END_ERROR)
    Serial.println("OTA: End Failed");
}

void otaCheckTask(void *pvParameters)
{
  Serial.println("Setup: wifi");

  wifiReconnectTimer = xTimerCreate(
      "wifiReconnect",
      pdMS_TO_TICKS(5000),
      pdFALSE,
      (void *)0,
      reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWifiEvent);

  ArduinoOTA
      .onStart(onOtaStart)
      .onError(onOtaError)
      .setPort(3232)
      .begin();

  connectToWifi();

  while (true)
  {
    ArduinoOTA.handle();
    delay(200);
  }
}

void setupOta()
{
  xTaskCreatePinnedToCore(
      otaCheckTask, // Function to implement the task
      "ota",        // Name of the task
      10000,        // Stack size in words
      NULL,         // Task input parameter
      1,            // Priority of the task
      NULL,         // Task handle.
      OTA_CORE_ID); // Core where the task should run
}

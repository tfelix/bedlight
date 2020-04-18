#include "mqtt.h"

#include <AsyncMqttClient.h>
#include <WiFi.h>

#include "config.h"
#include "mode.h"

const char *TOPIC_OUTPUT = "bedlight/out";
const char *TOPIC_BEDLIGHT = "bedlight";
const char *TOPIC_BEDLIGHT_COLOR = "bedlight/color";
const char *TOPIC_BEDLIGHT_MODE = "bedlight/mode";

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;

/**
 * Translates the incoming payload into an RGB value for the LEDs to display.
 */
void onMqttMessageBedlightColor(char payload[])
{
  uint8_t i = 0;
  uint8_t rgb[3] = {0, 0, 0};

  char *command = strtok(payload, ",");
  while (command != NULL && i < 3)
  {
    rgb[i] = atoi(command);
    i++;
    command = strtok(NULL, ",");
  }

  currentDisplayMode = COLOR;
  color.setRGB(rgb[0], rgb[1], rgb[2]);
  isLightOn = (rgb[0] + rgb[1] + rgb[2]) != 0;
}

void onMqttMessageBedlight(char *payload)
{
  Serial.print("onMqttMessageBedlight: ");
  Serial.println(payload);

  if (strcasecmp(payload, "ON") == 0)
  {
    isLightOn = true;
  }
  else
  {
    isLightOn = false;
  }
}

void onMqttMessageBedlightMode(char *payload)
{
  Serial.print("onMqttMessageBedlightMode: ");
  Serial.println(payload);

  if (strcasecmp(payload, "RAINBOW") == 0)
  {
    isLightOn = true;
    currentDisplayMode = RAINBOW;
  }
  else if (strcasecmp(payload, "FIRE"))
  {
    isLightOn = true;
    currentDisplayMode = FIRE;
  }
  else if (strcasecmp(payload, "PACIFICIA"))
  {
    isLightOn = true;
    currentDisplayMode = PACIFICIA;
  }
  else if (strcasecmp(payload, "THEATER_CHASE"))
  {
    isLightOn = true;
    currentDisplayMode = THEATER_CHASE;
  }
  else if (strcasecmp(payload, "COLOR"))
  {
    isLightOn = true;
    currentDisplayMode = COLOR;
  }
  else
  {
    isLightOn = false;
    currentDisplayMode = COLOR;
  }
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  // Payload has no zero termination! We need to fix this here.
  char new_payload[len + 1];
  new_payload[len] = '\0';
  strncpy(new_payload, payload, len);

  Serial.print("onMqttMessage - topic: ");
  Serial.print(topic);
  Serial.print(" payload: ");
  Serial.println(new_payload);

  if (strcmp(topic, TOPIC_BEDLIGHT) == 0)
  {
    onMqttMessageBedlight(new_payload);
  }
  else if (strcmp(topic, TOPIC_BEDLIGHT_COLOR) == 0)
  {
    onMqttMessageBedlightColor(new_payload);
  }
  else if (strcmp(topic, TOPIC_BEDLIGHT_MODE) == 0)
  {
    onMqttMessageBedlightMode(new_payload);
  }
  else
  {
    char buf[256];
    snprintf(buf, sizeof buf, "Unknown topic: %s, payload: %s", topic, new_payload);
    printMessage(buf);
  }
}

void printMessage(const __FlashStringHelper *ifsh)
{
  printMessage(reinterpret_cast<const char *>(ifsh));
}

void printMessage(const char msg[])
{
  Serial.println(msg);
  mqttClient.publish(TOPIC_OUTPUT, 1, false, msg);
}

void connectToMqtt()
{
  if (!isWifiConnected)
  {
    // Retry later when Wifi is maybe enabled again as we can not connect to
    // mqtt without wifi.
    xTimerStart(mqttReconnectTimer, 0);
    return;
  }

  if (!isMqttConnected)
  {
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT");
  isMqttConnected = true;

  mqttClient.subscribe(TOPIC_BEDLIGHT, 1);
  mqttClient.subscribe(TOPIC_BEDLIGHT_MODE, 1);
  mqttClient.subscribe(TOPIC_BEDLIGHT_COLOR, 1);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT");

  // Switch light of if we loose connection to MQTT so we are not
  // getting annoyed because we can not control it anymore.
  // If we loose connection too regularly we might to implement some kind of timed
  // switch which turns off the light after 30sec or so.
  isMqttConnected = false;

  xTimerStart(mqttReconnectTimer, 0);
}

void setupMqtt()
{
  Serial.println("Setup: mqtt");
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToMqtt();
}
#ifndef __INC_BEDLIGHT_MQTT_H
#define __INC_BEDLIGHT_MQTT_H

#include <WString.h>

void setupMqtt();

void printMessage(const __FlashStringHelper *ifsh);
void printMessage(const char msg[]);

#endif
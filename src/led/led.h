#ifndef __INC_BEDLIGHT_LED_H
#define __INC_BEDLIGHT_LED_H

#include <FastLED.h>
#include "config.h"

void setupLed();
void loopLed();

extern CRGB leds[NUM_LEDS];

#endif
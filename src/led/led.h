#ifndef __INC_BEDLIGHT_LED_H
#define __INC_BEDLIGHT_LED_H

#include <FastLED.h>

/**
 * TOP_START = 176
 * TOP_COUNT = 103
 */
#define NUM_LEDS 450

// #define MAX_BRIGTHNESS 90

// Energy decays for 3 units in one second
#define ENERGY_DECAY_S 3

void setupLed();
void loopLed();

extern CRGB leds[NUM_LEDS];

#endif
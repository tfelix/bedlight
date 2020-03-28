#ifndef __INC_BEDLIGHT_MODE_H
#define __INC_BEDLIGHT_MODE_H

#include <Arduino.h>
#include <FastLED.h>

enum DisplayMode
{
  RAINBOW,
  THEATER_CHASE,
  FIRE,
  COLOR,
  PACIFICIA
};

extern volatile bool isLightOn;
extern volatile DisplayMode currentDisplayMode;
extern CRGB color;

extern volatile bool shouldRecalibrateSensor;
extern volatile uint8_t energy;
extern volatile uint8_t frequency;

#endif
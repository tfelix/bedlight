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
  PACIFICIA,
  MOVING_DOT,
  HAL,
  SEXY_TIME
};

extern volatile bool isMotionActive;

extern volatile bool isUpdating;
extern volatile bool isWifiConnected;
extern volatile bool isMqttConnected;
extern volatile bool isLightOn;

extern volatile DisplayMode currentDisplayMode;
extern CRGB color;

extern volatile bool shouldRecalibrateSensor;

extern volatile uint16_t motionThreshold;
extern volatile uint8_t motionSensitivity;
extern volatile uint8_t motionDecay;
#endif
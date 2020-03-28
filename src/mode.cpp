#include "mode.h"

volatile bool isLightOn = false;
volatile DisplayMode currentDisplayMode = RAINBOW;
CRGB color = CRGB::Blue;

volatile uint8_t brightness = 0;
volatile uint8_t energy = 255;
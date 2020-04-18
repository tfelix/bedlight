#include "mode.h"

volatile bool isUpdating = false;
volatile bool isWifiConnected = false;
volatile bool isMqttConnected = false;
volatile bool isLightOn = false;

volatile DisplayMode currentDisplayMode = RAINBOW;
CRGB color = CRGB::Blue;

volatile uint8_t brightness = 0;
volatile uint8_t energy = 255;
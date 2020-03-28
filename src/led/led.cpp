#include "led.h"

#include <FastLED.h>

#include "mode.h"

#include "led/pacificia.h"
#include "led/fx.h"

#define DATA_PIN 27

CRGB leds[NUM_LEDS];

void setupLed()
{
  Serial.println("Setup: LED");
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);

  // Brigthness 255 is flickering at the end of the strip,
  // 200 less but too, 100 works so far.
  // Try out 5V and 15 Amps
  FastLED.setMaxPowerInMilliWatts(70000);
  // FastLED.setBrightness(100);
  FastLED.setCorrection(TypicalLEDStrip);

  FastLED.clear(true);
  FastLED.show();
}

void loopLed()
{

  if (!isLightOn)
  {
    FastLED.clear(true);
    delay(100);
    return;
  }

  switch (currentDisplayMode)
  {
  case COLOR:
    showColor();
    break;
  case PACIFICIA:
    showPacificia();
    break;
  case RAINBOW:
    rainbow();
    break;
  case THEATER_CHASE:
    theaterChase();
    break;
  default:
    isLightOn = false;
    return;
  }
}
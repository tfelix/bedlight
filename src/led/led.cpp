#include "led.h"

#include <FastLED.h>

#include "mode.h"
#include "motion.h"
#include "config.h"

#include "led/pacificia.h"
#include "led/fx.h"

CRGB leds[NUM_LEDS];

void setupLed()
{
  Serial.println("Setup: LED");
  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);

  // Brigthness 255 is flickering at the end of the strip,
  // 200 less but too, 100 works so far.
  FastLED.setMaxPowerInMilliWatts(MAX_LED_OUTPUT_MW);
  // FastLED.setBrightness(100);
  FastLED.setCorrection(TypicalLEDStrip);

  delay(100);
  FastLED.clear(true);
  FastLED.show();
}

void loopLed()
{
  if (isMotionActive)
  {
    decayMotionEnergy();
  }
  else
  {
    setAllEnergy(0);
  }

  if (!isLightOn || !isWifiConnected || !isMqttConnected || isUpdating)
  {
    FastLED.clear(true);
    FastLED.show();
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

  yield();
}
#include "fx.h"

#include <algorithm>

#include "led/led.h"
#include "motion.h"
#include "mode.h"

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
CRGB wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return CRGB(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170)
  {
    WheelPos -= 85;
    return CRGB(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return CRGB(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void rainbow()
{
  static uint16_t rainbowCounter = 0;

  rainbowCounter++;
  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = wheel(i + rainbowCounter);
  }

  FastLED.show();
  delay(50);
}

void showColor()
{
  if (isMotionActive)
  {
    auto energy = getEnergy();
    auto maxMotionE = std::max(std::max(energy.low, energy.mid), energy.high) / 255.0;
    // Perform the motion active animation.
    for (auto i = 0; i < NUM_LEDS; i++)
    {
      CHSV hsv = rgb2hsv_approximate(leds[i]);
      hsv.v = 255 * maxMotionE;
      hsv2rgb_rainbow(hsv, leds[i]);
    }
  }
  else
  {
    FastLED.showColor(color);
  }

  delay(5);
}

// Theatre-style crawling lights.
void theaterChase()
{
  static uint16_t j = 0;

  for (int q = 0; q < 3; q++)
  {
    for (uint16_t i = 0; i < NUM_LEDS; i = i + 3)
    {
      leds[i + q] = color;
    }

    FastLED.show();
    delay(30);

    for (uint16_t i = 0; i < NUM_LEDS; i = i + 3)
    {
      leds[i + q] = CRGB::Black;
    }
  }

  j++;

  //do 10 cycles of chasing
  if (j >= 10)
  {
    j = 0;
  }
}
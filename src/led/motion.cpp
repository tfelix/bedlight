#include "motion.h"

#include <algorithm>

#include "mode.h"

// FIXME Protect by sempahore
uint8_t motionEnergy[] = {0, 0, 0};

void convertToEnergy(const double bins[])
{
  double overshoot = 0;
  uint8_t newEnergy = 0;

  for (auto i = 0; i < 3; i++)
  {
    overshoot = bins[i + 1] - motionThreshold;
    if (overshoot < 0)
    {
      overshoot = 0;
    }
    // FIXME improve normalization here and use sensitivity setting.
    // We get a measurement every 0.4s (try to fasten this somehow...)
    // Find a way to "normalize" this so we aim for max energy after 1sek of
    // measurement.
    newEnergy = static_cast<uint8_t>(overshoot / 100.0 * 255 * 0.4);
    motionEnergy[i] = std::min(motionEnergy[i] + newEnergy, 255);
  }
}

void setAllEnergy(uint8_t value)
{
  motionEnergy[0] = value;
  motionEnergy[1] = value;
  motionEnergy[2] = value;
}

MotionEnergy getEnergy()
{
  struct MotionEnergy energy;

  // TODO Maybe use the struct to save the data as well.
  energy.low = motionEnergy[0];
  energy.mid = motionEnergy[1];
  energy.high = motionEnergy[2];

  return energy;
}

/**
 * Does periodically reduce the energy from motion.
 */
void decayMotionEnergy()
{
  static auto lastCallPrint = 0;
  static auto lastCall = millis();
  const auto dT = (millis() - lastCall) / 1000.0;

  // Usual reduction is 100% in 1s
  const auto reduction = static_cast<uint8_t>(dT * 255);

  // We need to make sure we keep our dT until we actually can reduce
  // a value.
  if (reduction > 0)
  {
    motionEnergy[0] = std::max(motionEnergy[0] - reduction, 0);
    motionEnergy[1] = std::max(motionEnergy[1] - reduction, 0);
    motionEnergy[2] = std::max(motionEnergy[2] - reduction, 0);
    lastCall = millis();
  }

  if (millis() - lastCallPrint > 200)
  {
    Serial.print("mE0: ");
    Serial.println(motionEnergy[0]);
    lastCallPrint = millis();
  }
}
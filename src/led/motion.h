#ifndef __INC_BEDLIGHT_MOTION_H
#define __INC_BEDLIGHT_MOTION_H

#include <Arduino.h>

struct MotionEnergy
{
  uint8_t low;
  uint8_t mid;
  uint8_t high;
};

/**
 * Converts the result of the FFT bins into motion energy.
 */
void convertToEnergy(const double bins[]);

MotionEnergy getEnergy();

/**
 * Sets all motion energy buckets to this value
 */
void setAllEnergy(uint8_t value);

void decayMotionEnergy();

#endif
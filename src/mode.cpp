#include "mode.h"

volatile bool isUpdating = false;
volatile bool isWifiConnected = false;
volatile bool isMqttConnected = false;
volatile bool isLightOn = false;
volatile bool isMotionActive = false;

volatile DisplayMode currentDisplayMode = RAINBOW;
CRGB color = CRGB::Blue;

volatile uint8_t brightness = 0;

/**
 * Marks the threshold above which the motion energy is added
 * up the the energy readings. Motion under this value is ignored.
 *
 * Possible values are 50-300
 */
volatile uint16_t motionThreshold = 0;

/**
 * The sensitivity controls how many of the energy above the threshold
 * is converted into motion energy.
 */
volatile uint8_t motionSensitivity = 100;

/**
 * Gives the percentage of motion energy decay within one second.
 *
 * A value of 100 means
 *
 * Values are 5-250.
 */
volatile uint8_t motionDecay = 100;
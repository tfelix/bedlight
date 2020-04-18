
#ifndef __INC_CONFIG_H
#define __INC_CONFIG_H

#define OTA_CORE_ID 0

#define MQTT_HOST IPAddress(192, 168, 178, 220)
#define MQTT_PORT 1883

/**
 * Core on which the sensors are running.
 */
#define SENSOR_CORE_ID 1

/**
 * Pin where the sensor interrupt is connected to.
 */
#define SENSOR_INTERRUPT_PIN 19

/**
 * TOP_START = 176
 * TOP_COUNT = 103
 */
#define NUM_LEDS 450

/**
 * Pin on which the LED dataline is connected.
 */
#define LED_DATA_PIN 27

/**
 * Max output power of the LEDs in mW
 */
#define MAX_LED_OUTPUT_MW 60000

// Energy decays for 3 units in one second
// #define ENERGY_DECAY_S 3

#endif
/*
 * sensor.c
 *
 *  Created on: Nov 25, 2025
 *      Author: Gandhi
 */
#include "sensor.h"
#include "main.h"   // for HAL_GetTick()
#include <math.h>   // optional

// Simple internal state
static int16_t base_temp = 250;   // 25.0 °C in tenths
static int16_t drift     = 0;

void sensor_init(void)
{
    base_temp = 250;  // 25.0 °C
    drift = 0;
}

// Emulated raw sensor reading in "tenths of °C"
int16_t sensor_read_raw(void)
{
    uint32_t t = HAL_GetTick();   // ms since boot

    // Make it change slowly over time
    int16_t wave = (t / 100) % 20;      // 0..19
    int16_t noise = (int16_t)(t & 0x03); // 0..3 low-bit noise

    // Base 25.0°C, plus a little variation 0..1.9°C
    int16_t temp_tenths = base_temp + wave + noise; // e.g., 250..272

    return temp_tenths;
}

float sensor_read_celsius(void)
{
    int16_t raw = sensor_read_raw();
    return raw / 10.0f;   // convert tenths to float °C
}



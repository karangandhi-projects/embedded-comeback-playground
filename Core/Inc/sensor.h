/*
 * sensor.h
 *
 *  Created on: Nov 25, 2025
 *      Author: Gandhi
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

void sensor_init(void);
int16_t sensor_read_raw(void);
float sensor_read_celsius(void);

#endif /* SENSOR_H_ */

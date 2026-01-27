#ifndef __SENSOR_H
#define __SENSOR_H
#include <stdint.h>
void sensor_init(void);
uint8_t sensor(void);
uint8_t IRSensorL1_Get(void);
uint8_t IRSensorL2_Get(void);
uint8_t IRSensorR2_Get(void);
uint8_t IRSensorR1_Get(void);
uint8_t control(void);


#endif
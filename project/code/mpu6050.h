#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include "zf_common_headfile.h"

extern uint8_t mpu6050_runflag;

void mpu_init(void);
void drift_init(void);
float mpu_calculate(void);


#endif
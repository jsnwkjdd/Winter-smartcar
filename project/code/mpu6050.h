#ifndef __MPU6050_H
#define __MPU6050_H
#include <stdint.h>
void mpu6050estimation_Pitch(float*Pitch);
void sum_Pitch(int16_t*sacc_x,int16_t*sacc_z,int16_t*sgyro_y);
void mpu6050estimation(float*Pitch,float*Roll,float*Yaw);
void fliter(float*a,float alpha);
#endif
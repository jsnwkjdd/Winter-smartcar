#ifndef __MPU6050_H
#define __MPU6050_H

void mpu6050estimation(float*Pitch,float*Roll,float*Yaw);
void fliter(float*a,float alpha);
#endif 


#ifndef __MAHONY_H
#define __MAHONY_H

#include <stdint.h>

void calibrate_gyro();
void calibrate_accel();
void filterax1(float* a, float alpha);
void filteray1(float* a, float alpha);
void filteraz1(float* a, float alpha);
void filtergx1(float* a, float alpha);
void filtergy1(float* a, float alpha);
void filtergz1(float* a, float alpha);
void pre_mahony() ;
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MahonyAHRSupdateIMU2(float gx, float gy, float gz, float ax, float ay, float az);
int16_t get_angles_from_quaternion(float q0, float q1, float q2, float q3,
                               float* roll, float* pitch, float* yaw);
#endif

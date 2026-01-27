#include "zf_device_mpu6050.h"
#include <math.h>
#include "zf_common_headfile.h"

uint8_t mpu6050_runflag=0;
float mpu6050_gyro_y_drift=0,mpu6050_acc_x_drift=0,mpu6050_acc_y_drift=0,mpu6050_acc_z_drift=0;
float angle_acc_x,angle_gyro_y,angle_acc_y,angle_acc_z;
float angle_acc_x_start,angle_gyro_y_start,angle_acc_y_start,angle_acc_z_start;
float pitch_acc_rad,pitch_acc_deg,pitch_gyro_deg,pitch;
float k=0.001;

//mpu6050初始化
void mpu_init(void)
{
	mpu6050_runflag=mpu6050_init();
}

//零漂初始化函数
void drift_init(void)
{
	int32_t i;
	for(i=0;i<1000;i++)
	{
		mpu6050_get_gyro();
		mpu6050_get_acc ();
		mpu6050_gyro_y_drift+=mpu6050_gyro_y;
		mpu6050_acc_x_drift+=mpu6050_acc_x;
		mpu6050_acc_y_drift+=mpu6050_acc_y;
		mpu6050_acc_z_drift+=mpu6050_acc_z;
		system_delay_ms(10);
	}
	mpu6050_gyro_y_drift=mpu6050_gyro_y_drift/1000.0f;
	mpu6050_acc_x_drift=mpu6050_acc_x_drift/1000.0f;
	mpu6050_acc_y_drift=mpu6050_acc_y_drift/1000.0f;
	mpu6050_acc_z_drift=mpu6050_acc_z_drift/1000.0f;
}


//姿态解算函数
float mpu_calculate(void)
{	
	mpu6050_get_acc ();
	mpu6050_get_gyro();
	
	angle_acc_x_start=mpu6050_acc_x-mpu6050_acc_x_drift;
	angle_acc_y_start=mpu6050_acc_y-mpu6050_acc_y_drift;
	angle_acc_z_start=mpu6050_acc_z-mpu6050_acc_z_drift;
	angle_gyro_y_start=mpu6050_gyro_y-mpu6050_gyro_y_drift;
	
	angle_acc_x=mpu6050_acc_transition(angle_acc_x_start);
	angle_acc_y=mpu6050_acc_transition(angle_acc_y_start);
	angle_acc_z=mpu6050_acc_transition(angle_acc_z_start);
	angle_gyro_y=mpu6050_gyro_transition(angle_gyro_y_start);
	
	pitch_acc_rad = atan2(-angle_acc_x, sqrt(angle_acc_y*angle_acc_y + angle_acc_z*angle_acc_z));
	pitch_acc_deg = pitch_acc_rad * 180.0 / 3.1415926;
	pitch_gyro_deg=pitch+angle_gyro_y*0.008;
	
	pitch=k*pitch_acc_deg+(1-k) * (pitch + pitch_gyro_deg);
	
	return pitch;
	
}
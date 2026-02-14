#include "zf_common_headfile.h"
#include "menu.h"
#include <math.h>
#define DEG_TO_RAD 0.017453292519943295f  // π/180
#define RAD_TO_DEG 57.29577951308232f     // 180/π    
float lubu_acc=0.8;
float roll,pitch,yaw;
int16_t mahony2cnt=50;

#define sampleFreq	100.0f			// 随调用频率改变，单位hz，目前10ms
#define twoKpDef	(2.0f * 0.5f)	// Kp=0.5
#define twoKiDef	(2.0f * 0.0f)	// Ki=0!!!别忘了改！！


/*
调用：mpu6050_init();
calibrate_accel_level();
calibrate_gyro();
在中断里面：pre_mahony() ;
		MahonyAHRSupdateIMU(gx1*DEG_TO_RAD,gy1*DEG_TO_RAD,gz1*DEG_TO_RAD,ax1,ay1,az1);
		或者MahonyAHRSupdateIMU2(gx1*DEG_TO_RAD,gy1*DEG_TO_RAD,gz1*DEG_TO_RAD,ax1,ay1,az1);
		get_angles_from_quaternion(q0,q1,q2,q3,&roll,&pitch,&yaw);
*/

//陀螺仪中值积分
//看一下三轴方向是否正确
//一般的处理方法可以计算角速度计的模长，若越大则加速度的置信越小。
//对于角加速度可以通过陀螺仪来计算得到，得到角加速度后可以计算出切向加速度和法向加速度分别补偿。

float gyro_bias_x=0,gyro_bias_y=0,gyro_bias_z=0,accel_bias_x=0,accel_bias_y=0,accel_bias_z=0;
float ax1,ay1,az1,gx1,gy1,gz1;
void filterax1(float* a, float alpha);
void filteray1(float* a, float alpha);
void filteraz1(float* a, float alpha);
void filtergx1(float* a, float alpha);
void filtergy1(float* a, float alpha);
void filtergz1(float* a, float alpha);



//@brief 数据预处理
void pre_mahony() 
{

    // 1. 读取传感器
	mpu6050_get_acc(); 
	mpu6050_get_gyro();
    // 2. 单位转换
    ax1=mpu6050_acc_transition(mpu6050_acc_x);//g
    ay1=mpu6050_acc_transition(mpu6050_acc_y);
	az1=mpu6050_acc_transition(mpu6050_acc_z);
	gx1=mpu6050_gyro_transition(mpu6050_gyro_x);//°/s
	gy1=mpu6050_gyro_transition(mpu6050_gyro_y);
	gz1=mpu6050_gyro_transition(mpu6050_gyro_z);
    // 3. 减去零偏
    gx1 -= gyro_bias_x;
    gy1 -= gyro_bias_y;
    gz1 -= gyro_bias_z;
	ax1 -= accel_bias_x;
    ay1 -= accel_bias_y;
    az1 -= accel_bias_z;
    // 4.一阶低通滤波
	filterax1(&ax1,lubu_acc);//本次权重
	filteray1(&ay1,lubu_acc);
	filteraz1(&az1,lubu_acc);
	filtergx1(&gx1,0.9);
	filtergy1(&gy1,0.9);
	filtergz1(&gz1,0.9);
	// 5.限幅(太小就忽略？？？？	
//	if(gy1>-5&&gy1<5)//角速度y限幅
//	{
//		gy1=0;
//	}
}


//四元数转换为欧拉角
int16_t get_angles_from_quaternion(float q0, float q1, float q2, float q3,
                               float* roll, float* pitch, float* yaw) 
{
    float norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;// 检查四元数是否为单位四元数
    if (norm < 0.9f || norm > 1.1f) 
	{
        return -1; 
    }
    // Pitch 
    float sinp = 2.0f * (q0 * q2 - q1 * q3);
    
	if (sinp > 1.0f) sinp = 1.0f;   // 强制拉回来
	if (sinp < -1.0f) sinp = -1.0f; // 强制拉回来

     *pitch = asinf(sinp) * RAD_TO_DEG;// 弧度转角度
    
    //  Roll
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;// 弧度转角度
    
    // Yaw
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;// 弧度转角度
    
//    // 4. 可选：将Yaw转换到[0, 360)范围
//    if (*yaw < 0)
//        *yaw += 360.0f;
	return 1;
}

// @brief:零飘校准(物理量转换后)

void calibrate_gyro() 
{
    int32_t sum_gx = 0,sum_gy = 0, sum_gz = 0;
    int16_t samples = 1000;
    tft180_show_string (0,110,"begin");
    
    for(int16_t i = 0; i < samples; i++)
	{
        mpu6050_get_gyro();
        sum_gx += mpu6050_gyro_transition(mpu6050_gyro_x);
        sum_gy += mpu6050_gyro_transition(mpu6050_gyro_y);
        sum_gz += mpu6050_gyro_transition(mpu6050_gyro_z);
		system_delay_ms(8);
    }
    gyro_bias_x = (float)sum_gx / samples;
    gyro_bias_y = (float)sum_gy / samples;
    gyro_bias_z = (float)sum_gz / samples;
	tft180_show_string (0,110,"ok");
    
}

// 加速度计校准（水平放置）
void calibrate_accel() 
{
    int16_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int16_t samples = 500; 
    tft180_show_string (0,110,"begin");
    for(int16_t i = 0; i < samples; i++) 
	{
		mpu6050_get_acc(); 
        sum_ax += mpu6050_acc_transition(mpu6050_acc_x);//g
        sum_ay += mpu6050_acc_transition(mpu6050_acc_y);
        sum_az += mpu6050_acc_transition(mpu6050_acc_z);
        system_delay_ms(8);
    }
    
    accel_bias_x = (float)sum_ax / samples;
    accel_bias_y = (float)sum_ay / samples;
    accel_bias_z = 1.0f / ((float)sum_az / samples);  // 归一化
	tft180_show_string (0,110,"ok");
}
//一阶低通滤波
void filterax1(float* a, float alpha)
{
    static float last = 0.0f;
    static int initialized = 0;
    
    if (!initialized) {
        last = *a;
        initialized = 1;
    }
    
    // 一阶低通滤波：y[n] = α*x[n] + (1-α)*y[n-1]
    float result = alpha * (*a) + (1.0f - alpha) * last;
    last = result;
    *a = result;
}
//一阶低通滤波
void filteray1(float* a, float alpha)
{
    static float last = 0.0f;
    static int initialized = 0;
    
    if (!initialized) {
        last = *a;
        initialized = 1;
    }
    
    // 一阶低通滤波：y[n] = α*x[n] + (1-α)*y[n-1]
    float result = alpha * (*a) + (1.0f - alpha) * last;
    last = result;
    *a = result;
}

//一阶低通滤波
void filteraz1(float* a, float alpha)
{
    static float last = 0.0f;
    static int initialized = 0;
    
    if (!initialized) {
        last = *a;
        initialized = 1;
    }
    
    // 一阶低通滤波：y[n] = α*x[n] + (1-α)*y[n-1]
    float result = alpha * (*a) + (1.0f - alpha) * last;
    last = result;
    *a = result;
}

//一阶低通滤波
void filtergx1(float* a, float alpha)
{
    static float last = 0.0f;
    static int initialized = 0;
    
    if (!initialized) {
        last = *a;
        initialized = 1;
    }
    
    // 一阶低通滤波：y[n] = α*x[n] + (1-α)*y[n-1]
    float result = alpha * (*a) + (1.0f - alpha) * last;
    last = result;
    *a = result;
}

//一阶低通滤波
void filtergy1(float* a, float alpha)
{
    static float last = 0.0f;
    static int initialized = 0;
    
    if (!initialized) {
        last = *a;
        initialized = 1;
    }
    
    // 一阶低通滤波：y[n] = α*x[n] + (1-α)*y[n-1]
    float result = alpha * (*a) + (1.0f - alpha) * last;
    last = result;
    *a = result;
}

//一阶低通滤波
void filtergz1(float* a, float alpha)
{
    static float last = 0.0f;
    static int initialized = 0;
    
    if (!initialized) {
        last = *a;
        initialized = 1;
    }
    
    // 一阶低通滤波：y[n] = α*x[n] + (1-α)*y[n-1]
    float result = alpha * (*a) + (1.0f - alpha) * last;
    last = result;
    *a = result;
}

// 函数简介   判断设备是否处于静止状态

int16_t is_static_state(float ax_g, float ay_g, float az_g)
{
    float norm = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g); // 计算加速度模长（g 为单位）
    return (norm >= 0.9f && norm <= 1.1f) ? 1 : 0;      // 判断是否在静止范围内
}

//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);



void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root


void MahonyAHRSupdateIMU2(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float integralFBx_limit,integralFBy_limit,integralFBz_limit;//未赋值

	
	//开始时快速收敛
	if(mahony2cnt>0)
	{
		twoKp=10.0f;
		twoKi=0.0f;
	
	}
	else
	{
		twoKp = twoKpDef;
		twoKi = twoKiDef;
	}
	if( !is_static_state(ax1,ay1,az1) )//如果运动
	{
		//改变kp，ki
//		twoKp=10.0f;//未设置
//		twoKi=0.0f;
		//改变一阶滤波系数
//		lubu_acc=0.55;
	
	}
	
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		//积分限幅
		if (integralFBx > integralFBx_limit)integralFBx = integralFBx_limit;
		else if (integralFBx < -integralFBx_limit) integralFBx = -integralFBx_limit;
		if (integralFBy > integralFBy_limit)integralFBy = integralFBy_limit;
		else if (integralFBy < -integralFBy_limit) integralFBy = -integralFBy_limit;
		if (integralFBz > integralFBz_limit)integralFBz = integralFBz_limit;
		else if (integralFBz < -integralFBz_limit) integralFBz = -integralFBz_limit;
		
		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}








float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
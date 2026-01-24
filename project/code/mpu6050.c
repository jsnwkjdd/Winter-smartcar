#include "mm32_device.h"                // Device header
#include "zf_device_mpu6050.h"
#include <math.h>

float gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z;
float AX,AY,AZ,GX,GY,GZ,AngleX,AngleY,AngleZ;//互补滤波中间量
float AlphaX = 0.001,AlphaY = 0.001,t=0.01;//t角速度积分，和定时中断同步
//使用：别忘了初始化：mpu6050_init();


/*	功能：数据预处理一阶低通滤波函数
	备注：内部调用
	返回值：void
	参数：需滤波变量地址,一阶低通滤波参数

*/
void fliter(float*a,float alpha)
{
	static int last;
	int sum;
	sum=(*a)*alpha+(1-alpha)*last;//两次加权平均数
	last=*a;//保留本次数据供下次使用
	*a=sum;//输出滤波后数据
}



/* 
功能：读取mpu6050数据并进行姿态解算
参数：储存解算好的Pitch,roll,Yaw角的变量指针
返回值：void
*/
void mpu6050estimation(float*Pitch,float*Roll,float*Yaw)
{
	mpu6050_get_acc(); //读取加速度计初始数据
	mpu6050_get_gyro(); //读取角速度计初始数据 
	acc_x = mpu6050_acc_transition(mpu6050_acc_x); //三轴加速度计转化为物理量
	acc_y = mpu6050_acc_transition(mpu6050_acc_y);
	acc_z = mpu6050_acc_transition(mpu6050_acc_z);	
	gyro_x = mpu6050_gyro_transition(mpu6050_gyro_x); //三轴角速度计转化为物理量
	gyro_y = mpu6050_gyro_transition(mpu6050_gyro_y);
	gyro_z = mpu6050_gyro_transition(mpu6050_gyro_z);
	
	//数据预处理:滤波，去零飘，坐标轴标定：
	fliter(&acc_x,0.8);
	fliter(&acc_y,0.8);
	fliter(&acc_z,0.8);
	fliter(&gyro_x,0.8);
	fliter(&gyro_y,0.8);
	fliter(&gyro_z,0.8);
		
	AY = -atan2(acc_x,acc_z);//-? 得到加速度计算出的角度
	GY = AngleY + gyro_y*t;		//得到角速度计算出的角度,t角速度积分，和定时中断同步
	AngleY = AlphaY * AY + (1 - AlphaY) * GY;//互补滤波
	*Pitch=AngleY;//赋值给储存Pitch的变量
		
	AX = -atan2(acc_y,acc_z);//-?
	GX = AngleX + gyro_x*t;
	AngleX = AlphaX * AX + (1 - AlphaX) * GX;
	*Roll=AngleX;//Roll get！
	
	AngleZ += gyro_z*t;
	*Yaw=AngleZ;//Yaw get！
	//AngleZ = Alpha * AZ + (1 - Alpha) * GZ;
	
}


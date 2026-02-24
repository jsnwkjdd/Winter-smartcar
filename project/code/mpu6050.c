#include "zf_device_mpu6050.h"
#include "zf_driver_soft_iic.h"
#include <math.h>
#include "zf_common_headfile.h"

 
float t=0.01,zero=7,A_ration=0.04,AlphaPitch = 0.025;//t角速度积分，和定时中断同步/zero机械零点/

extern soft_iic_info_struct mpu6050_iic_struct;
float AlphaX = 0.001;//互补滤波参数
int16_t acc_xbias,acc_ybias,acc_zbias,gyro_xbias,gyro_ybias,gyro_zbias;//零飘校准
float gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,gyro_y1;//数据处理中间量
float AX,AY,AZ,GX,GY,GZ,AngleX,AngleY,AngleZ;
int16_t ax,az,gy,flag_mpu=0;//互补滤波中间量
void filterax(float*a,float alpha);
void filteraz(float* a, float alpha);
void filtergy(float* a, float alpha);
/* 
功能：读取mpu6050数据并进行Pitch姿态解算 Pitch+限幅滤波+互补滤波
参数：储存解算好的Pitch,变量指针
返回值：void
*/


void mpu6050estimation_Pitch(float*Pitch)
{
    // 静态变量：滤波历史值 + 数据异常检测
    static float last_acc_x = 0.0f, last_acc_z = 0.0f, last_gyro_y = 0.0f;
    static uint8_t init_flag = 0;
    static int16_t last_acc_x_raw = 0, last_acc_z_raw = 0, last_gyro_y_raw = 0; // 新增：记录上一次原始数据
    static uint8_t error_count = 0; // 新增：异常计数

    if(init_flag == 0)
    {
        last_acc_x = 0.0f;
        last_acc_z = 0.0f;
        last_gyro_y = 0.0f;
        last_acc_x_raw = 0;
        last_acc_z_raw = 0;
        last_gyro_y_raw = 0;
        error_count = 0;
        init_flag = 1;
    }

    // 1. 读取传感器数据（逐飞库原生方式）
    mpu6050_get_acc(); 
    mpu6050_get_gyro(); 

    // 2. 检测数据是否异常（通信失败的特征）
    uint8_t data_error = 0;
    // 异常条件1：数据和上一次完全一样（总线卡死，数据不更新）
    if(mpu6050_acc_x == last_acc_x_raw && mpu6050_acc_z == last_acc_z_raw && mpu6050_gyro_y == last_gyro_y_raw)
    {
        error_count++;
        if(error_count >= 3) // 连续3次数据不变，判定为通信失败
        {
            data_error = 1;
            error_count = 0; // 重置计数
        }
    }
    else
    {
        error_count = 0; // 数据更新，重置异常计数
    }

    // 异常条件2：数据为极值（IIC通信乱码）
    if(mpu6050_acc_x == 32767 || mpu6050_acc_x == -32768 || 
       mpu6050_acc_z == 32767 || mpu6050_acc_z == -32768 ||
       mpu6050_gyro_y == 32767 || mpu6050_gyro_y == -32768)
    {
        data_error = 1;
    }

    // 3. 通信失败时，重新初始化（仅应急）
    if(data_error == 1)
    {
        // 重新初始化IIC和MPU6050
        soft_iic_init(&mpu6050_iic_struct, MPU6050_DEV_ADDR, MPU6050_SOFT_IIC_DELAY, MPU6050_SCL_PIN, MPU6050_SDA_PIN);
        mpu6050_init();
		system_delay_ms(1);
        // 重试读取一次
        mpu6050_get_acc();
        mpu6050_get_gyro();
    }

    // 4. 保存本次原始数据，用于下一次异常检测
    last_acc_x_raw = mpu6050_acc_x;
    last_acc_z_raw = mpu6050_acc_z;
    last_gyro_y_raw = mpu6050_gyro_y;

    // 5. 原有解算逻辑（不变）
    ax=mpu6050_acc_x-15;
    az=mpu6050_acc_z-20;
    gy=mpu6050_gyro_y+6;

    acc_x = mpu6050_acc_transition(ax);
    acc_z = mpu6050_acc_transition(az);
    gyro_y = mpu6050_gyro_transition(gy);
    
    // 简化滤波
    acc_x = 0.9 * acc_x + 0.1 * last_acc_x;
    acc_z = 0.9 * acc_z + 0.1 * last_acc_z;
    gyro_y = 0.83 * gyro_y + 0.17 * last_gyro_y;
    last_acc_x = acc_x;
    last_acc_z = acc_z;
    last_gyro_y = gyro_y;

    gyro_y1=gyro_y;
    AY = -atan2(acc_x,acc_z)* 180.0f / 3.14159265f;
    GY = AngleY + gyro_y1*t;
    if(flag_mpu<=0)
    {
        AngleY = AlphaPitch * AY + (1 - AlphaPitch) * GY;
    }
    else
    {
        AngleY=AY;
    }

    *Pitch=-AngleY+zero;
}







/*mpu6050_get_acc(); 
mpu6050_get_gyro();
ACC = -atan2(mpu6050_acc_x,mpu6050_acc_z)* 180.0f / 3.14159265f;
GCC=p+mpu6050_acc_y/32768.0*2000*0.001;
p=0.001*ACC+0.999*GCC;*/
//使用：别忘了初始化：mpu6050_init();


/*acc_x=(float)mpu6050_acc_x;
acc_y=(float)mpu6050_acc_y;
acc_z=(float)mpu6050_acc_z;
gyro_x=(float)mpu6050_gyro_x;
gyro_y=(float)mpu6050_gyro_y;
gyro_z=(float)mpu6050_gyro_z;
fliter(&acc_x,0.8);
fliter(&acc_y,0.8);
fliter(&acc_z,0.8);
fliter(&gyro_x,0.8);
fliter(&gyro_y,0.8);
fliter(&gyro_z,0.8);
*/


/*功能：数据预处理一阶低通滤波函数
备注：内部调用
返回值：void
参数：需滤波变量地址,一阶低通滤波参数

*/
void filterax(float* a, float alpha)
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

void filteraz(float* a, float alpha)
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

void filtergy(float* a, float alpha)
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

/* 
功能：读取mpu6050数据并进行Pitch姿态解算2 Pitch+限幅滤波+互补滤波，Kp版
参数：储存解算好的Pitch,变量指针
返回值：void
*/
void mpu6050estimation_Pitch2(float*Pitch)
{
mpu6050_get_acc(); //读取加速度计初始数据
mpu6050_get_gyro(); //读取角速度计初始数据 

//数据预处理:去零飘+限幅滤波，坐标轴标定：

mpu6050_acc_x-=acc_xbias;//去零飘
mpu6050_acc_z-=acc_zbias;
mpu6050_gyro_y-=gyro_ybias;

/*if(mpu6050_acc_x>-5&&mpu6050_acc_x<5)//限幅滤波
{
mpu6050_acc_x=0;
}
if(mpu6050_acc_z>-5&&mpu6050_acc_z<5)
{
mpu6050_acc_z=0;
}*/
if(mpu6050_gyro_y>-5&&mpu6050_gyro_y<5)
{
mpu6050_gyro_y=0;
}

//坐标轴标定

acc_x = mpu6050_acc_transition(mpu6050_acc_x); //加速度计转化为物理量
acc_z = mpu6050_acc_transition(mpu6050_acc_z);
gyro_y = mpu6050_gyro_transition(mpu6050_gyro_y);//角速度计转化为物理量


AY = -atan2(acc_x,acc_z)* 57.2957795f;//-? 得到加速度计算出的角度
GY = AngleY + gyro_y*t;//得到角速度计算出的角度,t角速度积分，和定时中断同步
AngleY = AlphaPitch * AY + (1 - AlphaPitch) * GY;//互补滤波
*Pitch=AngleY;//赋值给储存Pitch的变量

}




/* 
功能：读取mpu6050数据并进行Pitch姿态解算3 Pitch
参数：储存解算好的Pitch,变量指针
返回值：void
*/
void mpu6050estimation_Pitch3(float*Pitch)
{
mpu6050_get_acc(); //读取加速度计初始数据
mpu6050_get_gyro(); //读取角速度计初始数据 

//数据预处理:去零飘，坐标轴标定：

mpu6050_acc_x-=acc_xbias;//去零飘
mpu6050_acc_z-=acc_zbias;
mpu6050_gyro_y-=gyro_ybias;

//坐标轴标定

acc_x = mpu6050_acc_transition(mpu6050_acc_x); //加速度计转化为物理量
acc_z = mpu6050_acc_transition(mpu6050_acc_z);
gyro_y = mpu6050_gyro_transition(mpu6050_gyro_y);//角速度计转化为物理量


AY = -atan2(acc_x,acc_z)* 57.2957795f;//-? 得到加速度计算出的角度
GY = AngleY + gyro_y*t;//得到角速度计算出的角度,t角速度积分，和定时中断同步
AngleY = AlphaPitch * AY + (1 - AlphaPitch) * GY;//互补滤波
*Pitch=AngleY;//赋值给储存Pitch的变量

}

/* 
功能：读取mpu6050数据并进行姿态解算,三个欧拉角，互补滤波
参数：储存解算好的Pitch,roll,Yaw角的变量指针
返回值：void
*/
void mpu6050estimation(float*Pitch,float*Roll,float*Yaw)
{
mpu6050_get_acc(); //读取加速度计初始数据
mpu6050_get_gyro(); //读取角速度计初始数据 

acc_x=(float)mpu6050_acc_x;
acc_y=(float)mpu6050_acc_y;
acc_z=(float)mpu6050_acc_z;
gyro_x=(float)mpu6050_gyro_x;
gyro_y=(float)mpu6050_gyro_y;
gyro_z=(float)mpu6050_gyro_z;

//数据预处理:滤波，去零飘，坐标轴标定：
//fliter(&acc_x,0.8);
//fliter(&acc_y,0.8);
//fliter(&acc_z,0.8);
//fliter(&gyro_x,0.8);
//fliter(&gyro_y,0.8);
//fliter(&gyro_z,0.8);

acc_x = mpu6050_acc_transition(mpu6050_acc_x); //三轴加速度计转化为物理量acc_y = mpu6050_acc_transition(mpu6050_acc_y);acc_z = mpu6050_acc_transition(mpu6050_acc_z);gyro_x = mpu6050_gyro_transition(mpu6050_gyro_x); //三轴角速度计转化为物理量gyro_y = mpu6050_gyro_transition(mpu6050_gyro_y);gyro_z = mpu6050_gyro_transition(mpu6050_gyro_z);

AY = -atan2(acc_x,acc_z)* 57.2957795f;//-? 得到加速度计算出的角度GY = AngleY + gyro_y*t;//得到角速度计算出的角度,t角速度积分，和定时中断同步AngleY = AlphaPitch * AY + (1 - AlphaPitch) * GY;//互补滤波*Pitch=AngleY;//赋值给储存Pitch的变量
AX = -atan2(acc_y,acc_z)* 57.2957795f;//-?GX = AngleX + gyro_x*t;AngleX = AlphaX * AX + (1 - AlphaX) * GX;*Roll=AngleX;//Roll get！
AngleZ += gyro_z*t;*Yaw=AngleZ;//Yaw get！//AngleZ = Alpha * AZ + (1 - Alpha) * GZ;
}

/*功能：计算Pitch要用的500次mpu6050数据取平均值：参数：储存校准值的变量地址返回值：无*///void sum_Pitch(int16_t*sacc_x,int16_t*sacc_z,int16_t*sgyro_y){int64_t sumacc_x=0,sumacc_z=0,sumgyro_y=0;for(int16_t i=0;i<500;i++){mpu6050_get_acc(); //读取加速度计初始数据mpu6050_get_gyro(); //读取角速度计初始数据 sumacc_x+=mpu6050_acc_x;;sumacc_z+=mpu6050_acc_z;sumgyro_y+=gyro_y;*sacc_x=sumacc_x/500;*sacc_z=sumacc_z/500;*sgyro_y=sumgyro_y/500;}printf("OK");}
/*功能：计算500次mpu6050数据取平均值：参数：储存校准值的变量地址返回值：无*/
//void sum(int16_t*sacc_x,int16_t*sacc_y,int16_t*sacc_z,int16_t*sgyro_x,int16_t*sgyro_y,int16_t*sgyro_z){int64_t sumacc_x=0,sumacc_y=0,sumacc_z=0,sumgyro_x=0,sumgyro_y=0,sumgyro_z=0;for(int16_t i=0;i<500;i++){mpu6050_get_acc(); //读取加速度计初始数据mpu6050_get_gyro(); //读取角速度计初始数据 acc_x = mpu6050_acc_transition(mpu6050_acc_x); //三轴加速度计转化为物理量acc_y = mpu6050_acc_transition(mpu6050_acc_y);acc_z = mpu6050_acc_transition(mpu6050_acc_z);gyro_x = mpu6050_gyro_transition(mpu6050_gyro_x); //三轴角速度计转化为物理量gyro_y = mpu6050_gyro_transition(mpu6050_gyro_y);gyro_z = mpu6050_gyro_transition(mpu6050_gyro_z);sumacc_x+=acc_x;sumacc_y+=acc_y;sumacc_z+=acc_z;sumgyro_x+=gyro_x;sumgyro_y+=gyro_y;sumgyro_z+=gyro_z;*sacc_x=sumacc_x/500;*sacc_y=sumacc_y/500;*sacc_z=sumacc_z/500;//*sgyro_x=}printf("OK");}
	
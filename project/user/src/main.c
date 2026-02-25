/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MM32F327X-G8P 开源库的一部分
* 
* MM32F327X-G8P 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-08-10        Teternal            first version
********************************************************************************************************************/
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库移植用空工程


#include "zf_common_headfile.h"
#include <math.h>
#include <stdint.h>
#include "menu.h"
#include "key.h"
#include "flash.h"
#include "Encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "Mahony.h"
#include "pid.h"
#include "bluetooth.h"
#include "mode5.h"
#include "mode2.h"

#define DEG_TO_RAD 0.017453292519943295f  // π/180
#define RAD_TO_DEG 57.29577951308232f     // 180/π  

volatile uint16_t pit_cnt = 0;

extern soft_iic_info_struct mpu6050_iic_struct;
extern int16_t ax,az,gy,flag_mpu;//互补滤波中间量
extern int16_t acc_xbias,acc_ybias,acc_zbias,gyro_xbias,gyro_ybias,gyro_zbias;//零飘校准
extern float gyro_y1,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,AX,AY,AZ,GX,GY,GZ,AngleX,AngleY,AngleZ;;//数据处理中间量
extern volatile float q0 , q1, q2, q3 ;	
extern float ax1,ay1,az1,gx1,gy1,gz1;

float Pitch=0,Roll=0,Yaw=0;
float encoderleft,encoderright;
int16_t f=1;
float LeftSpeed,RightSpeed;
int16_t LeftPWM, RightPWM;
int16_t AvePWM, DifPWM=0;
int16_t AveSpeed, DifSpeed;
extern float AngleY;

PID_t wPID = {
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
	.Target=0,
	.OutMax = 100,
	.OutMin = -100,
};


PID_t AnglePID = {
	.Kp = 0,
	.Ki = 0,
	.Kd = 2,
	.Target=0,
	.OutMax = 100,
	.OutMin = -100,
};
PID_t SpeedPID = {
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
	.OutMax = 20,
	.OutMin = -20,
	.Target=0,
};
PID_t TurnPID = {
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
	.OutMax = 50,
	.OutMin = -50,
};

// 系统毫秒级时间函数（适配逐飞库，解决uint16_t溢出问题，永不卡死）
uint32_t system_get_time_ms(void)
{
    // 声明pit_cnt为外部变量（TIM6中断里的1ms计数变量）
    extern volatile uint16_t pit_cnt;
    static uint32_t ms_count = 0;    // 用32位存总毫秒数，不会溢出
    static uint16_t pit_last = 0;
    
    // 核心：处理pit_cnt的16位溢出（归零）情况
    uint16_t pit_current = pit_cnt;  // 先读当前值，避免中断中被修改
    
    // 情况1：正常计数（没有溢出）
    if(pit_current >= pit_last)
    {
        ms_count += (pit_current - pit_last);
    }
    // 情况2：pit_cnt溢出归零（比如从65535→0）
    else
    {
        // 先加：从last到65535的差值 + 从0到current的差值
        ms_count += (65535 - pit_last + 1 + pit_current);
    }
    pit_last = pit_current;  // 更新上次值
    
    return ms_count;
}

// **************************** 代码区域 ****************************
int main(void)
{
    // ===================== 核心修改1：调整初始化顺序 - MPU最后初始化 =====================
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
    pit_ms_init(PIT, 1);                                                        // 初始化 PIT（TIM6_PIT） 为周期中断 1ms 周期
    interrupt_set_priority(PIT_PRIORITY, 0);
    
    // 先初始化菜单/按键（避免后续抢引脚）
    my_key_init();
    timer_key();  // 开启按键中断（已调低优先级，不抢占IIC）
    menu_init();
    // menu_load();
    
    // 初始化其他外设
    bluetooth_ch9141_init();
    Encoder_Init();
    Motor_Init();
    PID_Init(&AnglePID);
    PID_Init(&SpeedPID);
    PID_Init(&TurnPID);
    PID_Init(&wPID);
    
    // ===================== 核心修改2：MPU6050最后初始化，避免被其他外设抢占资源 =====================
    mpu6050_init();
    
    // ===================== 核心修改3：初始化完成锁 + 延时稳定 =====================
    uint8_t system_init_ok = 0;
    system_delay_ms(200);  // 给MPU足够的初始化时间，避免断言
    system_init_ok = 1;    // 初始化完成，允许运行菜单代码

    while(1)
    {
        // 1. 优先读取MPU数据（核心，不能被挤占）
        mpu6050_get_acc(); //读取加速度计初始数据
        mpu6050_get_gyro(); //读取角速度计初始数据 

        // 2. 核心：仅初始化完成后，低频处理菜单/按键（50ms一次，避免抢占IIC）
        static uint32_t last_menu_time = 0;
        if(system_init_ok && (system_get_time_ms() - last_menu_time >= 50))
        {
//            menu_save();   // 低频保存参数
            menu_key();    // 低频处理按键
            last_menu_time = system_get_time_ms(); // 更新时间戳
        }
		
		// ✅ 【唯一安全的printf位置】
		static uint32_t last_print_time = 0;
		if(system_get_time_ms() - last_print_time >= 100)  // 100ms 打一次
		{
			// 这里随便 printf
			// 3. 姿态解算相关打印（保留你的原有代码）
//			 printf("[plot,%d]",ax);//1
//			 printf("[plot,%f,%f]",-AY,Pitch);//2
//			 printf("[plot,%f,%f,%f]",-AY,Pitch,-GY);//3
//			 printf("[plot,%f,%f]",-atan2(ax1,az1)* 180.0f / 3.14159265f,Pitch);//4

			last_print_time = system_get_time_ms();
		}

        // 4. 屏幕只显示核心角度，减少刷屏
        tft180_show_float(0, 90,-Pitch , 2,2);

        // 5. 调用mode5，保留原有逻辑
        mode5(&SpeedPID,&TurnPID);

        // 6. 短延时，避免空转（从8ms减到1ms，减少延迟）
        system_delay_ms(1);
    }
}
// **************************** 代码区域 ****************************
/*
		此为编码器的中断，isr.c中见tim6
*/
int cnt=0;
int cnt1=0;
int cnt2=0;
int cnt3=0;
void pit_handler (void)
{	
	flag_mpu--;
	cnt++;
	cnt1++;
	cnt2++;
	cnt3++;
	if(cnt==10)
	{
		mpu6050estimation_Pitch(&Pitch);  //姿态解算A
		
		
//		pre_mahony();						//B
//		MahonyAHRSupdateIMU2(gx1*DEG_TO_RAD,gy1*DEG_TO_RAD,gz1*DEG_TO_RAD,ax1,ay1,az1);
//		get_angles_from_quaternion(q0,q1,q2,q3,&Roll,&Pitch,&Yaw);
		
		
//		//===角速度环
		
		wPID.Actual=gyro_y1;
		PID_Update(&wPID);
		AvePWM = -wPID.Out;
		LeftPWM = AvePWM+DifPWM/2;
		RightPWM = AvePWM-DifPWM/2;
		if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
		if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}
		Motor_SetSpeedleft(LeftPWM*100);
		Motor_SetSpeedright(RightPWM*100);
		cnt=0;
//		//===fin
	}
	if(cnt1==20){
		cnt1=0;
		AnglePID.Actual = Pitch;			//角度环pid
		PID_Update(&AnglePID);

//	===角速度环
		wPID.Target =	AnglePID.Out;	
		
//		AvePWM = -AnglePID.Out;
//		LeftPWM = AvePWM+DifPWM/2;
//		RightPWM = AvePWM-DifPWM/2;
//		if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
//		if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}
//		Motor_SetSpeedleft(LeftPWM*100);
//		Motor_SetSpeedright(RightPWM*100);
		
		
	}
	if(cnt2==50)//速度环pid && 角度环pid
	{
		cnt2=0;
		LeftSpeed = Get_Encoder_Data_Left()/13.0/34/0.05;	//公式：编码器值/一圈计数值/减速比/周期（单位：转/秒）  
		RightSpeed = Get_Encoder_Data_Right()/13.0/34/0.05;
		AveSpeed=(LeftSpeed+RightSpeed)/2.0;
		DifSpeed=LeftSpeed-RightSpeed;
		SpeedPID.Actual=AveSpeed;
		PID_Update(&SpeedPID);
		AnglePID.Target=SpeedPID.Out;
		
		TurnPID.Actual=DifSpeed;
		PID_Update(&TurnPID);
		DifPWM=TurnPID.Out;
	}	
//公式：编码器值/一圈计数值/减速比/周期（单位：转/秒）  
	if(cnt3==50){
	    encoder_clear_count(ENCODER_QUADDEC_L);                                       // 清空编码器计数
	    encoder_clear_count(ENCODER_QUADDEC_R);
		cnt3=0;		
	}
}
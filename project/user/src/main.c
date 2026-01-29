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

#include "zf_common_headfile.h"
#include "Encoder.h"
#include "bluetooth.h"
#include "mpu6050.h"
#include "pid.h"
#include "motor.h"
#include "menu.h"
#include "key.h"
#include "flash.h"
extern int16_t acc_xbias,acc_ybias,acc_zbias,gyro_xbias,gyro_ybias,gyro_zbias;//零飘校准
extern float gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z;//数据处理中间量
float Pitch;
int16_t f=1;
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库移植用空工程
float LeftSpeed,RightSpeed;
int16_t LeftPWM, RightPWM;
int16_t AvePWM, DifPWM;
int16_t AveSpeed, DifSpeed;
extern float AngleY;
PID_t AnglePID = {
	.Kp = 1,
	.Ki = 0,
	.Kd = 0,
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
// **************************** 代码区域 ****************************
int main(void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
	pit_ms_init(PIT, 1);                                                      // 初始化 PIT（TIM6_PIT） 为周期中断 1ms 周期
	interrupt_set_priority(PIT_PRIORITY, 0);
	bluetooth_ch9141_init();
	Motor_Init();
	mpu6050_init();
	my_key_init();
	timer_key();
	menu_init();
	menu_load();
	Pitch=0;
	PID_Init(&AnglePID);
	PID_Init(&SpeedPID);
	bluetooth_ch9141_init();
//	Motor_SetSpeedleft(0);
//	Motor_SetSpeedright(0);
	// 设置 PIT 对周期中断的中断优先级为 0
    // 此处编写用户代码 例如外设初始化代码等
//    	Motor_SetSpeedright(7000);
//			Motor_SetSpeedleft(7000);
    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
		//printf("%f\n",Pitch);
		menu_save();
		menu_key();
		menu_save();
//		printf("[plot,%f]",-Pitch-2);
		tft180_show_int(0, 110,-Pitch-2 , 3);
		//tft180_show_int(0, 30,acc_z , 3); 
        // 此处编写需要循环执行的代码
//		Motor_SetSpeedright(1000);
//		Motor_SetSpeedleft(1000);
        // 此处编写需要循环执行的代码
    }
}
// **************************** 代码区域 ****************************
/*
		此为编码器的中断，isr.c中见tim6
*/
int cnt=0;
int cnt1=0;
int cnt2=0;
void pit_handler (void)
{	
	
	cnt++;
	cnt1++;
	cnt2++;
	if(cnt==10)
	{
		mpu6050estimation_Pitch(&Pitch);  
		cnt=0;
	}
	if(cnt1==20){
		AnglePID.Actual = -Pitch-2;
		PID_Update(&AnglePID);
		AvePWM = -AnglePID.Out;
		LeftPWM = AvePWM;
		RightPWM = AvePWM;
		if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
		if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}
		Motor_SetSpeedleft(LeftPWM*100);
		Motor_SetSpeedright(RightPWM*100);
		cnt1=0;
	}
	if(cnt2==50)
	{
		cnt2=0;
		LeftSpeed = Get_Encoder_Data_Left()/52/0.05/34;//公式：编码器值/一圈计数值/减速比/周期（单位：转/秒）  
		RightSpeed = Get_Encoder_Data_Right()/52/0.05/34;	
		AveSpeed=(LeftSpeed+RightSpeed)/2.0;
		DifSpeed=LeftSpeed-RightSpeed;
		SpeedPID.Actual=AveSpeed;
		PID_Update(&SpeedPID);
	}
	/*
    encoderleft = Get_Encoder_Data_Left();              // 获取编码器计数（并非标准单位）
    encoderright = Get_Encoder_Data_Right();            // 获取编码器计数（并非标准单位）
		
			如要获得转/秒，请用此公式
			encoderleft = Get_Encoder_Data_Left()/52/0.01/34;		//公式：编码器值/一圈计数值/减速比/周期（单位：转/秒）  
			encoderright = Get_Encoder_Data_Right()/52/0.01/34;	
	
    encoder_clear_count(ENCODER_QUADDEC_L);                                       // 清空编码器计数
    encoder_clear_count(ENCODER_QUADDEC_R); 
*/
// 清空编码器计数
}
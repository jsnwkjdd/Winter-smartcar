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

// 全局变量声明
extern soft_iic_info_struct mpu6050_iic_struct;
extern int16_t ax, az, gy, flag_mpu;
extern int16_t acc_xbias, acc_ybias, acc_zbias, gyro_xbias, gyro_ybias, gyro_zbias;
extern float gyro_y1, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, AX, AY, AZ, GX, GY, GZ, AngleX, AngleY, AngleZ;
extern volatile float q0 , q1, q2, q3 ;	
extern float ax1, ay1, az1, gx1, gy1, gz1;

float Pitch=0, Roll=0, Yaw=0;
float encoderleft, encoderright;
int16_t f=1;
float LeftSpeed, RightSpeed;
int16_t LeftPWM, RightPWM;
int16_t AvePWM, DifPWM=0;
int16_t AveSpeed, DifSpeed;

// PID 参数（基础可用值，可根据实际调试）
PID_t wPID = {
    .Kp = -0.1,
    .Ki = 0.01,
    .Kd = 0.05,
    .Target = 0,
    .OutMax = 100,
    .OutMin = -100,
};

PID_t AnglePID = {
    .Kp = 2.0,
    .Ki = 0.05,
    .Kd = -0.2,
    .Target = 0,
    .OutMax = 100,
    .OutMin = -100,
};

PID_t SpeedPID = {
    .Kp = 1.0,
    .Ki = 0.02,
    .Kd = 0.1,
    .OutMax = 20,
    .OutMin = -20,
    .Target = 0,
};

PID_t TurnPID = {
    .Kp = 1.5,
    .Ki = 0.01,
    .Kd = 0.05,
    .OutMax = 50,
    .OutMin = -50,
};

// 中断计数（全局变量，避免重复定义）
volatile uint16_t pit_cnt = 0;

// PIT中断处理函数（轻量化改造）
void pit_handler (void)
{	
    pit_cnt++;
    flag_mpu--;

    // 每10ms：姿态解算 + 角速度环PID + 电机控制
    if(pit_cnt % 10 == 0)
    {
        mpu6050estimation_Pitch(&Pitch);  // 姿态解算
        wPID.Actual = gyro_y1;
        wPID.Target = AnglePID.Out;
        PID_Update(&wPID);
        
        AvePWM = -wPID.Out;
        LeftPWM = AvePWM + DifPWM/2;
        RightPWM = AvePWM - DifPWM/2;
        
        // PWM限幅（简化逻辑）
        if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
		if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}
        
        Motor_SetSpeedleft(LeftPWM*400);
        Motor_SetSpeedright(RightPWM*400);
    }

    // 每20ms：角度环PID
    if(pit_cnt % 20 == 0)
    {
        AnglePID.Actual = Pitch;
        PID_Update(&AnglePID);
        AnglePID.Target = SpeedPID.Out;
    }

    // 每50ms：速度环PID + 转向PID + 编码器清零
    if(pit_cnt % 50 == 0)
    {
        // 速度计算
        LeftSpeed = Get_Encoder_Data_Left()/13.0/34/0.05;
        RightSpeed = Get_Encoder_Data_Right()/13.0/34/0.05;
        AveSpeed = (LeftSpeed + RightSpeed)/2.0;
        DifSpeed = LeftSpeed - RightSpeed;
        
        // 速度环PID
        SpeedPID.Actual = AveSpeed;
        PID_Update(&SpeedPID);
        
        // 转向PID
        TurnPID.Actual = DifSpeed;
        PID_Update(&TurnPID);
        DifPWM = TurnPID.Out;
        
        // 编码器清零
        encoder_clear_count(ENCODER_QUADDEC_L);
        encoder_clear_count(ENCODER_QUADDEC_R);
    }

    // 计数溢出重置
    if(pit_cnt >= 1000) pit_cnt = 0;
}

// 主函数（核心：初始化只执行一次，主循环轻量化）
int main(void)
{
    // 1. 基础时钟/中断初始化
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 120MHz
    debug_init();                                                               // 初始化Debug UART
    pit_ms_init(PIT, 1);                                                        // 初始化PIT 1ms周期中断
    interrupt_set_priority(PIT_PRIORITY, 1);                                    // 降低中断优先级，避免抢占所有资源

    // 2. 外设初始化（只执行一次）
    my_key_init();
    timer_key();
    menu_init();
    bluetooth_ch9141_init();
    Encoder_Init();
    Motor_Init();

    // 3. MPU6050初始化（IIC只初始化一次，通信失败时在解算函数里重试）
    soft_iic_init(&mpu6050_iic_struct, MPU6050_DEV_ADDR, MPU6050_SOFT_IIC_DELAY, MPU6050_SCL_PIN, MPU6050_SDA_PIN);
    mpu6050_init();                                                             // MPU6050初始化

    // 4. PID初始化
    PID_Init(&AnglePID);
    PID_Init(&SpeedPID);
    PID_Init(&TurnPID);
    PID_Init(&wPID);

    // 5. 电机初始状态
    Motor_SetSpeedleft(0);
    Motor_SetSpeedright(0);

    // 主循环（只处理低速逻辑）
    while(1)
    {
        system_delay_ms(8);
        menu_key();        // 按键处理
        menu_save();       // 菜单保存
        mode5(&SpeedPID,&TurnPID);  // 模式控制
        tft180_show_float(0, 90, -Pitch, 2, 2);  // 屏幕显示角度
		
//		printf("[plot,%d]",az);//1
		
//		printf("[plot,%f,%f]",-AY,Pitch);//2
		
		printf("[plot,%f,%f,%f]",-AY,Pitch,-GY);//3
	
//		printf("[plot,%f,%f]",-atan2(ax1,az1)* 180.0f / 3.14159265f,Pitch);//4
		system_delay_ms(50);
    }
}
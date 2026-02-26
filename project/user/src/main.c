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

// ===================== 全局变量重构（解决中断访问问题）=====================
volatile uint16_t pit_cnt = 0;
volatile uint8_t mpu_10ms_flag = 0;
volatile uint32_t mpu_10ms_cnt = 0;
uint8_t system_init_ok = 0; // 移到全局，中断能访问！

// MPU相关外部变量
extern soft_iic_info_struct mpu6050_iic_struct;
extern int16_t ax,az,gy,flag_mpu;
extern int16_t acc_xbias,acc_ybias,acc_zbias,gyro_xbias,gyro_ybias,gyro_zbias;
extern float gyro_y1,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,AX,AY,AZ,GX,GY,GZ,AngleX,AngleY,AngleZ;
extern volatile float q0 , q1, q2, q3 ;	
extern float ax1,ay1,az1,gx1,gy1,gz1;

// 姿态和运动变量
float Pitch=0,Roll=0,Yaw=0;
float encoderleft,encoderright;
int16_t f=1;
float LeftSpeed,RightSpeed;
int16_t LeftPWM, RightPWM;
int16_t AvePWM, DifPWM=0;
int16_t AveSpeed, DifSpeed;
extern float AngleY;

// ===================== PID参数（最终稳定版：单环角度PID+角速度阻尼）=====================
// 角速度阻尼（保留你最完美的参数，只负责消抖，不做主环）
PID_t wPID = {
	.Kp =-0.003,
	.Ki = 0,
	.Kd = 0.0008,
	.Target=0,
	.OutMax = 20,    // 缩小阻尼输出范围，避免干扰
	.OutMin = -20,
};

// 角度环（直接控电机，够力、不抖、不漂移）
PID_t AnglePID = {
	.Kp = -1.8,        // 核心：加大角度环力度，直接扶车
	.Ki = 0,
	.Kd = 0.3,         // 加大微分消抖
	.Target=0,
	.OutMax = 50,   
	.OutMin = -50,
};

// 速度环（暂时关闭，避免干扰平衡）
PID_t SpeedPID = {
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
	.OutMax = 0,    
	.OutMin = 0,
	.Target=0,
};

// 转向环（暂时关闭，先站稳）
PID_t TurnPID = {
	.Kp = 0,
	.Ki = 0,
	.Kd = 0,
	.OutMax = 0,
	.OutMin = 0,
};

// 系统毫秒级时间函数（适配逐飞库）
uint32_t system_get_time_ms(void)
{
    extern volatile uint16_t pit_cnt;
    static uint32_t ms_count = 0;    
    static uint16_t pit_last = 0;
    
    uint16_t pit_current = pit_cnt;
    
    if(pit_current >= pit_last)
    {
        ms_count += (pit_current - pit_last);
    }
    else
    {
        ms_count += (65535 - pit_last + 1 + pit_current);
    }
    pit_last = pit_current;
    
    return ms_count;
}

// ===================== 主函数（重构初始化顺序+逻辑）=====================
int main(void)
{
    // 1. 基础初始化（先开时钟，再开中断）
    clock_init(SYSTEM_CLOCK_120M);
    debug_init();
    pit_ms_init(PIT, 1);                // 1ms中断
    interrupt_set_priority(PIT_PRIORITY, 2); // 降低中断优先级，不抢屏幕
    
    // 2. 初始化无冲突外设（先不碰MPU和屏幕）
    my_key_init();
    timer_key();
    bluetooth_ch9141_init();
    Encoder_Init();
    Motor_Init();
    
    // 3. 初始化PID
    PID_Init(&AnglePID);
    PID_Init(&SpeedPID);
    PID_Init(&TurnPID);
    PID_Init(&wPID);
    
    // 4. 最后初始化MPU（避免被抢占）
    mpu6050_init();
    system_delay_ms(300); // 给MPU足够校准时间
    
    // 5. 初始化屏幕（此时中断优先级低，不会卡死）
    menu_init();
    // menu_load(); // 先注释，避免初始化负载过高
    
    // 6. 所有初始化完成，解锁！
    system_init_ok = 1;
    
    // 主循环变量
    static uint32_t last_menu_time = 0;
    static uint32_t last_print_time = 0;

    while(1)
    {
        // ========== 10ms姿态解算（主循环，仅打标后执行）==========
        if(mpu_10ms_flag && system_init_ok)
        {
            mpu_10ms_flag = 0;
        }

        // ========== 低频处理菜单/按键（50ms一次，不卡）==========
        if(system_init_ok && (system_get_time_ms() - last_menu_time >= 50))
        {
            menu_key();
            last_menu_time = system_get_time_ms();
        }

        // ========== 低频打印（100ms一次，不占资源）==========
        if(system_get_time_ms() - last_print_time >= 100)
        {
            tft180_show_float(0, 90, -Pitch, 2, 2); // 显示Pitch
            last_print_time = system_get_time_ms();
        }

        // ========== 调用模式逻辑（保留）==========
        mode5(&SpeedPID,&TurnPID);

        // ========== 短延时，降低CPU占用 ==========
        system_delay_ms(1);
    }
}

// ===================== PIT中断函数（核心修改：单环角度PID直接控电机）=====================
int cnt1=0,cnt2=0,cnt3=0;
void pit_handler (void)
{	
    pit_cnt++;
	flag_mpu--;
	mpu_10ms_cnt++;
	cnt1++;
	cnt2++;
	cnt3++;
	
	// ========== 1. 10ms姿态解算 ==========
	if(mpu_10ms_cnt >= 10)
    {
        mpu_10ms_cnt = 0;
		// 读MPU数据
		mpu6050_get_acc();
		mpu6050_get_gyro();
		// 姿态解算（稳定执行）
		mpu6050estimation_Pitch(&Pitch);
        mpu_10ms_flag = 1;
    }

    // 10ms 控制
	if(cnt1>=10 && system_init_ok)
	{
		cnt1=0;

		// 角度环
		AnglePID.Actual = Pitch;
		PID_Update(&AnglePID);

		// 串级：角度环输出 → 角速度环目标
		wPID.Target = AnglePID.Out * 6;   // 放大 6 倍，温和
		wPID.Actual = gyro_y;
		PID_Update(&wPID);

		// 直接输出，不加软启动、不加最小力（那些是坑）
		AvePWM = -(int16_t)wPID.Out;

		// 限幅
		if(AvePWM > 50) AvePWM = 50;
		if(AvePWM < -50) AvePWM = -50;

		LeftPWM  = AvePWM;
		RightPWM = AvePWM;

		// 电机输出
		Motor_SetSpeedleft(LeftPWM * 350);
		Motor_SetSpeedright(RightPWM * 350);
	}
    // ========== 3. 速度环/转向环暂时关闭（先站稳）==========
	if(cnt2==50 && system_init_ok)
	{
		cnt2=0;
		// 清空编码器（保留，但不计算速度）
		LeftSpeed = 0;
		RightSpeed = 0;
		AveSpeed = 0;
		DifSpeed = 0;
		SpeedPID.Out = 0;
		TurnPID.Out = 0;
		DifPWM = 0;
	}	
	
    // ========== 4. 50ms清空编码器 ==========
	if(cnt3==50)
	{
	    encoder_clear_count(ENCODER_QUADDEC_L);                                       
	    encoder_clear_count(ENCODER_QUADDEC_R);
		cnt3=0;		
	}
}
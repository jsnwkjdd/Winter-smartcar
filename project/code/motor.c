#include "zf_common_headfile.h"
#include "motor.h"
/*
初始化pwm，对应端口为a1和a3
并且初始化ain1和ain2，bin1和bin2
*/
void Motor_Init(void) 
{
	pwm_init(PWM_CH2, 17000, 0);  
	pwm_init(PWM_CH4, 17000, 0);  
	gpio_init(C0, GPO, GPIO_HIGH, GPO_PUSH_PULL); 
  gpio_init(C1, GPO, GPIO_HIGH, GPO_PUSH_PULL); 
	gpio_init(A0, GPO, GPIO_HIGH, GPO_PUSH_PULL); 
  gpio_init(A2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
}
void Motor_SetSpeedleft(int16_t Speed)
{
		if(Speed > 8000)Speed= 8000;
		if(Speed < -8000)Speed = -8000;  
		if(Speed > 8000)Speed= 8000;
		if(Speed < -8000)Speed = -8000;  
		if (Speed >= 0)							//如果设置正转的速度值
	{
		gpio_set_level(C0, GPIO_HIGH);	//c0置高电平
		gpio_set_level(C1, GPIO_LOW);	//c1置低电平，设置方向为正转
		pwm_set_duty(TIM5_PWM_CH2_A1, Speed); 				//PWM设置为速度值
	}
	else									//否则，即设置反转的速度值
	{Speed=-Speed;
		gpio_set_level(C1, GPIO_HIGH);	//c1置高电平
		gpio_set_level(C0, GPIO_LOW);		//c0置低电平，设置方向为反转
		pwm_set_duty(TIM5_PWM_CH2_A1, -Speed);			//PWM设置为负的速度值，因为此时速度值为负数，而PWM只能给正数
	}
}
void Motor_SetSpeedright(int16_t Speed)
{
		if(Speed > 8000)Speed= 8000;
		if(Speed < -8000)Speed = -8000;  
		if(Speed > 8000)Speed= 8000;
		if(Speed < -8000)Speed = -8000;  
		if (Speed >= 0)							//如果设置正转的速度值
	{
		gpio_set_level(A2, GPIO_HIGH);	//c2置高电平
		gpio_set_level(A0, GPIO_LOW);	//c3置低电平，设置方向为正转
		pwm_set_duty(TIM5_PWM_CH4_A3, Speed); 				//PWM设置为速度值
	}
	else									//否则，即设置反转的速度值
	{	
		gpio_set_level(A0, GPIO_HIGH);	//c3置高电平
		gpio_set_level(A2, GPIO_LOW);		//c2置低电平，设置方向为反转
		pwm_set_duty(TIM5_PWM_CH4_A3, -Speed);			//PWM设置为负的速度值，因为此时速度值为负数，而PWM只能给正数
	}
}
#include "sensor.h"
#include "beep.h"
#include "LED.h"
#include "PID.h"
extern PID_t TurnPID,SpeedPID;
uint8_t B_State,count_B,flag_beep_led=0;
uint8_t mode2(void)
{
	if(sensor()==0)//如果初始位置在白线，进入题目模式，并计次4
	{
		B_State=2;
		count_B=4;
	}
	else if(sensor()!=0)//如果初始位置在黑线，进入循迹模式，并计次5
	{
		B_State=1;
		count_B=5;
	}
	while(1)
	{
		if(count_B>0)//计次未到，run
		{
			if(B_State==1)
			{
				count_B--;
				control();//循迹
				if(sensor()==0)//检测到全白线时，切换至题目模式
				{
				
					B_State=2;
					beep_on();					//蜂鸣器+led提示
					LED_up();
					system_delay_ms(500);
					beep_off();
					LED_down();
				}
			}
			else if(B_State==2)
			{
				count_B--;
			//======题目======
				TurnPID.Target=0;		//转向环角度为零,要不要补一个角速度呢
				SpeedPID.Target=20;			//给一个速度	


				
				if(sensor()!=0)//检测到黑线时，切换至循迹模式
				{
					B_State=1;
					beep_on();//蜂鸣器+led提示
					LED_up();
					system_delay_ms(500);
					beep_off();
					LED_down();
				}
			}
		
		}
		else //计次结束，停止
		{return 2;}
	}
}
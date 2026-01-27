
#include "sensor.h"
uint8_t B_State,count_B;
uint8_t qestionB(void)
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
		if(count_B>0)
		{
			if(B_State==1)
			{
				count_B--;
				control();//循迹
				if(sensor()==0)//检测到全白线时，切换至题目模式
				{B_State=2;}
			}
			else if(B_State==2)
			{
				count_B--;
				//题目
				if(sensor()!=0)//检测到黑线时，切换至循迹模式
				{B_State=1;}
			}
		
		}
		else 
		{return 2;}
	}
}
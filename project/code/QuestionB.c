
#include "sensor.h"
uint8_t S;
void qestionB(void)
{
	while(1)
	{
		if(S==1)
		{
			control();//循迹
			if(sensor()==0)//检测到全白线时，切换至题目模式
			{S=2;}
		}
		else if(S==2)
		{
			//题目
			if(sensor()!=0)//检测到黑线时，切换至循迹模式
			{S=1;}
		}

	}
}
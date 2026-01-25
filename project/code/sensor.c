#include "zf_driver_gpio.h"
/*
功能：端口初始化：上拉输入模式
参数/返回值：无
*/
void sensor_init()
{
	gpio_init(D5, GPI, GPIO_HIGH, GPI_PULL_UP);//上拉输入
	gpio_init(D6, GPI, GPIO_HIGH, GPI_PULL_UP);
	gpio_init(B2, GPI, GPIO_HIGH, GPI_PULL_UP);
	gpio_init(B3, GPI, GPIO_HIGH, GPI_PULL_UP);
}

/*
功能：以下为红外传感器读取，读取点平，并转换为位掩码
备注：内部调用(用不上，看下面那个函数)+但是要记得看接线来改改引脚对应关系
返回值：位掩码
参数：void

*/
uint8_t IRSensorL1_Get(void)//检测红外左一
{
	if(gpio_get_level(D5)==1)//看接线来改改引脚对应关系
	{return 0x08;}
	else 
	{return 0x00;}	
}

uint8_t IRSensorL2_Get(void)//左二
{
	if(gpio_get_level(D6)==1)//看接线来改改引脚对应关系
	{return 0x04;}
	else 
	{return 0x00;}
}

uint8_t IRSensorR2_Get(void)//右二（中间的）
{
	if(gpio_get_level(B2)==1)
	{return 0x02;}
	else 
	{return 0x00;}
}

uint8_t IRSensorR1_Get(void)//右一
{
	if(gpio_get_level(B3)==1)
	{return 0x01;}
	else 
	{return 0x00;}
}

/*
功能：获取红外传感器状态
1代表黑线，0白
0110代表中间两路为黑色，外边两路为白色
参数：void
返回值：int16_t，结合了位掩码，用的时候看一下这个函数下面那个注释
*/

uint8 sensor(void)
{
	uint8_t state;
	state=IRSensorL1_Get()|IRSensorL2_Get()|IRSensorR2_Get()|IRSensorR1_Get();
	return state;
}
/*
1代表黑线，0白
对应表：
0	0000	
1	0001	
2	0010		
3	0011	
4	0100	
5	0101	
6	0110	
7	0111	
8	1000	
9	1001	
10	1010	
11	1011	
12	1100	
13	1101	
14	1110	
15	1111	
state=sensor();
switch (state)//switch运算速度比if快很多
			{
				case 6:		//表明状态为0110 直行
					改轮的速度
					MotorA_SetSpeed(70);
					MotorB_SetSpeed(70);
				break;
				case 2: //0010
					改轮的速度
					
				break;
				case 4://0100
					改轮的速度
					break;
				default:
				break;
*/

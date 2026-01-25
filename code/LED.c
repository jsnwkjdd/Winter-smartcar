#include "zf_common_headfile.h"
#include "LED.h"

//LED初始化函数
void LED_init(void)
{
	 gpio_init(H2, GPO, GPIO_LOW, GPO_PUSH_PULL);                             // 初始化 LED1 输出 默认高电平 推挽输出模式
}

//LED控制灯亮函数
void LED_up(void)
{
	gpio_set_level(H2, GPIO_HIGH);
}

//LED控制灯灭函数
void LED_down(void)
{
	gpio_set_level(H2, GPIO_LOW);
}
#include "beep.h"
#include "zf_common_headfile.h"

//蜂鸣器初始化函数
void beep_init(void){
	gpio_init(BEEP, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

//蜂鸣器定时响函数
void beep_ms(uint16 time){
	gpio_set_level(BEEP, GPIO_HIGH);                                            
	system_delay_ms(time);
	gpio_set_level(BEEP, GPIO_LOW);                                             
}

//蜂鸣器响函数
void beep_on(void){
	gpio_set_level(BEEP, GPIO_HIGH);                                            

}

//蜂鸣器关函数
void beep_off(void){
	gpio_set_level(BEEP, GPIO_LOW);                                             

}
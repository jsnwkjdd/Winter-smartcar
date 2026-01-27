#include "key.h"
#include "zf_common_headfile.h"


// 正确的按键初始化函数
void my_key_init(void)
{
	key_init(10);
}

// 定时中断初始化函数
void timer_key(void)
{
	pit_ms_init(TIM2_PIT, 100); 
	interrupt_set_priority(TIM2_IRQn, 0);	
}

// 扫描按键
void key_scan(void)
{
	key_scanner();
}

//获取按键状态
key_state_enum key_getstate (key_index_enum key_n)
{
   return key_get_state(key_n);
}

//清除对应按键状态
void key_clearstate (key_index_enum key_n)
{
	key_clear_state(key_n);
}

//清除所有按键状态
void key_clear_allstate (void)
{
	key_clear_all_state();
}


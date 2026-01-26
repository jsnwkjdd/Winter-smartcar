#include "key.h"
#include "zf_common_headfile.h"
#include "menu.h"

int32_t mode=0,hang=1,change=0;
float kp=0.0;
float ki=0.0;
float kd=0.0;

//屏幕初始化函数
void menu_init(void)
{
	tft180_set_dir(TFT180_PORTAIT);
	tft180_set_font(TFT180_6X8_FONT);
	tft180_set_color(RGB565_WHITE, RGB565_BLACK);
	tft180_init();
}

//显示函数
void show(void)
{
	tft180_clear();
	if(mode==0)
	{
		tft180_show_string (1,1,hang==1?">mode1":"mode1");
		tft180_show_string (2,1,hang==2?">mode2":"mode2");
		tft180_show_string (3,1,hang==3?">mode3":"mode3");
		tft180_show_string (4,1,hang==4?">mode4":"mode4");
		tft180_show_string (5,1,hang==5?">mode5":"mode5");
	}else if(mode==1)
	{
		tft180_show_string (1,1,hang==1?">ki":"ki");
		tft180_show_string (2,1,hang==2?">kd":"kd");
		tft180_show_string (3,1,hang==3?">kp":"kp");
		tft180_show_uint(1, 5, ki, 3);
		tft180_show_uint(1, 5, kd, 3);
		tft180_show_uint(1, 5, kp, 3);
		tft180_show_string (1,10,change==1?"yes":"no");
	}else if(mode==2)
	{
		
	}else if(mode==3)
	{
		
	}else if(mode==4)
	{
		
	}else if(mode==5)
	{
		
	}
}


//按键检测函数
void menu_key(void)
{
	if(key_getstate(KEY_1)==KEY_MAX_SHOCK_PERIOD)
	{
		key_clear_allstate();
		if(mode==0)
		{
			if(hang==5)hang=1;
			else hang=hang+1;
		}else if(mode==1)
		{
			if(change==0)
			{
				if(hang==3)hang=1;
				else hang=hang+1;
			}else
			{
				if(hang==1)ki++;
				else if(hang==2)kd++;
				else if(hang==3)kp++;
			}	
		}else if(mode==2)
		{
			
		}else if(mode==3)
		{
			
		}else if(mode==4)
		{
			
		}else if(mode==5)
		{
			
		}
	}else if(key_getstate(KEY_2)==KEY_MAX_SHOCK_PERIOD)
	{
		key_clear_allstate();
		if(mode==0)
		{
			if(hang==1)hang=5;
			else hang=hang-1;
		}else if(mode==1)
		{
			if(change==0)
			{
				if(hang==1)hang=3;
				else hang=hang-1;
			}else
			{
				if(hang==1)ki--;
				else if(hang==2)kd--;
				else if(hang==3)kp--;
			}	
		}else if(mode==2)
		{
			
		}else if(mode==3)
		{
			
		}else if(mode==4)
		{
			
		}else if(mode==5)
		{
			
		}
	}else if(key_getstate(KEY_3)==KEY_MAX_SHOCK_PERIOD)
	{
		key_clear_allstate();
		if(mode==0)
		{
			mode=hang;
			hang=1;
		}else if(mode==1)
		{
			if(change==0)change=1;
		}else if(mode==2)
		{
			
		}else if(mode==3)
		{
			
		}else if(mode==4)
		{
			
		}else if(mode==5)
		{
			
		}
	}else if(key_getstate(KEY_4)==KEY_MAX_SHOCK_PERIOD)
	{
		key_clear_allstate();
		if(change==0)
		{
			if(mode!=0)
			{
				mode=0;
				hang=1;
			}
		}else if(change==1)
		{
			if(mode==1)
			{
				change=0;
			}else if(mode==2)
			{
				
			}else if(mode==3)
			{
				
			}else if(mode==4)
			{
				
			}else if(mode==5)
			{
				
			}
		}
	}
	show();
}
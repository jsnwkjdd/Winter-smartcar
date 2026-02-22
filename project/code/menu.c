#include "key.h"
#include "zf_common_headfile.h"
#include "menu.h"
#include "pid.h"
#include "flash.h"
int32_t mode=0,hang=1,change=0;
float kp=0.0;
float ki=0.0;
float kd=0.0;
extern PID_t AnglePID;
extern PID_t SpeedPID;
extern PID_t wPID;
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
//	tft180_clear();
	if(mode==0)
	{
		tft180_show_string (0,0,hang==1?">mode1":"mode1");
		tft180_show_string (0,10,hang==2?">mode2":"mode2");
		tft180_show_string (0,20,hang==3?">mode3":"mode3");
		tft180_show_string (0,30,hang==4?">mode4":"mode4");
		tft180_show_string (0,40,hang==5?">mode5":"mode5");
	}else if(mode==1)
	{
		tft180_show_string (0,0,hang==1?">ki1":"ki1");
		tft180_show_string (0,10,hang==2?">kd1":"kd1");
		tft180_show_string (0,20,hang==3?">kp1":"kp1");
		tft180_show_string (0,30,hang==4?">ki2":"ki2");
		tft180_show_string (0,40,hang==5?">kd2":"kd2");
		tft180_show_string (0,50,hang==6?">kp2":"kp2");
		tft180_show_string (0,60,hang==7?">ki3":"ki3");
		tft180_show_string (0,70,hang==8?">kd3":"kd3");
		tft180_show_string (0,80,hang==9?">kp3":"kp3");
		tft180_show_float(40, 0, AnglePID.Ki, 3,3);
		tft180_show_float(40, 10, AnglePID.Kd, 3,3);
		tft180_show_float(40, 20, AnglePID.Kp, 3,3);
		tft180_show_float(40, 30, SpeedPID.Ki, 3,3);
		tft180_show_float(40, 40, SpeedPID.Kd, 3,3);
		tft180_show_float(40, 50, SpeedPID.Kp, 3,3);
		tft180_show_float(40, 60, wPID.Ki, 3,3);
		tft180_show_float(40, 70, wPID.Kd, 3,3);
		tft180_show_float(40, 80, wPID.Kp, 3,3);
		tft180_show_string (100,0,change==1?"yes":"no");
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
    // 获取按键状态
    key_state_enum key1_state = key_getstate(KEY_1);
    key_state_enum key2_state = key_getstate(KEY_2);
    key_state_enum key3_state = key_getstate(KEY_3);
    key_state_enum key4_state = key_getstate(KEY_4);
    
    // 调试输出（可选）
    // printf("Key states: %d %d %d %d\n", key1_state, key2_state, key3_state, key4_state);
    
    // KEY_1：向下/增加
    if(key1_state == KEY_SHORT_PRESS)
    {
		tft180_clear();
        key_clear_allstate();
        if(mode==0)
        {
            if(hang==5) hang=1;
            else hang=hang+1;
        }
        else if(mode==1)
        {
            if(change==0)
            {
                if(hang==9) hang=1;
                else hang=hang+1;
            }
            else
            {
                if(hang==1) AnglePID.Ki+=0.01;
                else if(hang==2) AnglePID.Kd+=0.1;
                else if(hang==3) AnglePID.Kp+=0.1;
				else if(hang==4) SpeedPID.Ki+=0.01;
                else if(hang==5) SpeedPID.Kd+=0.1;
                else if(hang==6) SpeedPID.Kp+=0.1;
				else if(hang==7) wPID.Ki+=0.01;
                else if(hang==8) wPID.Kd+=0.1;
                else if(hang==9) wPID.Kp+=0.001;
            }	
        }
    }
    // KEY_2：向上/减少
    else if(key2_state == KEY_SHORT_PRESS)
    {
		tft180_clear();
        key_clear_allstate();
        if(mode==0)
        {
            if(hang==1) hang=5;
            else hang=hang-1;
        }
        else if(mode==1)
        {
            if(change==0)
            {
                if(hang==1) hang=9;
                else hang=hang-1;
            }
            else
            {
                if(hang==1)  AnglePID.Ki-=0.01;
                else if(hang==2) AnglePID.Kd-=0.1;
                else if(hang==3) AnglePID.Kp-=0.1;
				else if(hang==4) SpeedPID.Ki-=0.01;
                else if(hang==5) SpeedPID.Kd-=0.1;
                else if(hang==6) SpeedPID.Kp-=0.1;
				else if(hang==7) wPID.Ki-=0.01;
                else if(hang==8) wPID.Kd-=0.1;
                else if(hang==9) wPID.Kp-=0.001;
            }	
        }
    }
    // KEY_3：进入/确认
    else if(key3_state == KEY_SHORT_PRESS)
    {
		tft180_clear();
        key_clear_allstate();
        if(mode==0)
        {
            mode=hang;
            hang=1;
        }
        else if(mode==1)
        {
            if(change==0) change=1;
        }
    }
    // KEY_4：退出/返回
    else if(key4_state == KEY_SHORT_PRESS)
    {
		tft180_clear();
        key_clear_allstate();
        if(change==0)
        {
            if(mode!=0)
            {
                mode=0;
                hang=1;
            }
        }
        else if(change==1)
        {
            if(mode==1)
            {
                change=0;
            }
        }
    }
    
    // 更新显示
    show();
}
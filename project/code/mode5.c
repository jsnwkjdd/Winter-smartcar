#include "zf_common_headfile.h"
#include "motor.h"
#include "Encoder.h"
#include "pid.h"
#include "mode5.h"

uint8 buffer[16];  // 创建缓冲区
uint32 read_bytes;

/*
停止=90
前进=65
右前=66
右转=67
右后=68
后退=69
左后=70
左转=71
左前=72
*/

void mode5(PID_t *p1,PID_t *p2)
{
	// 尝试读取16字节数据
	read_bytes = bluetooth_ch9141_read_buffer(buffer, 16);

	if (read_bytes > 0)
	{
		if(buffer[0]==90){
			p1->Target=0;
			p2->Target=0;
		}
		else if(buffer[0]==65){
			p1->Target=2;
			p2->Target=0;
		}
		else if(buffer[0]==69){
			p1->Target=-2;
			p2->Target=0;
		}
		else if(buffer[0]==71){
			p1->Target=0.75;
			p2->Target=-0.5;
		}
		else if(buffer[0]==67){
			p1->Target=0.75;
			p2->Target=0.5;
		}
		else if(buffer[0]==72){
			p1->Target=1.25;
			p2->Target=-0.5;
		}
		else if(buffer[0]==66){
			p1->Target=1.25;
			p2->Target=0.5;
		}
		else if(buffer[0]==70){
			p1->Target=-1.25;
			p2->Target=0.5;
		}
		else if(buffer[0]==68){
			p1->Target=-1.25;
			p2->Target=-0.5;
		}
	}
}
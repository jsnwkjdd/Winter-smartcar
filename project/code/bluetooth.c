#include "mm32_device.h"                // Device header
#include "zf_device_bluetooth_ch9141.h"
#include "zf_driver_uart.h"
#include <string.h>
#include <stdint.h>
/*
功能：蓝牙初始化，逐飞库里面的，别忘了初始化
参数说明     void
// 返回参数     uint8           初始化状态 0-成功 1-失败
// 使用示例     bluetooth_ch9141_init();
*/

/*
功能：接收数据并进行解算：适用于摇杆
返回值：void
参数：储存摇杆四个数据的变量地址
*/
void sensorget_joystick(int8_t*LX,int8_t*LY,int8_t*RX,int8_t*RY)
{	
	uint8 temp[40];
	char data[40];
	bluetooth_ch9141_read_buffer(temp, 31);//接收数据
	for(uint8 i=0;i<40;i++)
	{
		data[i]=(char)temp[i];//类型转换便于字符串处理函数使用
	}
	char *Tag = strtok(data, ",");
	*LX = atoi(strtok(NULL, ","));//分割数据，转换为数字，储存到变量中
	*LY = atoi(strtok(NULL, ","));
	*RX = atoi(strtok(NULL, ","));
	*RY = atoi(strtok(NULL, ","));
}


/*
功能：串口重定向，用printf就可以向蓝牙发数据了 
并在工程选项里勾选Use MicroLIB！！！！！
我把zf_common_debug.c中调试串口的重定向注释掉了，
只能重定向一次，所以要用那个调试的话，把这个注释掉，那个恢复
  */
/*
int fputc(int ch, FILE *f)
{
	uart_write_byte(BLUETOOTH_CH9141_INDEX, ch);                      // 发送数据，将printf的底层重定向蓝牙
	return ch;
}*/
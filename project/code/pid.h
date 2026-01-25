#ifndef __PID_H
#define __PID_H
/*
	这是一个pid结构体定义的例子（可以再改动）
*/
//PID_t AnglePID = {
//	.Kp = 3,
//	.Ki = 0.1,
//	.Kd = 3,
//	
//	.OutMax = 100,
//	.OutMin = -100,
//};
typedef struct {
	float Target;
	float Actual;
	float Out;
	
	float Kp;
	float Ki;
	float Kd;
	
	float Error0;
	float Error1;
	float ErrorInt; 
	
	float OutMax;  //最大限幅
	float OutMin;  //最小限幅
} PID_t;

void PID_Init(PID_t *p);
void PID_Update(PID_t *p);
void PID_Update2(PID_t *p);
#endif

#include "PID.h"
/*
	此为pid清空函数
	作用：防止之前的数据影响，例如倒地过度积分使小车无法站立
	使用时机（个人理解）：在每次切入或切出发车功能时调用
	调用参数：地址（例：&speedpid）
*/
void PID_Init(PID_t *p)
{
	p->Target = 0;
	p->Actual = 0;
	p->Out = 0;
	p->Error0 = 0;
	p->Error1 = 0;
	p->ErrorInt = 0;
}
/*
	这是位置式pid的控制算法，也是江协平衡车pid的代码，如需在电机控制环中使用，还需赋值
	调用参数：地址（例：&speedpid）
*/
void PID_Update(PID_t *p)
{
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	
	if (p->Ki != 0)
	{
		p->ErrorInt += p->Error0;
	}
	else
	{
		p->ErrorInt = 0;
	}
	
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
		   + p->Kd * (p->Error0 - p->Error1);
	
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
}
/*
	这是增量式pid的算法，不一定会用到，但是先放这里吧，用法同上一个函数
*/
void PID_Update2(PID_t *p)
{
    // 增量式PID需要保存上上次误差，复用原位置式的ErrorInt来存储Error2（上上次误差）
    float Error2 = p->ErrorInt;  

    // 1. 更新误差历史（和原代码逻辑对齐，仅新增Error2的保存）
    p->Error1 = p->Error0;       // 上一次误差 = 当前误差
    p->Error0 = p->Target - p->Actual;  // 当前误差 = 目标值 - 实际值

    // 2. 核心：计算增量式PID输出增量 ΔOut
    // 公式：ΔOut = Kp*(e0-e1) + Ki*e0 + Kd*(e0-2e1+e2)
    float OutInc = p->Kp * (p->Error0 - p->Error1)
                 + p->Ki * p->Error0
                 + p->Kd * (p->Error0 - 2 * p->Error1 + Error2);

    // 3. 计算最终输出（增量式：当前输出 = 上一次输出 + 增量）
    p->Out += OutInc;

    // 4. 输出限幅（和原位置式代码逻辑完全一致）
    if (p->Out > p->OutMax) {p->Out = p->OutMax;}
    if (p->Out < p->OutMin) {p->Out = p->OutMin;}

    // 5. 保存上上次误差到ErrorInt，供下一次计算使用
    p->ErrorInt = p->Error1;
}
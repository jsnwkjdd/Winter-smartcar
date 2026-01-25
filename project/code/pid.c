#include "PID.h"
/*
	此为pid清空函数
	用途：清除小车之前的输出值，防止影响下一次输出，例如小车倒地后积分不断累加，再次发车会直接倒地
	用法（个人见解）：切入或切出发车模式调用一次？
	参数：结构体的地址
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
	此为位置式pid运算函数，也是江协平衡车里面pid的运算函数，不过这只是个pid模型，实现运动环还需构建运动环模型
	参数：结构体的地址
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
	此为增量式pid算法，不知道能不能用上，先放这里吧，用法同上
	参数：结构体的地址
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
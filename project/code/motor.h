#ifndef __MOTOR_H
#define __MOTOR_H

#define PWM_CH2                 (TIM5_PWM_CH2_A1)
#define PWM_CH4                 (TIM5_PWM_CH4_A3)
#include <stdint.h>
void Motor_Init(void);
void Motor_SetSpeedleft(int16_t Speed);
void Motor_SetSpeedright(int16_t Speed);

#endif
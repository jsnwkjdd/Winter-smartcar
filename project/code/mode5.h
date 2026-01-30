#ifndef _mode5_H_
#define _mode5_H_

#include "zf_common_headfile.h"
#include "motor.h"
#include "Encoder.h"
#include "pid.h"

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

void mode5(PID_t *p1,PID_t *p2);


#endif

#ifndef __menu_H_
#define __menu_H_

#include "key.h"
#include "zf_common_headfile.h"

extern int32_t mode;
extern int32_t hang;
extern int32_t change;
extern float kp;
extern float ki;
extern float kd;

void show(void);
void menu_key(void);
void menu_init(void);

#endif

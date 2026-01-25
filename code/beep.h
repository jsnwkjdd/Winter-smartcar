#ifndef _beep_H_
#define _beep_H_

#include "zf_common_headfile.h"
#define BEEP                (D7 )

void beep_init(void);
void beep_ms(uint16 time);
void beep_on(void);
void beep_off(void);


#endif

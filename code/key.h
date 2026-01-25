#ifndef __key_H_
#define __key_H_


#include "zf_common_headfile.h"


void my_key_init(void);
void timer_key(void);
void key_scan(void);
key_state_enum key_getstate (key_index_enum key_n);
void key_clearstate (key_index_enum key_n);
void key_clear_allstate (void);

#endif

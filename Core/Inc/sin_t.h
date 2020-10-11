
#ifndef _SIN_T_H_
#define _SIN_T_H_


#include <stdint.h>



extern const float sin_table2[];



#define sin_t(phase)	(sin_table2[(int)(phase * 1591.549430918953f) + 10000])
#define cos_t(phase)	(sin_table2[(int)(phase * 1591.549430918953f) + 12500])



/*
const float sin_table[];



float sin_t(float rad);

*/


#endif


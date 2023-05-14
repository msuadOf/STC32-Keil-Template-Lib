#ifndef _MAIN_H
#define _MAIN_H

#include "STC32G.H"
#include "Config.h"

typedef struct
{
	float Vref_left;
	float Vref_right;
	float V_forward_car;
	float V_roat_car;

	u16 Vref_left_datatx;
	u16 Vref_right_datatx;

} Car_t;

#endif // !_MAIN_H
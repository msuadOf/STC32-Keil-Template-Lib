#include "LineHunting.h"

float LineHunting_process(LineHunting_t *line)
{
	float out;

	out=(float)( (line->LineHunting_adc[2])*1+(line->LineHunting_adc[1])*100+(line->LineHunting_adc[0])*199 ) /
	 (float)((line->LineHunting_adc[0])+(line->LineHunting_adc[1])+(line->LineHunting_adc[2]));

	return out;
}
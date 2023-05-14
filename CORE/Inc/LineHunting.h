#ifndef _LINEHUNTING_H
#define _LINEHUNTING_H

#define LineHunting_adc_num 7
typedef struct 
{
	float LineHunting_adc[LineHunting_adc_num];
	float LineHunting_adc_normalize[LineHunting_adc_num];
	float LineHunting_adc_normalize_max;

	float adc_init_val[LineHunting_adc_num];

}LineHunting_t;

float LineHunting_process(LineHunting_t* line);

#endif // !_LINEHUNTING_H
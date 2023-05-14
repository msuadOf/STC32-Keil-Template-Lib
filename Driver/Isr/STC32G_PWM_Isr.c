#include "STC32G_PWM.h"


void PWMA_ISR() interrupt PWMA_VECTOR
{
	if(PWMA_SR1 & 0x01)
	{
		PWMA_SR1 &= ~0x01;
	}
}
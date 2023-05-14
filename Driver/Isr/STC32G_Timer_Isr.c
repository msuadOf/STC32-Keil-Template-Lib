#include	"STC32G_Timer.h"


//void TIM_Callback(TIM_HandleTypeDef* htim);
void TIM4_Callback();

//========================================================================
// ����: Timer0_ISR_Handler
// ����: Timer0�жϺ���.
// ����: none.
// ����: none.
// �汾: V1.0, 2020-09-23
//========================================================================
void Timer0_ISR_Handler (void) interrupt TMR0_VECTOR		//���ж�ʱ�Ѿ������־
{
  //Task_Marks_Handler_Callback();	//�����ǻص�����
	
	 #ifdef TIM0
	TIM_Callback(&htim0);
 #endif
}

//========================================================================
// ����: Timer1_ISR_Handler
// ����: Timer1�жϺ���.
// ����: none.
// ����: none.
// �汾: V1.0, 2020-09-23
//========================================================================
void Timer1_ISR_Handler (void) interrupt TMR1_VECTOR		//���ж�ʱ�Ѿ������־
{
	// TODO: �ڴ˴�����û�����
	//P66 = ~P66;
		 #ifdef TIM1
	TIM_Callback(&htim1);
 #endif
}

//========================================================================
// ����: Timer2_ISR_Handler
// ����: Timer2�жϺ���.
// ����: none.
// ����: none.
// �汾: V1.0, 2020-09-23
//========================================================================
void Timer2_ISR_Handler (void) interrupt TMR2_VECTOR		//���ж�ʱ�Ѿ������־
{
	// TODO: �ڴ˴�����û�����
	//P65 = ~P65;
		 #ifdef TIM2
	TIM_Callback(&htim2);
 #endif
}

//========================================================================
// ����: Timer3_ISR_Handler
// ����: Timer3�жϺ���.
// ����: none.
// ����: none.
// �汾: V1.0, 2020-09-23
//========================================================================
void Timer3_ISR_Handler (void) interrupt TMR3_VECTOR		//���ж�ʱ�Ѿ������־
{
	// TODO: �ڴ˴�����û�����
	//P64 = ~P64;
		 #ifdef TIM3
	TIM_Callback(&htim3);
 #endif
}

//========================================================================
// ����: Timer4_ISR_Handler
// ����: Timer4�жϺ���.
// ����: none.
// ����: none.
// �汾: V1.0, 2020-09-23
//========================================================================
void Timer4_ISR_Handler (void) interrupt TMR4_VECTOR		//���ж�ʱ�Ѿ������־
{
	// TODO: �ڴ˴�����û�����
	//P63 = ~P63;
	TIM4_Callback();
		 #ifdef TIM4
	TIM_Callback(&htim4);
 #endif
}

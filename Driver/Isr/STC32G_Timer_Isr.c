#include	"STC32G_Timer.h"


//void TIM_Callback(TIM_HandleTypeDef* htim);
void TIM4_Callback();

//========================================================================
// 函数: Timer0_ISR_Handler
// 描述: Timer0中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2020-09-23
//========================================================================
void Timer0_ISR_Handler (void) interrupt TMR0_VECTOR		//进中断时已经清除标志
{
  //Task_Marks_Handler_Callback();	//任务标记回调函数
	
	 #ifdef TIM0
	TIM_Callback(&htim0);
 #endif
}

//========================================================================
// 函数: Timer1_ISR_Handler
// 描述: Timer1中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2020-09-23
//========================================================================
void Timer1_ISR_Handler (void) interrupt TMR1_VECTOR		//进中断时已经清除标志
{
	// TODO: 在此处添加用户代码
	//P66 = ~P66;
		 #ifdef TIM1
	TIM_Callback(&htim1);
 #endif
}

//========================================================================
// 函数: Timer2_ISR_Handler
// 描述: Timer2中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2020-09-23
//========================================================================
void Timer2_ISR_Handler (void) interrupt TMR2_VECTOR		//进中断时已经清除标志
{
	// TODO: 在此处添加用户代码
	//P65 = ~P65;
		 #ifdef TIM2
	TIM_Callback(&htim2);
 #endif
}

//========================================================================
// 函数: Timer3_ISR_Handler
// 描述: Timer3中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2020-09-23
//========================================================================
void Timer3_ISR_Handler (void) interrupt TMR3_VECTOR		//进中断时已经清除标志
{
	// TODO: 在此处添加用户代码
	//P64 = ~P64;
		 #ifdef TIM3
	TIM_Callback(&htim3);
 #endif
}

//========================================================================
// 函数: Timer4_ISR_Handler
// 描述: Timer4中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2020-09-23
//========================================================================
void Timer4_ISR_Handler (void) interrupt TMR4_VECTOR		//进中断时已经清除标志
{
	// TODO: 在此处添加用户代码
	//P63 = ~P63;
	TIM4_Callback();
		 #ifdef TIM4
	TIM_Callback(&htim4);
 #endif
}

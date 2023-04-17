#include "STC32G_LCM.h"

//========================================================================
// 函数: LCM_ISR_Handler
// 描述: LCM 中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2022-03-23
//========================================================================
void LCM_ISR_Handler (void) interrupt LCM_VECTOR
{
	// TODO: 在此处添加用户代码
	if(LCMIFSTA & 0x01)
	{
		LCMIFSTA = 0x00;
//		LcmFlag = 0;
	}
}



#include	"STC32G_SPI.h"

//========================================================================
//                               本地变量声明
//========================================================================

u8 	SPI_RxCnt;

//========================================================================
// 函数: SPI_ISR_Handler
// 描述: SPI中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2020-09-23
//========================================================================
void SPI_ISR_Handler() interrupt SPI_VECTOR
{
	if(MSTR)	//主机模式
	{
		B_SPI_Busy = 0;
	}
	else							//从机模式
	{
		if(SPI_RxCnt >= SPI_BUF_LENTH)		SPI_RxCnt = 0;
		SPI_RxBuffer[SPI_RxCnt++] = SPDAT;
		SPI_RxTimerOut = 5;
	}
	SPI_ClearFlag();	//清0 SPIF和WCOL标志
}


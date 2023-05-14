#include "config.h"
#include "STC32G_GPIO.h"
#include "STC32G_UART.h"
#include "STC32G_NVIC.h"
#include "STC32G_Switch.h"
/****************  串口初始化函数 *****************/
void bsp_uart_init(void)
{
	COMx_InitDefine COMx_InitStructure;			   // 结构定义
	COMx_InitStructure.UART_Mode = UART_8bit_BRTx; // 模式,   UART_ShiftRight,UART_8bit_BRTx,UART_9bit,UART_9bit_BRTx
	//	COMx_InitStructure.UART_BRT_Use   = BRT_Timer2;			//选择波特率发生器, BRT_Timer2 (注意: 串口2固定使用BRT_Timer2, 所以不用选择)
	COMx_InitStructure.UART_BaudRate = 115200ul;	// 波特率,     110 ~ 115200
	COMx_InitStructure.UART_RxEnable = ENABLE;		// 接收允许,   ENABLE 或 DISABLE
	UART_Configuration(UART2, &COMx_InitStructure); // 初始化串口2 UART1,UART2,UART3,UART4
	NVIC_UART2_Init(ENABLE, Priority_1);			// 中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3

	UART2_SW(UART2_SW_P46_P47); // UART2_SW_P10_P11,UART2_SW_P46_P47
}
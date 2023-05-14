#include "config.h"
#include "STC32G_GPIO.h"
#include "STC32G_UART.h"
#include "STC32G_NVIC.h"
#include "STC32G_Switch.h"
/****************  ���ڳ�ʼ������ *****************/
void bsp_uart_init(void)
{
	COMx_InitDefine COMx_InitStructure;			   // �ṹ����
	COMx_InitStructure.UART_Mode = UART_8bit_BRTx; // ģʽ,   UART_ShiftRight,UART_8bit_BRTx,UART_9bit,UART_9bit_BRTx
	//	COMx_InitStructure.UART_BRT_Use   = BRT_Timer2;			//ѡ�����ʷ�����, BRT_Timer2 (ע��: ����2�̶�ʹ��BRT_Timer2, ���Բ���ѡ��)
	COMx_InitStructure.UART_BaudRate = 115200ul;	// ������,     110 ~ 115200
	COMx_InitStructure.UART_RxEnable = ENABLE;		// ��������,   ENABLE �� DISABLE
	UART_Configuration(UART2, &COMx_InitStructure); // ��ʼ������2 UART1,UART2,UART3,UART4
	NVIC_UART2_Init(ENABLE, Priority_1);			// �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3

	UART2_SW(UART2_SW_P46_P47); // UART2_SW_P10_P11,UART2_SW_P46_P47
}
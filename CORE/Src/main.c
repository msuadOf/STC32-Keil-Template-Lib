#include "config.h"
#include "STC32G_GPIO.h"
#include "STC32G_Delay.h"
#include "STC32G_Switch.h"
#include "STC32G_NVIC.h"
#include "STC32G_UART.h"

#include "STC32G_ADC.h"
#include "STC32G_DMA.h"
#include "STC32G_Timer.h"

#include "main.h"
#include "math.h"
#include "pid.h"
#include "LineHunting.h"
#include "MPU6500.h"

/***************	����˵��	****************

����ʹ��P2������ʾ����ơ�

����ʱ, ѡ��ʱ�� 24MHz (�����������ļ�"config.h"���޸�).

******************************************/

bit DmaTx1Flag;
bit DmaRx1Flag;
bit DmaTx2Flag;
bit DmaRx2Flag;
bit DmaTx3Flag;
bit DmaRx3Flag;
bit DmaTx4Flag;
bit DmaRx4Flag;

#define DMA_Buffer_len 6
u8 xdata DmaBuffer[DMA_Buffer_len]; // �շ����û��棬ͬʱʹ�ö�·����ʱÿ��������ֱ��建�棬�����໥����
u16 i_A = 0;
u16 i_B = 0;

u8 code ledNum[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
u8 ledIndex;

#define ADC_CH 16  /* 1~16, ADCת��ͨ����, ��ͬ���޸�ת��ͨ�� */
#define ADC_DATA 6 /* 6~n, ÿ��ͨ��ADCת����������, 2*ת������+4, ��ͬ���޸�ת������ */

typedef union
{
	u8 data8[2];
	u16 data16;
} ADC_DMA_Data_Union;

u8 chn = 0;
bit DmaADCFlag = 0;
ADC_DMA_Data_Union xdata DmaAdBuffer[ADC_CH][ADC_DATA];

float roat = 0;

Car_t hcar = {0};

pid_t PID_direction = {0, 0, 0, {0, 0, 0}, 0.7, 0, 1, 0, 0};
/*************  �ⲿ�����ͱ������� *****************/

/******************** IO������ ********************/
// void GPIO_config(void)
//{
//	//	P3_MODE_IO_PU(GPIO_Pin_0 | GPIO_Pin_1);	//P3.0,P3.1 ����Ϊ׼˫��� - UART1
//			P4_MODE_IO_PU(GPIO_Pin_6 | GPIO_Pin_7); // P4.6,P4.7 ����Ϊ׼˫��� - UART2
//	//	P0_MODE_IO_PU(GPIO_Pin_0 | GPIO_Pin_1);	//P0.0,P0.1 ����Ϊ׼˫��� - UART3
//	//	P0_MODE_IO_PU(GPIO_Pin_2 | GPIO_Pin_3);	//P0.2,P0.3 ����Ϊ׼˫��� - UART4
//	P2_MODE_IO_PU(GPIO_Pin_0 | GPIO_Pin_1);
// }

///******************** UART���� ********************/
// void UART_config(void)
//{
//	COMx_InitDefine COMx_InitStructure; // �ṹ����

//	COMx_InitStructure.UART_Mode = UART_8bit_BRTx; // ģʽ,   UART_ShiftRight,UART_8bit_BRTx,UART_9bit,UART_9bit_BRTx
//	COMx_InitStructure.UART_BRT_Use = BRT_Timer2;  // ѡ�����ʷ�����, BRT_Timer1, BRT_Timer2 (ע��: ����2�̶�ʹ��BRT_Timer2)
//	COMx_InitStructure.UART_BaudRate = 115200ul;   // ������,     110 ~ 115200
//	COMx_InitStructure.UART_RxEnable = ENABLE;	   // ��������,   ENABLE��DISABLE
//	//	UART_Configuration(UART1, &COMx_InitStructure);		//��ʼ������ UART1,UART2,UART3,UART4
//	//	NVIC_UART1_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3
//	UART_Configuration(UART2, &COMx_InitStructure); // ��ʼ������ UART1,UART2,UART3,UART4
//	NVIC_UART2_Init(ENABLE, Priority_0);			// �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3
//	//	UART_Configuration(UART3, &COMx_InitStructure);		//��ʼ������ UART1,UART2,UART3,UART4
//	//	NVIC_UART3_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3
//	//	UART_Configuration(UART4, &COMx_InitStructure);		//��ʼ������ UART1,UART2,UART3,UART4
//	//	NVIC_UART4_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3

//	//	UART1_SW(UART1_SW_P30_P31);		//UART1_SW_P30_P31,UART1_SW_P36_P37,UART1_SW_P16_P17,UART1_SW_P43_P44
//	UART2_SW(UART2_SW_P46_P47); // UART2_SW_P10_P11,UART2_SW_P46_P47
//	//	UART3_SW(UART3_SW_P00_P01);		//UART3_SW_P00_P01,UART3_SW_P50_P51
//	//	UART4_SW(UART4_SW_P02_P03);		//UART4_SW_P02_P03,UART4_SW_P52_P53
//}

/******************** DMA ���� ********************/
void DMA_UART_config(void)
{

	DMA_UART_InitTypeDef DMA_UART_InitStructure; // �ṹ����

	DMA_UART_InitStructure.DMA_TX_Length = DMA_Buffer_len - 1; // DMA�������ֽ���  	(0~65535) + 1
	DMA_UART_InitStructure.DMA_TX_Buffer = (u16)DmaBuffer;	   // �������ݴ洢��ַ
	//	DMA_UART_InitStructure.DMA_RX_Length = 255;				//DMA�������ֽ���  	(0~65535) + 1
	//	DMA_UART_InitStructure.DMA_RX_Buffer = (u16)DmaBuffer;	//�������ݴ洢��ַ
	DMA_UART_InitStructure.DMA_TX_Enable = ENABLE; // DMAʹ��  	ENABLE,DISABLE
	//	DMA_UART_InitStructure.DMA_RX_Enable = ENABLE;		//DMAʹ��  	ENABLE,DISABLE
	//	DMA_UART_Inilize(UART1, &DMA_UART_InitStructure);	//��ʼ��
	DMA_UART_Inilize(UART2, &DMA_UART_InitStructure); // ��ʼ��
	//	DMA_UART_Inilize(UART3, &DMA_UART_InitStructure);	//��ʼ��
	//	DMA_UART_Inilize(UART4, &DMA_UART_InitStructure);	//��ʼ��

	//	NVIC_DMA_UART1_Tx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	//	NVIC_DMA_UART1_Rx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	NVIC_DMA_UART2_Tx_Init(ENABLE, Priority_2, Priority_2); // �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	//	NVIC_DMA_UART2_Rx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	//	NVIC_DMA_UART3_Tx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	//	NVIC_DMA_UART3_Rx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	//	NVIC_DMA_UART4_Tx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	//	NVIC_DMA_UART4_Rx_Init(ENABLE,Priority_0,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3

	//	DMA_UR1R_CLRFIFO();		//��� DMA FIFO
	DMA_UR2R_CLRFIFO(); // ��� DMA FIFO
						//	DMA_UR3R_CLRFIFO();		//��� DMA FIFO
						//	DMA_UR4R_CLRFIFO();		//��� DMA FIFO
}

void GPIO_config(void)
{
	P0_MODE_IN_HIZ(GPIO_Pin_LOW | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6); // P0.0~P0.6 ����Ϊ��������

	P1_MODE_IN_HIZ(GPIO_Pin_All); // P1.0~P1.7 ����Ϊ��������

	P2_MODE_IO_PU(GPIO_Pin_All);

	P4_MODE_IO_PU(GPIO_Pin_6 | GPIO_Pin_7); // P4.6,P4.7 ����Ϊ׼˫��� - UART2
											// P2_MODE_IO_PU(GPIO_Pin_0 | GPIO_Pin_1);
}
void UART_config(void)
{
	//		COMx_InitDefine COMx_InitStructure; // �ṹ����

	//	COMx_InitStructure.UART_Mode = UART_8bit_BRTx; // ģʽ,   UART_ShiftRight,UART_8bit_BRTx,UART_9bit,UART_9bit_BRTx
	//	//	COMx_InitStructure.UART_BRT_Use   = BRT_Timer2;			//ѡ�����ʷ�����, BRT_Timer2 (ע��: ����2�̶�ʹ��BRT_Timer2, ���Բ���ѡ��)
	//	COMx_InitStructure.UART_BaudRate = 115200ul;	// ������,     110 ~ 115200
	//	COMx_InitStructure.UART_RxEnable = ENABLE;		// ��������,   ENABLE �� DISABLE
	//	UART_Configuration(UART2, &COMx_InitStructure); // ��ʼ������2 UART1,UART2,UART3,UART4
	//	while (NVIC_UART2_Init((u8)ENABLE, (u8)Priority_1) != SUCCESS)
	//		; // �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3

	//	UART2_SW(UART2_SW_P46_P47); // UART2_SW_P10_P11,UART2_SW_P46_P47

	COMx_InitDefine COMx_InitStructure; // �ṹ����

	COMx_InitStructure.UART_Mode = UART_8bit_BRTx; // ģʽ,   UART_ShiftRight,UART_8bit_BRTx,UART_9bit,UART_9bit_BRTx
	COMx_InitStructure.UART_BRT_Use = BRT_Timer2;  // ѡ�����ʷ�����, BRT_Timer1, BRT_Timer2 (ע��: ����2�̶�ʹ��BRT_Timer2)
	COMx_InitStructure.UART_BaudRate = 115200ul;   // ������,     110 ~ 115200
	COMx_InitStructure.UART_RxEnable = ENABLE;	   // ��������,   ENABLE��DISABLE
	//	UART_Configuration(UART1, &COMx_InitStructure);		//��ʼ������ UART1,UART2,UART3,UART4
	//	NVIC_UART1_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3
	UART_Configuration(UART2, &COMx_InitStructure); // ��ʼ������ UART1,UART2,UART3,UART4
	NVIC_UART2_Init(ENABLE, Priority_0);			// �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3
	//	UART_Configuration(UART3, &COMx_InitStructure);		//��ʼ������ UART1,UART2,UART3,UART4
	//	NVIC_UART3_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3
	//	UART_Configuration(UART4, &COMx_InitStructure);		//��ʼ������ UART1,UART2,UART3,UART4
	//	NVIC_UART4_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3

	//	UART1_SW(UART1_SW_P30_P31);		//UART1_SW_P30_P31,UART1_SW_P36_P37,UART1_SW_P16_P17,UART1_SW_P43_P44
	UART2_SW(UART2_SW_P46_P47); // UART2_SW_P10_P11,UART2_SW_P46_P47
								//	UART3_SW(UART3_SW_P00_P01);		//UART3_SW_P00_P01,UART3_SW_P50_P51
								//	UART4_SW(UART4_SW_P02_P03);		//UART4_SW_P02_P03,UART4_SW_P52_P53
}

/******************** ADC ���� ********************/
void ADC_config(void)
{
	ADC_InitTypeDef ADC_InitStructure; // �ṹ����

	ADC_InitStructure.ADC_SMPduty = 31;					   // ADC ģ���źŲ���ʱ�����, 0~31��ע�⣺ SMPDUTY һ����������С�� 10��
	ADC_InitStructure.ADC_CsSetup = 0;					   // ADC ͨ��ѡ��ʱ����� 0(Ĭ��),1
	ADC_InitStructure.ADC_CsHold = 1;					   // ADC ͨ��ѡ�񱣳�ʱ����� 0,1(Ĭ��),2,3
	ADC_InitStructure.ADC_Speed = ADC_SPEED_2X16T;		   // ���� ADC ����ʱ��Ƶ��	ADC_SPEED_2X1T~ADC_SPEED_2X16T
	ADC_InitStructure.ADC_AdjResult = ADC_RIGHT_JUSTIFIED; // ADC�������,	ADC_LEFT_JUSTIFIED,ADC_RIGHT_JUSTIFIED
	ADC_Inilize(&ADC_InitStructure);					   // ��ʼ��
	ADC_PowerControl(ENABLE);							   // ADC��Դ����, ENABLE��DISABLE
	NVIC_ADC_Init(DISABLE, Priority_0);					   // �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3
}
/******************** DMA ���� ********************/
void DMA_ADC_config(void)
{
	//***********DMA_ADC**************

	DMA_ADC_InitTypeDef DMA_ADC_InitStructure; // �ṹ����

	DMA_ADC_InitStructure.DMA_Enable = ENABLE;			 // DMAʹ��  	ENABLE,DISABLE
	DMA_ADC_InitStructure.DMA_Channel = 0xffff;			 // ADCͨ��ʹ�ܼĴ���, 1:ʹ��, bit15~bit0 ��Ӧ ADC15~ADC0
	DMA_ADC_InitStructure.DMA_Buffer = (u16)DmaAdBuffer; // ADCת�����ݴ洢��ַ
	DMA_ADC_InitStructure.DMA_Times = ADC_4_Times;		 // ÿ��ͨ��ת������, ADC_1_Times,ADC_2_Times,ADC_4_Times,ADC_8_Times,ADC_16_Times,ADC_32_Times,ADC_64_Times,ADC_128_Times,ADC_256_Times
	DMA_ADC_Inilize(&DMA_ADC_InitStructure);			 // ��ʼ��
	NVIC_DMA_ADC_Init(ENABLE, Priority_3, Priority_3);	 // �ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0~Priority_3; �������ȼ�(�͵���) Priority_0~Priority_3
	DMA_ADC_TRIG();										 // ��������ת��
}

/*************	����˵��	**************

������ʾ5����ʱ����ʹ��, ��ʹ��16λ�Զ���װ.

��ʱ��0��16λ�Զ���װ, �ж�Ƶ��Ϊ100000Hz���жϺ�����P6.7ȡ�����50KHz�����ź�.

��ʱ��1��16λ�Զ���װ, �ж�Ƶ��Ϊ10000Hz���жϺ�����P6.6ȡ�����5KHz�����ź�.

��ʱ��2��16λ�Զ���װ, �ж�Ƶ��Ϊ1000Hz���жϺ�����P6.5ȡ�����500Hz�����ź�.

��ʱ��3��16λ�Զ���װ, �ж�Ƶ��Ϊ100Hz���жϺ�����P6.4ȡ�����50Hz�����ź�.

��ʱ��4��16λ�Զ���װ, �ж�Ƶ��Ϊ50Hz���жϺ�����P6.3ȡ�����25Hz�����ź�.

����ʱ, ѡ��ʱ�� 24MHz (�����������ļ�"config.h"���޸�).

******************************************/
/************************ ��ʱ������ ****************************/
void Timer_config(void)
{
	TIM_InitTypeDef TIM_InitStructure; // �ṹ����
	//	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//ָ������ģʽ,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	//	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
	//	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 100000UL));		//��ֵ,
	//	TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	//	Timer_Inilize(Timer0,&TIM_InitStructure);					//��ʼ��Timer0	  Timer0,Timer1,Timer2,Timer3,Timer4
	//	NVIC_Timer0_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3

	//	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//ָ������ģʽ,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//ָ��ʱ��Դ, TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	//	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
	//	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 10000));			//��ֵ,
	//	TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	//	Timer_Inilize(Timer1,&TIM_InitStructure);					//��ʼ��Timer1	  Timer0,Timer1,Timer2,Timer3,Timer4
	//	NVIC_Timer1_Init(ENABLE,Priority_0);		//�ж�ʹ��, ENABLE/DISABLE; ���ȼ�(�͵���) Priority_0,Priority_1,Priority_2,Priority_3

	//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;		//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	//	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
	//	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / 1000));				//��ֵ
	//	TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	//	Timer_Inilize(Timer2,&TIM_InitStructure);					//��ʼ��Timer2	  Timer0,Timer1,Timer2,Timer3,Timer4
	//	NVIC_Timer2_Init(ENABLE,NULL);		//�ж�ʹ��, ENABLE/DISABLE; �����ȼ�

	//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;	//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	//	TIM_InitStructure.TIM_ClkOut    = ENABLE;					//�Ƿ������������, ENABLE��DISABLE
	//	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / (100*12)));		//��ֵ
	//	TIM_InitStructure.TIM_Run       = ENABLE;					//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	//	Timer_Inilize(Timer3,&TIM_InitStructure);					//��ʼ��Timer3	  Timer0,Timer1,Timer2,Timer3,Timer4
	//	NVIC_Timer3_Init(ENABLE,NULL);		//�ж�ʹ��, ENABLE/DISABLE; �����ȼ�

	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T; // ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut = ENABLE;			 // �Ƿ������������, ENABLE��DISABLE
	//	TIM_InitStructure.TIM_Value     = (u16)(65536UL - (MAIN_Fosc / (1000*12)));		//��ֵ
	TIM_InitStructure.TIM_Value = (u16)(65536UL - (MAIN_Fosc / (500 * 12)));
	TIM_InitStructure.TIM_Run = ENABLE;		   // �Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	Timer_Inilize(Timer4, &TIM_InitStructure); // ��ʼ��Timer4	  Timer0,Timer1,Timer2,Timer3,Timer4
	NVIC_Timer4_Init(ENABLE, NULL);			   // �ж�ʹ��, ENABLE/DISABLE; �����ȼ�
}

LineHunting_t hLineHunting = {{1, 1, 1}, {1, 1, 1}, 1, {1, 1, 1}};

void speed_transfer(u8 *DmaTxBuffer, u16 left_speed, u16 right_speed)
{
	DmaTxBuffer[0] = 0x5A;
	DmaTxBuffer[3] = (u8)(left_speed >> 8);
	DmaTxBuffer[4] = (u8)(left_speed);
	DmaTxBuffer[1] = (u8)(right_speed >> 8);
	DmaTxBuffer[2] = (u8)(right_speed);
	DmaTxBuffer[5] = 0xA5;
	DMA_UR2T_TRIG();
}

void Vcar_2_Vref_transfer(Car_t *hcar)
{
	hcar->Vref_left = 1 * (hcar->V_forward_car) - 1 * (hcar->V_roat_car);
	hcar->Vref_right = 1 * (hcar->V_forward_car) + 1 * (hcar->V_roat_car);
}
void vref_2_data_transfer(Car_t *hcar)
{
	hcar->Vref_left_datatx = (u16)(hcar->Vref_left) + 0x8000;
	hcar->Vref_right_datatx = (u16)(hcar->Vref_right) + 0x8000;
}
void Car_speed_set(Car_t *hcar, float V_forward_car, float V_roat_car)
{
	hcar->V_forward_car = V_forward_car;
	hcar->V_roat_car = V_roat_car;
	Vcar_2_Vref_transfer(hcar);
	vref_2_data_transfer(hcar);
	speed_transfer(DmaBuffer, hcar->Vref_left_datatx, hcar->Vref_right_datatx);
}

// void PWMA_config(void){
//	PWMA_CR2=0x10;
//	PWMA_ARRH=0x13;
//	PWMA_ARRL=0x38;
//	PWMA_IER=0x01;
//	PWMA_CR1=0x01;
// }

// void TIM_Callback(TIM_HandleTypeDef* htim){
//	if(htim->Instance==TIM4){
//		P20 = ~P20;
//	}
// }

void TIM4_Callback()
{

	DMA_ADC_TRIG();
}

// void LineHunting_ADC_fill_in(LineHunting_t* line){
// hLineHunting.LineHunting_adc[0]=(DmaAdBuffer[0][5].data16);
// hLineHunting.LineHunting_adc[1]=(DmaAdBuffer[1][5].data16);
// hLineHunting.LineHunting_adc[2]=(DmaAdBuffer[2][5].data16);
// }

float Sensor_Left,
	Sensor_Middle,
	Sensor_Right,
	Sensor_Left_M,
	Sensor_Right_M,
	sum,
	Sensor, Bias, Last_Bias,
	Velocity, Angle,
	kA, kB, kC, kD;
float gyro_buffer[3], acc_buffer[3];
float roat_speed_action, roat_speed_aim, roat_speed_current, roat_speed_err, roat_speed_err_last = 0;
void STC32G_DMA_ADC_IRQHandler()
{

	// LineHunting_ADC_fill_in(&hLineHunting);
	hLineHunting.LineHunting_adc[0] = (float)(DmaAdBuffer[0][5].data16);
	hLineHunting.LineHunting_adc[1] = (float)(DmaAdBuffer[1][5].data16);
	hLineHunting.LineHunting_adc[2] = (float)(DmaAdBuffer[3][5].data16);
	hLineHunting.LineHunting_adc[3] = (float)(DmaAdBuffer[4][5].data16);
	hLineHunting.LineHunting_adc[4] = (float)(DmaAdBuffer[5][5].data16);
	hLineHunting.LineHunting_adc[5] = (float)(DmaAdBuffer[6][5].data16);
	hLineHunting.LineHunting_adc[6] = (float)(DmaAdBuffer[7][5].data16);

	Sensor_Left = hLineHunting.LineHunting_adc[6];	 // ��ߵ�вɼ�ֵ
	Sensor_Middle = hLineHunting.LineHunting_adc[3]; // �м��вɼ�ֵ
	Sensor_Right = hLineHunting.LineHunting_adc[0];	 // �ұߵ�вɼ�ֵ

	Sensor_Left_M = hLineHunting.LineHunting_adc[0];  // ��ߵ�вɼ�ֵ
	Sensor_Right_M = hLineHunting.LineHunting_adc[5]; // �ұߵ�вɼ�ֵ

	// if (Sensor_Left + Sensor_Middle + Sensor_Right > 25)
	// {
	// 	sum = sqrt(Sensor_Left) * 1 + Sensor_Middle * 50 + sqrt(Sensor_Right) * 99; // ��һ������
	// 	Sensor = sum / (Sensor_Left + Sensor_Middle + Sensor_Right);	// ��ƫ��
	// }
	// //Velocity = 35;									// ���Ѳ��ģʽ�µ��ٶ�
	// Bias = Sensor - 50;								// ��ȡƫ��
	kB = 0.1;
	if (Sensor_Left + Sensor_Middle + Sensor_Right > 25)
	{
		// Bias = 50.0 * ((1 * (Sensor_Left - Sensor_Right) +
		// 				kB * (Sensor_Left_M - Sensor_Right_M)) /
		// 			   (1 * (Sensor_Left + Sensor_Right) +
		// 				kB * (Sensor_Left_M + Sensor_Right_M)));

		Bias = 50.0 * (
						( (Sensor_Left - Sensor_Right) )   /
					    ( (Sensor_Left + Sensor_Right) )
					   );

		if (fabs(Bias) < 0.5)
			Bias = 0;
	}
	else
	{
		Bias = 0;
	}
	Angle = Bias * (280.0f) + (Bias - Last_Bias) * (-100.0f); // ����PID*/ ���˫���
	// Angle = Bias * (280.0f) + (Bias - Last_Bias) * (-80.0f); // ����PID*/ �ڲ�˫���
	//  Angle = abs(Bias)*Bias * 0.02 + Bias * 0.074 + (Bias - Last_Bias) * 1;
	Last_Bias = Bias; // ��һ�ε�ƫ��

	// bias = LineHunting_process(&hLineHunting);
	//	if((bias-bias_old)>50 && ((bias-bias_old)<-50)){
	//		bias = bias_old; //update
	//	}
	// roat_speed_aim = -0.065f * Angle;

	// MPU6500_get_buffer(gyro_buffer, acc_buffer);
	// if (fabs(gyro_buffer[2]) > 8.5)
	// {
	// 	roat_speed_current = -gyro_buffer[2];
	// }
	// else
	// {
	// 	roat_speed_current = 0;
	// }

	// roat_speed_err = roat_speed_aim - roat_speed_current;
	// roat_speed_action = (-16.0f) * (roat_speed_err) + (10.0f) * (roat_speed_err - roat_speed_err_last);
	// roat_speed_err_last = roat_speed_err;

	//    roat=pid_process(&PID_direction,bias,0,6000,-6000);
	// roat=arm_pid_f(&PID_direction,bias-0);

	//	if (hLineHunting.LineHunting_adc_normalize[1] < 50)
	//	{
	//		Car_speed_set(&hcar, 0, 0);
	//	}
	//	else
	//	{
	Car_speed_set(&hcar, 3500, Angle);
	//	}

	// Car_speed_set(&hcar,0,0);
	//	printf("ADC:%f,%f\n", (DmaAdBuffer[0][5].data16*2.5f/4096),(DmaAdBuffer[2][5].data16*2.5f/4096)); // ��1������,...,��n������,ADͨ��,ƽ������,ƽ��ֵ

	// DMA_ADC_TRIG(); // ���´���������һ��ת��
}

void main(void)
{
	WTST = 0;  // ���ó���ָ����ʱ��������ֵΪ0�ɽ�CPUִ��ָ����ٶ�����Ϊ���
	EAXSFR();  // ��չSFR(XFR)����ʹ��
	CKCON = 0; // ��߷���XRAM�ٶ�
	EA = 1;	   // �����ж�

	arm_pid_init_f(&PID_direction);

	GPIO_config();
	UART_config();
	ADC_config();
	DMA_ADC_config();
	DMA_UART_config();
	Timer_config();

	P20 = 0; // ��ʵ���LED��Դ

	// u16 i,n;

	MPU6500_Init();

	delay_ms(500);

	hLineHunting.adc_init_val[0] = DmaAdBuffer[0][5].data16;
	hLineHunting.adc_init_val[1] = DmaAdBuffer[1][5].data16;
	hLineHunting.adc_init_val[2] = DmaAdBuffer[3][5].data16;

	while (1)
	{
		// printf("ADC:%f,%f,%f\n", (1000.0f*DmaAdBuffer[0][5].data16*1.0f/hLineHunting.adc_init_val[0]),(1000.0f*DmaAdBuffer[1][5].data16*1.0f/hLineHunting.adc_init_val[1]),(1000.0f*DmaAdBuffer[3][5].data16*1.0f/hLineHunting.adc_init_val[2])); // ��1������,...,��n������,ADͨ��,ƽ������,ƽ��ֵ

		delay_ms(500);
		P20 = ~P20;
		// Car_speed_set(&hcar,700,260);
		// DMA_ADC_TRIG(); // ���´���������һ��ת��

		//(*(unsigned char volatile  *)0x7e00a0)=~(*(unsigned char volatile  *)0x7e00a0);
		// TX2_write2buff(0xAA);
		// printf("%d\n", kA);
	}
}
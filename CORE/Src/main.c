#include	"config.h"
#include	"STC32G_GPIO.h"
#include	"STC32G_Delay.h"

/***************	����˵��	****************

����ʹ��P2������ʾ����ơ�

����ʱ, ѡ��ʱ�� 24MHz (�����������ļ�"config.h"���޸�).

******************************************/

u8 code ledNum[]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
u8 ledIndex;

void GPIO_config(void)
{
	//P4_MODE_IO_PU(GPIO_Pin_0);			//P4.0����Ϊ׼˫���
	P2_MODE_IO_PU(GPIO_Pin_All);		//P2 ����Ϊ׼˫���
}


void main(void)
{
	WTST = 0;		//���ó���ָ����ʱ��������ֵΪ0�ɽ�CPUִ��ָ����ٶ�����Ϊ���
	EAXSFR();		//��չSFR(XFR)����ʹ�� 
	CKCON = 0;      //��߷���XRAM�ٶ�

	GPIO_config();
	P20 = 0;		//��ʵ���LED��Դ
	
	while(1)
	{
		delay_ms(250);
		P2 = ~ledNum[ledIndex];	//���������
		ledIndex++;
		if(ledIndex > 7)
		{
			ledIndex = 0;
		}
	}
}
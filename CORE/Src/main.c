#include	"config.h"
#include	"STC32G_GPIO.h"
#include	"STC32G_Delay.h"

/***************	功能说明	****************

程序使用P2口来演示跑马灯。

下载时, 选择时钟 24MHz (可以在配置文件"config.h"中修改).

******************************************/

u8 code ledNum[]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
u8 ledIndex;

void GPIO_config(void)
{
	//P4_MODE_IO_PU(GPIO_Pin_0);			//P4.0设置为准双向口
	P2_MODE_IO_PU(GPIO_Pin_All);		//P2 设置为准双向口
}


void main(void)
{
	WTST = 0;		//设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快
	EAXSFR();		//扩展SFR(XFR)访问使能 
	CKCON = 0;      //提高访问XRAM速度

	GPIO_config();
	P20 = 0;		//打开实验板LED电源
	
	while(1)
	{
		delay_ms(250);
		P2 = ~ledNum[ledIndex];	//输出低驱动
		ledIndex++;
		if(ledIndex > 7)
		{
			ledIndex = 0;
		}
	}
}
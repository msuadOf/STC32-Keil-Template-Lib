#include	"STC32G_I2C.h"

I2C_IsrTypeDef I2CIsr;
bit DisplayFlag;

//========================================================================
// 函数: I2C_ISR_Handler
// 描述: I2C中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2020-09-23
//========================================================================
void I2C_ISR_Handler() interrupt I2C_VECTOR
{
	// TODO: 在此处添加用户代码

	// 主机模式
	I2CMSST &= ~0x40;       //I2C指令发送完成状态清除

	if(DMA_I2C_CR & 0x04)   //ACKERR
	{
		DMA_I2C_CR &= ~0x04;  //发数据后收到NAK
	}

	// 从机模式
	if (I2CSLST & 0x40)
	{
		I2CSLST &= ~0x40;                       //处理START事件
	}
	else if (I2CSLST & 0x20)
	{
		I2CSLST &= ~0x20;                       //处理RECV事件，SLACKO设置为0
		if (I2CIsr.isda)
		{
			I2CIsr.isda = 0;                           //处理RECV事件（RECV DEVICE ADDR）
		}
		else if (I2CIsr.isma)
		{
			I2CIsr.isma = 0;                           //处理RECV事件（RECV MEMORY ADDR）
			I2CIsr.addr = I2CRXD;
			I2CTXD = I2C_Buffer[I2CIsr.addr];
		}
		else
		{
			I2C_Buffer[I2CIsr.addr++] = I2CRXD;            //处理RECV事件（RECV DATA）
		}
	}
	else if (I2CSLST & 0x10)
	{
		I2CSLST &= ~0x10;                       //处理SEND事件
		if (I2CSLST & 0x02)
		{
			I2CTXD = 0xff;
		}
		else
		{
			I2CTXD = I2C_Buffer[++I2CIsr.addr];
		}
	}
	else if (I2CSLST & 0x08)
	{
		I2CSLST &= ~0x08;                       //处理STOP事件
		I2CIsr.isda = 1;
		I2CIsr.isma = 1;
		DisplayFlag = 1;
	}
}


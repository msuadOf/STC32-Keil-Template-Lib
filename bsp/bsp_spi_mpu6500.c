
/*
====================================
    ||
    ||
    ||  硬件实现层，更改设备的话请在这里进行实现
    ||
    ||
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/
#include "config.h"

//========SPI实现（部分函数SPI库有实现：SPI_WriteByte、SPI_ReadByte）========


#include "STC32G_SPI.h"
#include "STC32G_NVIC.h"
#include "STC32G_Switch.h"

#define SPI_CS_EN() SPI_SS_2 = 0
#define SPI_CS_DISABLE() SPI_SS_2 = 1

/****************  SPI初始化函数 *****************/
void SPI_config(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    SPI_InitStructure.SPI_Enable = ENABLE;        // SPI启动    ENABLE, DISABLE
    SPI_InitStructure.SPI_SSIG = DISABLE;          // 片选位     ENABLE, DISABLE
    SPI_InitStructure.SPI_FirstBit = SPI_MSB;     // 移位方向   SPI_MSB, SPI_LSB
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master; // 主从选择   SPI_Mode_Master, SPI_Mode_Slave
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;   // 时钟相位   SPI_CPOL_High,   SPI_CPOL_Low
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;  // 数据边沿   SPI_CPHA_1Edge,  SPI_CPHA_2Edge
    SPI_InitStructure.SPI_Speed = SPI_Speed_16;    // SPI速度    SPI_Speed_4, SPI_Speed_16, SPI_Speed_64, SPI_Speed_128
    SPI_Init(&SPI_InitStructure);
    //NVIC_SPI_Init(ENABLE, Priority_3); // 中断使能, ENABLE/DISABLE; 优先级(低到高) Priority_0,Priority_1,Priority_2,Priority_3

    SPI_SW(SPI_P22_P23_P24_P25); // SPI_P54_P13_P14_P15,SPI_P22_P23_P24_P25,SPI_P54_P40_P41_P43,SPI_P35_P34_P33_P32
    SPI_SS_2 = 1;
}

// void SPI_WriteByte(u8 out)
//{
//     SPDAT = out;
//     while(SPIF == 0) ;
//     SPIF = 1;   //清SPIF标志
//     WCOL = 1;   //清WCOL标志
// }

///************************************************************************/
// u8 SPI_ReadByte(void)
//{
//     SPDAT = 0xff;
//     while(SPIF == 0) ;
//     SPIF = 1;   //清SPIF标志
//     WCOL = 1;   //清WCOL标志
//     return (SPDAT);
// }

/************************************************************************
读出size个字节,跟指定的数据进行比较, 错误返回1,正确返回0
************************************************************************/
void SPI_Read_Nbytes(u8 addr, u8 *buffer, u16 size)
{
    if (size == 0)
        return;

    SPI_CS_EN(); // enable device

    SPI_WriteByte(addr); // 设置起始地址

    do
    {
        *buffer = SPI_ReadByte(); // receive byte and store at buffer
        buffer++;
    } while (--size); // read until no_bytes is reached
    SPI_CS_DISABLE(); // disable device
}



/************************************************
写数据到spi总线上
入口参数:
    addr   : 地址参数
    buffer : 缓冲需要写入Flash的数据
    size   : 字节数
出口参数: 无
************************************************/
void SPI_Write_Nbytes(u8 addr, u8 *buffer, u8 size)
{
    if (size == 0)
        return;

    SPI_CS_EN(); // enable device

    SPI_WriteByte(addr); // 设置起始地址

    do
    {
        SPI_WriteByte(*buffer++); // 连续页内写
        addr++;
        // if ((addr & 0xff) == 0) break;
    } while (--size);
    SPI_CS_DISABLE(); // disable device
}

void bsp_spi_init(){
    SPI_config();
}

void bsp_spi_read(unsigned char addr, unsigned char *buffer, int len){
    SPI_Read_Nbytes(addr+(1<<7), buffer, len);
}

void bsp_spi_write(unsigned char addr, unsigned char *buffer, int len){
    SPI_Write_Nbytes(addr+(0<<7), buffer, len);
}
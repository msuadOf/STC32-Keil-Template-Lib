#include <stdio.h>
#include "MPU6500.h"
/* 选择使用SPI还是I2C进行通讯捏，当果然是SPI腻 */
#define MPU6500_USING_SPI
// #define MPU6500_USING_I2C

/**
 *
 * ********************************************************
 *
 *  兼容层捏 HAL_level start
 *
 * ********************************************************
 *
 **/

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
#include "bsp.h"
/**
 * @brief 统一API接入
 *
 */

/* STC32G_Delay库有相关实现函数进行替代 */
#include "STC32G_Delay.h"
void mpu6500_delay_ms(int ms)
{
    delay_ms(ms);
}
#ifdef MPU6500_USING_SPI
void hal_spi_init()
{
    bsp_spi_init();
}
void hal_spi_read(unsigned char addr, unsigned char *buffer, int len)
{
    bsp_spi_read(addr, buffer, len);
}

void hal_spi_write(unsigned char addr, unsigned char *buffer, int len)
{
    bsp_spi_write(addr, buffer, len);
}
#endif

#ifdef MPU6500_USING_I2C
void hal_i2c_init()
{
    I2C_soft_GPIO_Config();
}
void hal_i2c_read(unsigned char addr, unsigned char *buffer, int len)
{
}

void hal_i2c_write(unsigned char addr, unsigned char *buffer, int len)
{
}
#endif
void MPU6500_Peripheral_Init()
{
#ifdef MPU6500_USING_I2C
    hal_i2c_init();
#endif
#ifdef MPU6500_USING_SPI
    hal_spi_init();
#endif
}
void MPU6500_Read(unsigned char addr, unsigned char *buffer, int len)
{
#ifdef MPU6500_USING_I2C
    hal_i2c_read(addr, buffer, len);
#endif
#ifdef MPU6500_USING_SPI
    hal_spi_read(addr, buffer, len);
#endif
}

void MPU6500_Write(unsigned char addr, unsigned char *buffer, int len)
{
#ifdef MPU6500_USING_I2C
    hal_i2c_write(addr, buffer, len);
#endif
#ifdef MPU6500_USING_SPI
    hal_spi_write(addr, buffer, len);
#endif
}
/**
 * *******************************************
 *
 *  兼容层捏 end
 *
 * *******************************************
 * */



void MPU6500_Write_u8(unsigned char addr, u8 buffer){
    u8 reg_buffer=buffer;
	MPU6500_Write(MPU6500_GYRO_CONFIG,&reg_buffer,1);
}
// void MPU6500_Write_u16(unsigned char addr, u16 *buffer){
//     u8 buffer_u8[2];
//     buffer_u8[]
//     MPU6500_Write(addr, buffer, 1);
// }
// void MPU6500_Write_u32(unsigned char addr, u32 *buffer){

// }
/* 定义MPU6500初始化函数 */
void MPU6500_Init()
{
    unsigned char reg_buffer;

    MPU6500_Peripheral_Init();

    // /* 设置陀螺仪采样率为1KHz */
    // reg_buffer = 0x00; //分频系数为0
    // MPU6500_Write(MPU6500_SMPLRT_DIV, &reg_buffer, 1);

    // /* 设置加速度计采样率为1KHz */
    // reg_buffer = 0x00; //分频系数为0
    // MPU6500_Write(MPU6500_ACCEL_CONFIG, &reg_buffer, 1);

    /* 设置陀螺仪自检 */
    reg_buffer = 0x80;
    MPU6500_Write(MPU6500_SELF_TEST_X_GYRO, &reg_buffer, 1);
    // 等待100ms，让陀螺仪完成自检
    delay_ms(40);

    /* 关闭陀螺仪自检 */
    reg_buffer = 0x00;
    MPU6500_Write(MPU6500_SELF_TEST_X_GYRO, &reg_buffer, 1);

    /* 打开陀螺仪和加速度计 */
    reg_buffer = 0x00;
    MPU6500_Write(MPU6500_PWR_MGMT_1, &reg_buffer, 1);

        reg_buffer = MPU6500_GYRO_FS_SEL_500dps;
    MPU6500_Write(MPU6500_GYRO_CONFIG, &reg_buffer, 1);
}

// /* 定义MPU6500六轴数据获取函数 */
// void MPU6500_get_buffer(float *gyro_buffer, float *acc_buffer)
// {
//     unsigned char buf[14];

//     /* 读取陀螺仪和加速度计数据 */
//     MPU6500_Read(MPU6500_ACCEL_XOUT_H, buf, (3 + 1 + 3) * 2); // 3个gyro轴数据+1个temp数据+3个acc数据

//     /* 解析陀螺仪数据 */
//     gyro_buffer[0] = (float)(((short)buf[8] << 8) | buf[9]) / 32768.0f * 250.0f;
//     gyro_buffer[1] = (float)(((short)buf[10] << 8) | buf[11]) / 32768.0f * 250.0f;
//     gyro_buffer[2] = (float)(((short)buf[12] << 8) | buf[13]) / 32768.0f * 250.0f;

//     /* 解析加速度计数据 */
//     acc_buffer[0] = (float)(((short)buf[0] << 8) | buf[1]) / 32768.0f * 2.0f;
//     acc_buffer[1] = (float)(((short)buf[2] << 8) | buf[3]) / 32768.0f * 2.0f;
//     acc_buffer[2] = (float)(((short)buf[4] << 8) | buf[5]) / 32768.0f * 2.0f;
// }

void MPU6500_get_buffer(float *gyro_buffer, float *acc_buffer)
{
    static float gyroLast[3] = {0, 0, 0}; // 上一次的陀螺仪数据
    static float accLast[3] = {0, 0, 0}; // 上一次的加速度计数据

    unsigned char buf[14];
		    float alphaGyro = 0.5f; // 陀螺仪数据的滤波系数
		    float alphaAcc = 0.5f; // 加速度计数据的滤波系数

    /* 读取陀螺仪和加速度计数据 */
    MPU6500_Read(MPU6500_ACCEL_XOUT_H, buf, (3 + 1 + 3) * 2); // 3个gyro轴数据+1个temp数据+3个acc数据

    /* 解析陀螺仪数据，使用互补滤波平滑数据 */

    gyro_buffer[0] = (float)(((short)buf[8] << 8) | buf[9]) / 32768.0f * 500.0f;
    gyro_buffer[1] = (float)(((short)buf[10] << 8) | buf[11]) / 32768.0f * 500.0f;
    gyro_buffer[2] = (float)(((short)buf[12] << 8) | buf[13]) / 32768.0f * 500.0f;
    gyro_buffer[0] = alphaGyro * gyroLast[0] + (1 - alphaGyro) * gyro_buffer[0];
    gyro_buffer[1] = alphaGyro * gyroLast[1] + (1 - alphaGyro) * gyro_buffer[1];
    gyro_buffer[2] = alphaGyro * gyroLast[2] + (1 - alphaGyro) * gyro_buffer[2];
    gyroLast[0] = gyro_buffer[0];
    gyroLast[1] = gyro_buffer[1];
    gyroLast[2] = gyro_buffer[2];

    // /* 解析加速度计数据，使用互补滤波平滑数据 */

    // acc_buffer[0] = (float)(((short)buf[0] << 8) | buf[1]) / 32768.0f * 2.0f;
    // acc_buffer[1] = (float)(((short)buf[2] << 8) | buf[3]) / 32768.0f * 2.0f;
    // acc_buffer[2] = (float)(((short)buf[4] << 8) | buf[5]) / 32768.0f * 2.0f;
    // acc_buffer[0] = alphaAcc * accLast[0] + (1 - alphaAcc) * acc_buffer[0];
    // acc_buffer[1] = alphaAcc * accLast[1] + (1 - alphaAcc) * acc_buffer[1];
    // acc_buffer[2] = alphaAcc * accLast[2] + (1 - alphaAcc) * acc_buffer[2];
    // accLast[0] = acc_buffer[0];
    // accLast[1] = acc_buffer[1];
    // accLast[2] = acc_buffer[2];
}


// int main()
// {
//     float gyro_buffer[3], acc_buffer[3];

//     /* 初始化MPU6500 */
//     mpu6500_init();

//     while (1)
//     {
//         /* 获取陀螺仪和加速度计数据 */
//         mpu6500_get_buffer(gyro_buffer, acc_buffer);

//         /* 打印数据 */
//         printf("Gyro: %.2f, %.2f, %.2f, Acc: %.2f, %.2f, %.2f\n", gyro_buffer[0], gyro_buffer[1], gyro_buffer[2], acc_buffer[0], acc_buffer[1], acc_buffer[2]);

//         /* 延时一段时间 */
//         delay_ms(10);
//     }

//     return 0;
// }

//void MPU6500_SelfCalibration(void)
//{
//    unsigned char buffer;
//    unsigned char rawData[6];      // 原始传感器数据
//    float gyroBias[3] = {0, 0, 0}; // 陀螺仪偏置
//    int i, j;
//    // 加速度计自校准
//    float accelBias[3] = {0, 0, 0}; // 加速度计偏置

//    // 温度稳定化
//    buffer = 0x40;
//    MPU6500_Write(0x1A, &buffer, 1); // 将Config寄存器的TEMP_FIFO_EN位置为1，使能温度传感器
//    mpu6500_delay_ms(200);           // 等待200ms，让温度稳定

//    // 陀螺仪自校准
//    for (i = 0; i < 1000; i++) // 取1000个样本进行平均值计算
//    {
//        MPU6500_Read(0x43, rawData, 6); // 读取陀螺仪X、Y、Z轴的原始数据
//        for (j = 0; j < 3; j++)
//        {
//            gyroBias[j] += (float)(((int16_t)rawData[j * 2] << 8) | rawData[j * 2 + 1]); // 累加每个轴的原始数据
//        }
//        mpu6500_delay_ms(10);
//    }
//    for (i = 0; i < 3; i++)
//    {
//        gyroBias[i] /= 1000.0f; // 计算平均值
//    }
//    MPU6500_Write(0x13, (unsigned char *)&gyroBias[0], 2); // 将陀螺仪X轴的偏置写入MPU6500芯片的GYRO_X_OFFS_H和GYRO_X_OFFS_L寄存器中
//    MPU6500_Write(0x15, (unsigned char *)&gyroBias[1], 2); // 将陀螺仪Y轴的偏置写入MPU6500芯片的GYRO_Y_OFFS_H和GYRO_Y_OFFS_L寄存器中
//    MPU6500_Write(0x17, (unsigned char *)&gyroBias[2], 2); // 将陀螺仪Z轴的偏置写入MPU6500芯片的GYRO_Z_OFFS_H和GYRO_Z_OFFS_L寄存器中

//    // 加速度计自校准
//    for (i = 0; i < 500; i++) // 取500个样本进行平均值计算
//    {
//        MPU6500_Read(0x3B, rawData, 6); // 读取加速度计X、Y、Z轴的原始数据
//        for (j = 0; j < 3; j++)
//        {
//            accelBias[j] += (float)(((int16_t)rawData[j * 2] << 8) | rawData[j * 2 + 1]); // 累加每个轴的原始数据
//        }
//        mpu6500_delay_ms(10);
//    }
//    for (i = 0; i < 3; i++)
//    {
//        accelBias[i] /= 500.0f; // 计算平均值
//    }
//    accelBias[2] -= 16384.0f;                               // 减去1g重力加速度对Z轴的影响

//    MPU6500_Write(0x06, (unsigned char *)&accelBias[0], 2); // 将加速度计X轴的偏置写入MPU6500芯片的ACCEL_X_OFFS_H和ACCEL_X_OFFS_L寄存器中
//    MPU6500_Write(0x08, (unsigned char *)&accelBias[1], 2); // 将加速度计Y轴的偏置写入MPU6500芯片的ACCEL_Y_OFFS_H和ACCEL_Y_OFFS_L寄存器中
//    MPU6500_Write(0x0A, (unsigned char *)&accelBias[2], 2); // 将加速度计Z轴的偏置写入MPU6500芯片的ACCEL_Z_OFFS_H和ACCEL_Z_OFFS_L寄存器中
//}

//void MPU6500_GYRO_CONFIG(){
//    MPU6500_Write_u8(MPU6500_GYRO_CONFIG,MPU6500_GYRO_FS_SEL_250dps);
//}
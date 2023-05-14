#include <stdio.h>
#include "MPU6500.h"
/* ѡ��ʹ��SPI����I2C����ͨѶ�󣬵���Ȼ��SPI�� */
#define MPU6500_USING_SPI
// #define MPU6500_USING_I2C

/**
 *
 * ********************************************************
 *
 *  ���ݲ��� HAL_level start
 *
 * ********************************************************
 *
 **/

/*
====================================
    ||
    ||
    ||  Ӳ��ʵ�ֲ㣬�����豸�Ļ������������ʵ��
    ||
    ||
+++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/
#include "config.h"
#include "bsp.h"
/**
 * @brief ͳһAPI����
 *
 */

/* STC32G_Delay�������ʵ�ֺ���������� */
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
 *  ���ݲ��� end
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
/* ����MPU6500��ʼ������ */
void MPU6500_Init()
{
    unsigned char reg_buffer;

    MPU6500_Peripheral_Init();

    // /* ���������ǲ�����Ϊ1KHz */
    // reg_buffer = 0x00; //��Ƶϵ��Ϊ0
    // MPU6500_Write(MPU6500_SMPLRT_DIV, &reg_buffer, 1);

    // /* ���ü��ٶȼƲ�����Ϊ1KHz */
    // reg_buffer = 0x00; //��Ƶϵ��Ϊ0
    // MPU6500_Write(MPU6500_ACCEL_CONFIG, &reg_buffer, 1);

    /* �����������Լ� */
    reg_buffer = 0x80;
    MPU6500_Write(MPU6500_SELF_TEST_X_GYRO, &reg_buffer, 1);
    // �ȴ�100ms��������������Լ�
    delay_ms(40);

    /* �ر��������Լ� */
    reg_buffer = 0x00;
    MPU6500_Write(MPU6500_SELF_TEST_X_GYRO, &reg_buffer, 1);

    /* �������Ǻͼ��ٶȼ� */
    reg_buffer = 0x00;
    MPU6500_Write(MPU6500_PWR_MGMT_1, &reg_buffer, 1);

        reg_buffer = MPU6500_GYRO_FS_SEL_500dps;
    MPU6500_Write(MPU6500_GYRO_CONFIG, &reg_buffer, 1);
}

// /* ����MPU6500�������ݻ�ȡ���� */
// void MPU6500_get_buffer(float *gyro_buffer, float *acc_buffer)
// {
//     unsigned char buf[14];

//     /* ��ȡ�����Ǻͼ��ٶȼ����� */
//     MPU6500_Read(MPU6500_ACCEL_XOUT_H, buf, (3 + 1 + 3) * 2); // 3��gyro������+1��temp����+3��acc����

//     /* �������������� */
//     gyro_buffer[0] = (float)(((short)buf[8] << 8) | buf[9]) / 32768.0f * 250.0f;
//     gyro_buffer[1] = (float)(((short)buf[10] << 8) | buf[11]) / 32768.0f * 250.0f;
//     gyro_buffer[2] = (float)(((short)buf[12] << 8) | buf[13]) / 32768.0f * 250.0f;

//     /* �������ٶȼ����� */
//     acc_buffer[0] = (float)(((short)buf[0] << 8) | buf[1]) / 32768.0f * 2.0f;
//     acc_buffer[1] = (float)(((short)buf[2] << 8) | buf[3]) / 32768.0f * 2.0f;
//     acc_buffer[2] = (float)(((short)buf[4] << 8) | buf[5]) / 32768.0f * 2.0f;
// }

void MPU6500_get_buffer(float *gyro_buffer, float *acc_buffer)
{
    static float gyroLast[3] = {0, 0, 0}; // ��һ�ε�����������
    static float accLast[3] = {0, 0, 0}; // ��һ�εļ��ٶȼ�����

    unsigned char buf[14];
		    float alphaGyro = 0.5f; // ���������ݵ��˲�ϵ��
		    float alphaAcc = 0.5f; // ���ٶȼ����ݵ��˲�ϵ��

    /* ��ȡ�����Ǻͼ��ٶȼ����� */
    MPU6500_Read(MPU6500_ACCEL_XOUT_H, buf, (3 + 1 + 3) * 2); // 3��gyro������+1��temp����+3��acc����

    /* �������������ݣ�ʹ�û����˲�ƽ������ */

    gyro_buffer[0] = (float)(((short)buf[8] << 8) | buf[9]) / 32768.0f * 500.0f;
    gyro_buffer[1] = (float)(((short)buf[10] << 8) | buf[11]) / 32768.0f * 500.0f;
    gyro_buffer[2] = (float)(((short)buf[12] << 8) | buf[13]) / 32768.0f * 500.0f;
    gyro_buffer[0] = alphaGyro * gyroLast[0] + (1 - alphaGyro) * gyro_buffer[0];
    gyro_buffer[1] = alphaGyro * gyroLast[1] + (1 - alphaGyro) * gyro_buffer[1];
    gyro_buffer[2] = alphaGyro * gyroLast[2] + (1 - alphaGyro) * gyro_buffer[2];
    gyroLast[0] = gyro_buffer[0];
    gyroLast[1] = gyro_buffer[1];
    gyroLast[2] = gyro_buffer[2];

    // /* �������ٶȼ����ݣ�ʹ�û����˲�ƽ������ */

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

//     /* ��ʼ��MPU6500 */
//     mpu6500_init();

//     while (1)
//     {
//         /* ��ȡ�����Ǻͼ��ٶȼ����� */
//         mpu6500_get_buffer(gyro_buffer, acc_buffer);

//         /* ��ӡ���� */
//         printf("Gyro: %.2f, %.2f, %.2f, Acc: %.2f, %.2f, %.2f\n", gyro_buffer[0], gyro_buffer[1], gyro_buffer[2], acc_buffer[0], acc_buffer[1], acc_buffer[2]);

//         /* ��ʱһ��ʱ�� */
//         delay_ms(10);
//     }

//     return 0;
// }

//void MPU6500_SelfCalibration(void)
//{
//    unsigned char buffer;
//    unsigned char rawData[6];      // ԭʼ����������
//    float gyroBias[3] = {0, 0, 0}; // ������ƫ��
//    int i, j;
//    // ���ٶȼ���У׼
//    float accelBias[3] = {0, 0, 0}; // ���ٶȼ�ƫ��

//    // �¶��ȶ���
//    buffer = 0x40;
//    MPU6500_Write(0x1A, &buffer, 1); // ��Config�Ĵ�����TEMP_FIFO_ENλ��Ϊ1��ʹ���¶ȴ�����
//    mpu6500_delay_ms(200);           // �ȴ�200ms�����¶��ȶ�

//    // ��������У׼
//    for (i = 0; i < 1000; i++) // ȡ1000����������ƽ��ֵ����
//    {
//        MPU6500_Read(0x43, rawData, 6); // ��ȡ������X��Y��Z���ԭʼ����
//        for (j = 0; j < 3; j++)
//        {
//            gyroBias[j] += (float)(((int16_t)rawData[j * 2] << 8) | rawData[j * 2 + 1]); // �ۼ�ÿ�����ԭʼ����
//        }
//        mpu6500_delay_ms(10);
//    }
//    for (i = 0; i < 3; i++)
//    {
//        gyroBias[i] /= 1000.0f; // ����ƽ��ֵ
//    }
//    MPU6500_Write(0x13, (unsigned char *)&gyroBias[0], 2); // ��������X���ƫ��д��MPU6500оƬ��GYRO_X_OFFS_H��GYRO_X_OFFS_L�Ĵ�����
//    MPU6500_Write(0x15, (unsigned char *)&gyroBias[1], 2); // ��������Y���ƫ��д��MPU6500оƬ��GYRO_Y_OFFS_H��GYRO_Y_OFFS_L�Ĵ�����
//    MPU6500_Write(0x17, (unsigned char *)&gyroBias[2], 2); // ��������Z���ƫ��д��MPU6500оƬ��GYRO_Z_OFFS_H��GYRO_Z_OFFS_L�Ĵ�����

//    // ���ٶȼ���У׼
//    for (i = 0; i < 500; i++) // ȡ500����������ƽ��ֵ����
//    {
//        MPU6500_Read(0x3B, rawData, 6); // ��ȡ���ٶȼ�X��Y��Z���ԭʼ����
//        for (j = 0; j < 3; j++)
//        {
//            accelBias[j] += (float)(((int16_t)rawData[j * 2] << 8) | rawData[j * 2 + 1]); // �ۼ�ÿ�����ԭʼ����
//        }
//        mpu6500_delay_ms(10);
//    }
//    for (i = 0; i < 3; i++)
//    {
//        accelBias[i] /= 500.0f; // ����ƽ��ֵ
//    }
//    accelBias[2] -= 16384.0f;                               // ��ȥ1g�������ٶȶ�Z���Ӱ��

//    MPU6500_Write(0x06, (unsigned char *)&accelBias[0], 2); // �����ٶȼ�X���ƫ��д��MPU6500оƬ��ACCEL_X_OFFS_H��ACCEL_X_OFFS_L�Ĵ�����
//    MPU6500_Write(0x08, (unsigned char *)&accelBias[1], 2); // �����ٶȼ�Y���ƫ��д��MPU6500оƬ��ACCEL_Y_OFFS_H��ACCEL_Y_OFFS_L�Ĵ�����
//    MPU6500_Write(0x0A, (unsigned char *)&accelBias[2], 2); // �����ٶȼ�Z���ƫ��д��MPU6500оƬ��ACCEL_Z_OFFS_H��ACCEL_Z_OFFS_L�Ĵ�����
//}

//void MPU6500_GYRO_CONFIG(){
//    MPU6500_Write_u8(MPU6500_GYRO_CONFIG,MPU6500_GYRO_FS_SEL_250dps);
//}
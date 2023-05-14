#ifndef _BSP_H
#define _BSP_H


void bsp_spi_init();
void bsp_spi_read(unsigned char addr, unsigned char *buffer, int len);
void bsp_spi_write(unsigned char addr, unsigned char *buffer, int len);

void bsp_uart_init(void);

#endif // !_BSP_H
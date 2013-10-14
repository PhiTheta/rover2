#ifndef _SPI_H_
#define _SPI_H_

#include <stm32f4xx.h>

#define CSPORT GPIOA
#define CSPIN GPIO_Pin_4

#define CS_low CSPORT->ODR &= ~CSPIN
#define CS_high CSPORT->ODR |= CSPIN

void spi_init(SPI_TypeDef* SPIx, unsigned int prescaler);
void spi_send_single(SPI_TypeDef* SPIx, unsigned char data);
unsigned char spi_receive_single(SPI_TypeDef* SPIx);
void spi_send(SPI_TypeDef* SPIx, unsigned char* data, unsigned int length);
void spi_receive(SPI_TypeDef* SPIx, unsigned char* data, unsigned int length);
void spi_transmit(SPI_TypeDef* SPIx, unsigned char* txbuf, unsigned char* rxbuf, unsigned int len);


#endif // _SPI_H_

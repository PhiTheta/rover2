#ifndef _I2C_H_
#define _I2C_H_

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
void I2C_stop(I2C_TypeDef* I2Cx);
void i2cMasterTransmit(I2C_TypeDef* I2Cx, uint8_t address, uint8_t* txbuffer, uint8_t txlen, uint8_t* rxbuffer, uint8_t rxlen);

#endif

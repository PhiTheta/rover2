#include "MPU6050.h"
#include "i2c.h"

static uint8_t i2crxbuf[14];		// I2C receive buffer
static uint8_t i2ctxbuf[2];		// I2C transmit buffer

void MPU6050_init(I2C_TypeDef* I2Cx)
{
	// wake up from sleep mode
	// PLL with X axis gyroscope reference
	i2ctxbuf[0] = MPU6050_RA_PWR_MGMT_1;
	i2ctxbuf[1] = MPU6050_CLOCK_PLL_XGYRO;
	i2cMasterTransmit(I2Cx, MPU6050_DEFAULT_ADDRESS, i2ctxbuf, 2, i2crxbuf, 0);
	
	// set gyro to 250°/s scale
	i2ctxbuf[0] = MPU6050_RA_GYRO_CONFIG;
	i2ctxbuf[1] = MPU6050_GYRO_FS_250<<3;	
	i2cMasterTransmit(I2Cx, MPU6050_DEFAULT_ADDRESS, i2ctxbuf, 2, i2crxbuf, 0);

	// set accelerometer to +- 2g scale
	i2ctxbuf[0] = MPU6050_RA_ACCEL_CONFIG;
	i2ctxbuf[1] = MPU6050_ACCEL_FS_2<<3;
	i2cMasterTransmit(I2Cx, MPU6050_DEFAULT_ADDRESS, i2ctxbuf, 2, i2crxbuf, 0);
	
	// set sample rate 8kHz / (SMPLRT_DIV + 1)
	i2ctxbuf[0] = MPU6050_RA_SMPLRT_DIV;
	i2ctxbuf[1] = 0x09;
	i2cMasterTransmit(I2Cx, MPU6050_DEFAULT_ADDRESS, i2ctxbuf, 2, i2crxbuf, 0);
}

void MPU6050_SetGyroFullScale(I2C_TypeDef* I2Cx, uint8_t scale)
{
	i2ctxbuf[0] = MPU6050_RA_GYRO_CONFIG;
	i2ctxbuf[1] = scale<<3;
	i2cMasterTransmit(I2Cx, MPU6050_DEFAULT_ADDRESS, i2ctxbuf, 2, i2crxbuf, 0);
}

void MPU6050_SetAccelFullScale(I2C_TypeDef* I2Cx, uint8_t scale)
{
	i2ctxbuf[0] = MPU6050_RA_ACCEL_CONFIG;
	i2ctxbuf[1] = scale<<3;
	i2cMasterTransmit(I2Cx, MPU6050_DEFAULT_ADDRESS, i2ctxbuf, 2, i2crxbuf, 0);
}

void MPU6050_read(I2C_TypeDef* I2Cx, int16_t* mpuAxisData)
{	
	// set register pointer to accel X axis MSB
	// and receive 14 bytes of data
	// 6 bytes gyro, 1 byte temp, 6 bytes accel
	i2ctxbuf[0] = MPU6050_RA_ACCEL_XOUT_H;
		
	i2cMasterTransmit(I2Cx, MPU6050_DEFAULT_ADDRESS, i2ctxbuf, 1, i2crxbuf, 14);

	// convert received 2s complement values to 16 bit signed int
	mpuAxisData[0] = i2crxbuf[0]<<8 | i2crxbuf[1]; // accel data
	mpuAxisData[1] = i2crxbuf[2]<<8 | i2crxbuf[3];
	mpuAxisData[2] = i2crxbuf[4]<<8 | i2crxbuf[5];
	mpuAxisData[3] = i2crxbuf[8]<<8 | i2crxbuf[9]; // gyro data
	mpuAxisData[4] = i2crxbuf[10]<<8 | i2crxbuf[11];
	mpuAxisData[5] = i2crxbuf[12]<<8 | i2crxbuf[13];
}

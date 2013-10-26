#ifndef COMPASS_H
#define COMPASS_H
 
#include <stm32f4xx.h> 

// register map
#define HMC5883L_ADDRESS 	0x1E
#define HMC5883L_CTRL_REG1  0x00
#define HMC5883L_CTRL_REG2  0x01
#define HMC5883L_MODE_REG 	0x02
#define HMC5883L_X_MSB   	0x03
#define HMC5883L_X_LSB		0x04
#define HMC5883L_Z_MSB		0x05
#define HMC5883L_Z_LSB		0x06
#define HMC5883L_Y_MSB		0x07
#define	HMC5883L_Y_LSB		0x08
#define HMC5883L_STATUS_REG 0x09
#define HMC5883L_ID_REG1	0x0A
#define HMC5883L_ID_REG2 	0x0B
#define HMC5883L_ID_REG3	0x0C

// CRA bits
#define HMC5883L_MEAS_NORM	0x00
#define HMC5883L_MEAS_POSB	0x01
#define HMC5883L_MEAS_NEGB	0x02
#define HMC5883L_ODR_0075	0x00
#define HMC5883L_ODR_0150	0x04
#define HMC5883L_ODR_0300	0x08
#define HMC5883L_ODR_0750	0x0C
#define HMC5883L_ODR_1500	0x10
#define HMC5883L_ODR_3000	0x14
#define HMC5883L_ODR_7500	0x18
#define HMC5883L_OVER_1 	0x00
#define HMC5883L_OVER_2		0x20
#define HMC5883L_OVER_4		0x40
#define HMC5883L_OVER_8		0x60

// CRB bits
#define HMC5883L_FS_088		0x00
#define HMC5883L_FS_130		0x20
#define HMC5883L_FS_190		0x40
#define HMC5883L_FS_250		0x60
#define HMC5883L_FS_400		0x80
#define HMC5883L_FS_470		0xA0
#define HMC5883L_FS_560		0xC0
#define HMC5883L_FS_810		0xE0

// Mode Register bits
#define HMC5883L_MODE_CONT	0x00
#define HMC5883L_MODE_SING	0x01
#define HMC5883L_MODE_IDLE	0x02
#define HMC5883L_MODE_HS	0x80

// Status register bits
#define HMC5883L_STAT_LOCK	0x01
#define HMC5883L_STAT_RDY	0x02

struct HMC5883L {
	I2C_TypeDef* i2cp;		// I2C port 
	uint8_t measbias;		// bias configuration of sensors
	uint8_t outdatarate;	// output data rate
	uint8_t oversample;		// number of samples to be averaged
	uint8_t fullscale;		// axis fullscale
	uint8_t mode;		
	int16_t axisRaw[3];		// axis raw data
	float axisScaled[3];	// axis scaled data
	float scalefactor;		// scale factor to determine Gauss reading from raw data
	float headingDegrees;	// calculated heading to magnetic north in degrees
};

void HMC5883L_init(struct HMC5883L* mag);
void HMC5883L_readRaw(struct HMC5883L* mag);
void HMC5883L_readScaled(struct HMC5883L* mag);
void HMC5883L_heading(struct HMC5883L* mag, float* accAxisScaled);

#endif /* COMPASS_H */

#include <math.h>
#include "HMC5883L.h"
#include "i2c.h"

#define HMC5883L_ADDRESS 	0x1E
#define HMC5883L_CTRL_REG1  0x00
#define HMC5883L_CTRL_REG2  0x01
#define HMC5883L_MODE_REG 	0x02
#define HMC5883L_OUT_DATA   0x03


static uint8_t i2crxbuf[6];		// I2C receive buffer
static uint8_t i2ctxbuf[4];		// I2C transmit buffer
static float headingDegrees = 0;

// initialize HMC5883L on I2C bus passed in i2cp
void HMC5883L_init(I2C_TypeDef* I2Cx)
{
	// set pointer to CRA, register pointer is incremented automatically
	i2ctxbuf[0] = HMC5883L_CTRL_REG1; // CRA address
	i2ctxbuf[1] = 0x70; // 
	i2ctxbuf[2] = 0xA0; // 
	i2ctxbuf[3] = 0x00; // continous measurement

	i2cMasterTransmit(I2Cx, HMC5883L_ADDRESS, i2ctxbuf, 4, i2crxbuf, 0); // configure magnetometer
}

void HMC5883L_read(I2C_TypeDef* I2Cx, int16_t* magAxisData)
{
	// set register pointer to X axis MSB
	i2ctxbuf[0] = HMC5883L_OUT_DATA;
		
	// transmit register address and read back 6 bytes of data
	i2cMasterTransmit(I2Cx, HMC5883L_ADDRESS, i2ctxbuf, 1, i2crxbuf, 6);

	magAxisData[0] = i2crxbuf[0]<<8; // X axis
	magAxisData[0] |= i2crxbuf[1];

	magAxisData[1] = i2crxbuf[4]<<8; // Y axis
	magAxisData[1] |= i2crxbuf[5];

	magAxisData[2] = i2crxbuf[2]<<8; // Z axis
	magAxisData[2] |= i2crxbuf[3];
}

// reads HMC5883L axis data and calculates heading in degrees from magnetic north
// using accelerometer data to compensate for tilt
float HMC5883L_heading(int16_t* magAxisData, int16_t* accAxisData)
{
	float Xh, Yh;
	float Xm, Ym, Zm;
	float Xa, Ya, Za;
	float pitchRadians, rollRadians;
	float cosRoll, sinRoll, cosPitch, sinPitch;

	// convert magnetometer readings to Gauss
	Xm = magAxisData[0] * (9.4 / 4096);
	Ym = magAxisData[1] * (9.4 / 4096);
	Zm = magAxisData[2] * (9.4 / 4096);
		
	// convert accelerometer data to g
	Xa = (float)accAxisData[0] / 16384;
	Ya = (float)accAxisData[1] / 16384;
	Za = (float)accAxisData[2] / 16384;
	
	// calculate these beforehand rather than calculating them twice
	pitchRadians = asin(Xa);
	rollRadians = asin(Ya);
	
	cosRoll = cos(rollRadians);
	sinRoll = sin(rollRadians);  
	cosPitch = cos(pitchRadians);
	sinPitch = sin(pitchRadians);
	
	Xh = Xm * cosPitch + Zm * sinPitch;
	Yh = Xm * sinRoll * sinPitch + Ym * cosRoll - Zm * sinRoll * cosPitch;
	
	headingDegrees = atan2(Yh, Xh) * 180 / 3.141592654 + 180;
	
	//headingDegrees = atan2((double)magAxisData[1],(double)magAxisData[0]) * 180 / 3.141592654 + 180; 
	return headingDegrees;
}

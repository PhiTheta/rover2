#include <math.h>
#include "HMC5883L.h"
#include "i2c.h"

static uint8_t i2crxbuf[6];		// I2C receive buffer
static uint8_t i2ctxbuf[4];		// I2C transmit buffer
static float headingDegrees = 0;

// initialize HMC5883L on I2C bus passed in i2cp
void HMC5883L_init(struct HMC5883L* mag)
{
	// set pointer to CRA, register pointer is incremented automatically
	i2ctxbuf[0] = HMC5883L_CTRL_REG1;
	i2ctxbuf[1] = mag->measbias | mag->outdatarate | mag->oversample;
	i2ctxbuf[2] = mag->fullscale;
	i2ctxbuf[3] = mag->mode;

	// figure out the fullscale of the magnetometer
	switch(mag->fullscale)
	{
		case HMC5883L_FS_088:
			mag->scalefactor = (1.76 / 4096);
			break;
		
		case HMC5883L_FS_130:
			mag->scalefactor = (2.6 / 4096);
			break;
		
		case HMC5883L_FS_190:
			mag->scalefactor = (3.8 / 4096);
			break;
			
		case HMC5883L_FS_250:
			mag->scalefactor = (5 / 4096);
			break;
			
		case HMC5883L_FS_400:
			mag->scalefactor = (8 / 4096);
			break;
			
		case HMC5883L_FS_470:
			mag->scalefactor = (9.4 / 4096);
			break;
			
		case HMC5883L_FS_560:
			mag->scalefactor = (11.2 / 4096);
			break;
			
		case HMC5883L_FS_810:
			mag->scalefactor = (16.2 / 4096);
			break;
	}

	i2cMasterTransmit(mag->i2cp, HMC5883L_ADDRESS, i2ctxbuf, 4, i2crxbuf, 0); // configure magnetometer
}

void HMC5883L_readRaw(struct HMC5883L* mag)
{
	// set register pointer to X axis MSB
	i2ctxbuf[0] = HMC5883L_X_MSB;
		
	// transmit register address and read back 6 bytes of data
	i2cMasterTransmit(mag->i2cp, HMC5883L_ADDRESS, i2ctxbuf, 1, i2crxbuf, 6);

	mag->axisRaw[0] = i2crxbuf[0]<<8; // X axis
	mag->axisRaw[0] |= i2crxbuf[1];

	mag->axisRaw[1] = i2crxbuf[4]<<8; // Y axis
	mag->axisRaw[1] |= i2crxbuf[5];

	mag->axisRaw[2] = i2crxbuf[2]<<8; // Z axis
	mag->axisRaw[2] |= i2crxbuf[3];
}

void HMC5883L_readScaled(struct HMC5883L* mag)
{
	HMC5883L_readRaw(mag);
	mag->axisScaled[0] = mag->axisRaw[0] * mag->scalefactor;
	mag->axisScaled[1] = mag->axisRaw[1] * mag->scalefactor;
	mag->axisScaled[2] = mag->axisRaw[2] * mag->scalefactor;
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

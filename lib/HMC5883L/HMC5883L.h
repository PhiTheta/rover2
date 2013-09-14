#ifndef COMPASS_H
#define COMPASS_H
 
#include <stm32f4xx.h> 

void HMC5883L_init(I2C_TypeDef* i2cp);
void HMC5883L_read(I2C_TypeDef* i2cp, int16_t magAxisData[]);
float HMC5883L_heading(int16_t* magAxisData);

#endif /* COMPASS_H */

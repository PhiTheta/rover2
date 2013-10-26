/* Pin connections
 * USART1_TX: PB6; USART1_RX: PB7
 * USART2_TX: PA2
 * I2C1_SDA: PB9; I2C1_SCL: PB8
 */
#include <stm32f4xx.h>
#include <misc.h> // I recommend you have a look at these in the ST firmware folder under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "gps.h"
#include "i2c.h"
#include "ff.h"
#include "MPU6050.h"
#include "HMC5883L.h"

volatile uint16_t stringcnt = 60;
volatile uint8_t icomp = 0;
FATFS FatFs;
FIL fil;

float headingDegrees;

#define NMEALEN 80
struct gps_data_t my_gps;
struct HMC5883L mag;

static char nmeachar = 0; // index for character number
char nmeabuffer[NMEALEN];
char buffer[100];
char timebuf[20];

#define EarthRadius 6371 // mean Earth radius in km
#define d2r (3.14159265/180)
#define r2d (180/3.14159265)

int16_t mpuData[6];
int16_t Ax,Ay,Az,Rx,Ry,Rz;

int16_t hmcData[6];

uint16_t TimerPeriod;
uint16_t Channel1Width, Channel2Width;

void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

void GPIO_init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12; // we want to configure all LED GPIO pins
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOD, &GPIO_InitStruct); 			// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;		  // we want to configure PA0
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 	  // we want it to be an input
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;   // this enables the pulldown resistor --> we want to detect a high level
	GPIO_Init(GPIOA, &GPIO_InitStruct);			  // this passes the configuration to the Init function which takes care of the low level stuff
}

void USART1_init(uint32_t baudrate){
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); // TX
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); // RX
	
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt 
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff	

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

void USART2_init(uint32_t baudrate){
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART1); 	

	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting
	
	// finally this enables the complete USART2 peripheral
	USART_Cmd(USART2, ENABLE);
}

void I2C1_init(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; // we are going to use PB8 and PB9
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // SDA
	
	I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1
	
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

void Timer4_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	TimerPeriod = (SystemCoreClock / (4 * 50) )-1; // 20 ms Period = 50Hz
	Channel1Width = ((TimerPeriod * 16) / 200); // 1.66 ms TIM4_CH1 PD12
	Channel2Width = ((TimerPeriod * 14) / 200); // 1.4 ms TIM4_CH3 PD14
	/*
	Channel1Width = TimerPeriod / 2;
	Channel2Width = TimerPeriod / 2;
	
	Channel1Width = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
	Channel2Width = (uint16_t) (((uint32_t) 5 * (TimerPeriod - 1)) / 10);
	*/
	// enable Timer 4 clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	
	// Timer 4 configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 9;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	// PWM configuration
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;

	TIM_OCInitStructure.TIM_Pulse = Channel1Width;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel2Width;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	
	// enable GPIOD clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 		// we want the pins to be an output
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 		// this sets the pin type to push / pull (as opposed to open drain)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
	GPIO_Init(GPIOB, &GPIO_InitStructure); 				// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	
	// TIM4 counter enable
	TIM_Cmd(TIM4, ENABLE);

	// TIM4 Main Output Enable	  
	TIM_CtrlPWMOutputs(TIM4, ENABLE);
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) ); 
		USART_SendData(USARTx, *s);
		*s++;
	}
}

void USART_putc(USART_TypeDef* USARTx, char c)
{
	while ( !(USARTx->SR & 0x00000040) );
	USART_SendData(USARTx, c);
}

// calculates the distance between two points given their latitude and longitude in degrees
unsigned int distance(float lat1, float lon1, float lat2, float lon2)
{
	float dLat, dLon, a;
	unsigned int d;	
	dLat = (lat2-lat1)*d2r; // delta between latitudes in radians
	dLon = (lon2-lon1)*d2r; // delta between longitudes in radians
	lat1 = lat1*d2r; // convert latitude 1 to radians
	lat2 = lat2*d2r; // convert latitude 2 to radians
	a = sin(dLat/2) * sin(dLat/2) + cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
	// calculate distance between locations in meters
	d = (unsigned int)(2*EarthRadius*atan2(sqrt(a), sqrt(1-a))*1000);
	return d;
}

unsigned int bearing(float lat1, float lon1, float lat2, float lon2)
{
	float dLon, x, y;
    unsigned int bearing;
    dLon = (lon2-lon1)*d2r; // delta between longitudes in radians
    lat1 = lat1*d2r; // convert latitude 1 to radians
    lat2 = lat2*d2r; // convert latitude 2 to radians
	y = sin(dLon) * cos(lat2);
	x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
	bearing = (unsigned int)((atan2(y, x)*r2d)+360)%360;
	return bearing;
}

int main(void) {
	// set CP10 and CP11 Full Access 
	// enable hardware FPU
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  
  
	USART1_init(9600); // initialize USART1 @ 9600 baud
	USART2_init(9600);
	I2C1_init();
	GPIO_init();
	//Timer4_init();
	MPU6050_init(I2C1);
	
	mag.i2cp = I2C1;
	mag.fullscale = HMC5883L_FS_470;
	mag.measbias = HMC5883L_MEAS_NORM;
	mag.mode = HMC5883L_MODE_CONT;
	mag.outdatarate = HMC5883L_ODR_1500;
	mag.oversample = HMC5883L_OVER_8;
	HMC5883L_init(&mag);
	
	USART_puts(USART2, "Init complete! Hello World!\r\n"); // just send a message to indicate that it works
	
	GPIOD->ODR |= GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;
	Delay(20000000L);
	GPIOD->ODR &= ~(GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12);

	/*
	GPIOD->ODR |= GPIO_Pin_14;
	f_mount(&FatFs, "", 1);
	f_open(&fil, "IMU.csv", FA_OPEN_EXISTING | FA_WRITE);
	GPIOD->ODR &= ~GPIO_Pin_14;
	
	f_puts("Ax,Ay,Az,Rx,Ry,Rz,Mx,My,Mz\n", &fil); // print file header
	
	for (uint16_t i = 0; i < 100; i++)
	{
		GPIOD->ODR |= GPIO_Pin_15; // set PD12 thru PD15
		MPU6050_read(I2C1, mpuData);
		HMC5883L_read(I2C1, hmcData);		
		sprintf(buffer, "%i,%i,%i,%i,%i,%i,%i,%i,%i\n", mpuData[0], mpuData[1], mpuData[2], mpuData[3], mpuData[4], mpuData[5], hmcData[0], hmcData[1], hmcData[2]);
		f_puts(buffer, &fil);
		GPIOD->ODR &= ~GPIO_Pin_15; // reset PD12 thru PD15
		Delay(1000000L);
	}
		
	f_close(&fil);
	f_mount(0, 0, 0);
	*/
	
	while (1){  
		
		GPIOD->ODR |= GPIO_Pin_15; // set PD12 thru PD15
		
		MPU6050_read(I2C1, mpuData);
		HMC5883L_readScaled(&mag);
	 
		
		//headingDegrees = HMC5883L_heading(hmcData, mpuData);
	 
		GPIOD->ODR &= ~GPIO_Pin_15;
	 
		//sprintf(buffer, "Heading: %f\n", headingDegrees);
		sprintf(buffer, "Xm: %f Ym: %f Zm: %f\n", mag.axisScaled[0], mag.axisScaled[1], mag.axisScaled[2]);
		USART_puts(USART2, buffer);
	 
		Delay(1000000L);
		/*
		sprintf(buffer, "Xm: %f Ym: %f Zm: %f\n", Xm,Ym,Zm);
		USART_puts(USART2, buffer);
		
		sprintf(buffer, "Xar: %i Yar: %i Zar: %i\nXa: %f Ya: %f Za: %f\n", mpuData[0], mpuData[1], mpuData[2], Xa,Ya,Za);
		USART_puts(USART2, buffer);
		
		sprintf(buffer, "Pitch: %f, Roll: %f\ncP: %f sP: %f cR: %f sR: %f\n", pitchRadians, rollRadians, cosPitch, sinPitch, cosRoll, sinRoll);
		USART_puts(USART2, buffer);
		
		sprintf(buffer, "Xh: %f Yh: %f\n", Xh,Yh);
		USART_puts(USART2, buffer);
		
		sprintf(buffer, "Heading: %f\n\n", headingDegrees);
		USART_puts(USART2, buffer);
		*/
		
		//GPIOD->ODR |= GPIO_Pin_15; // set PD12 thru PD15
		/*
		unix_to_iso8601(my_gps.fix.time, timebuf, 20);
		sprintf(buffer, "%s\n", timebuf);
		f_puts(buffer, &fil);
		*/

		//unsigned int dist1 = distance((float)my_gps.fix.latitude, (float)my_gps.fix.longitude, (float)50.759117, (float)8.803984);
		//unsigned int dist2 = distance((float)my_gps.fix.latitude, (float)my_gps.fix.longitude, (float)50.756866, (float)8.787066);
		//unsigned int bear1 = bearing((float)my_gps.fix.latitude, (float)my_gps.fix.longitude, (float)50.759117, (float)8.803984);
		//unsigned int bear2 = bearing((float)my_gps.fix.latitude, (float)my_gps.fix.longitude, (float)50.756866, (float)8.787066);

		/*
		sprintf(buffer, "AccelX: %i\n", mpuData[0]);
		USART_puts(USART2, buffer);
		sprintf(buffer, "AccelY: %i\n", mpuData[1]);
		USART_puts(USART2, buffer);
		sprintf(buffer, "AccelZ: %i\n", mpuData[2]);
		USART_puts(USART2, buffer);
		sprintf(buffer, "GyroX: %i\n", mpuData[3]);
		USART_puts(USART2, buffer);
		sprintf(buffer, "GyroY: %i\n", mpuData[4]);
		USART_puts(USART2, buffer);
		sprintf(buffer, "GyroZ: %i\n", mpuData[5]);
		USART_puts(USART2, buffer);
		USART_putc(USART2, '\n');
		
		sprintf(buffer, "MagX: %i\n", hmcData[0]);
		USART_puts(USART2, buffer);
		sprintf(buffer, "MagY: %i\n", hmcData[1]);
		USART_puts(USART2, buffer);
		sprintf(buffer, "MagZ: %i\n", hmcData[2]);
		USART_puts(USART2, buffer);
		USART_putc(USART2, '\n');
		*/

		//GPIOD->ODR &= ~GPIO_Pin_15; // reset PD12 thru PD15
		
		//Delay(1000000L);
		//Delay(20000000L);
	}
}

// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void){
	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){		
		GPIOD->ODR |= GPIO_Pin_13;
		char charbuf = USART1->DR; // the character from the USART1 data register is saved in t
		if (charbuf != '\n')
		{
			nmeabuffer[nmeachar] = charbuf;
			nmeachar++;
		}
		else
		{
			nmeachar = 0;
			nmea_parse(nmeabuffer, &my_gps);
		}
		GPIOD->ODR &= ~GPIO_Pin_13;
	}
}

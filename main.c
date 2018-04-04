/*-------------------------------------------------------------------------------------------
 ********************************************************************************************
 *-------------------------------------------------------------------------------------------
 *
 *				DATA LOGGER DE VARIABLES AMBIENTALES INTERNAS
 *							CIMEC CONICET - UTN FRP
 *								     2016
 *
 *						Polo, Franco		fjpolo@frp.utn.edu.ar
 *						Burgos, Sergio		sergioburgos@frp.utn.edu.ar
 *						Bre, Facundo		facubre@cimec.santafe-conicet.gov.ar
 *
 *	main.c
 *
 *	Descripción:
 *
 *  Desarrollo del firmware de la placa base del data logger, constando de:
 *
 *  - Periféricos I2C:
 *  	a) HR y Tbs		HIH9131		0b0100111		0x27
 *  	b) Ev			TSL2563		0b0101001		0x29
 *  	c) Va			ADS			0b1001000		0x48
 *  	d) Tg			LM92		0b1001011		0x51
 *  	e) RTC			DS1703		0b1101000		0x68
 *
 *  - Periféricos OneWire@PD6
 *  	a) Ts01			MAX31850	ROM_Addr		0x3B184D8803DC4C8C
 *  	b) Ts02			MAX31850	ROM_Addr		0x3B0D4D8803DC4C3C
 *  	c) Ts03			MAX31850	ROM_Addr		0x3B4D4D8803DC4C49
 *  	d) Ts04			MAX31850	ROM_Addr		0x3B234D8803DC4C99
 *  	e) Ts05			MAX31850	ROM_Addr		0x3B374D8803DC4C1E
 *  	f) Ts06			MAX31850	ROM_Addr
 *
 *  	//3
 *		//1
 *   	//2
 *		//4
 *		//0
 *
 *  - IHM
 *  	a) RESET		!RST
 *  	b) SW_SD		PC6
 *  	c) SW_ON		PC5
 *  	d) SW_1			PC7
 *  	e) WAKE			!Wake
 *  	f) LEDON		PE0
 *  	g) LED1			PE1
 *  	h) LED2			PE2
 *
 *  - SD
 *  	a) SD_IN		PA6
 *  	b) SD_RX		PA4
 *  	c) SD_TX		PA5
 *  	d) SD_CLK		PA2
 *  	e) SD_FSS		PA3
 *
 *
 *  - USB Commands
 *  	0x33		Set calendar
 *  	0x34		Set period
 *  	0x35		Start logging
 *  	0x36		Stop logging
 *
 *  	0x40		Sending Tbs
 *  	0x41		Sending HR
 *  	0x42		Sending Tg
 *  	0x43		Sending Ev
 *  	0x44		Sending Va
 *  	0x45		Sending Ts1
 *  	0x46		Sending Ts2
 *  	0x47		Sending Ts3
 *  	0x48		Sending Ts4
 *  	0x49		Sending Ts5
 *  	0x50		Sending Ts6
 *
 *--------------------------------------------------------------------------------------------
 *********************************************************************************************
 *-------------------------------------------------------------------------------------------*/

// standard C
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
// inc
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
// driverlib
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/hibernate.h"
#include "driverlib/i2c.h"
// datalogger
#include "datalogger/datalogger.h"
#include "datalogger/delay.h"
// third_party
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"
// usbserial
#include "usbserial.h"
//#include "usbcdc.h"
// leds
#include "leds.h"
// OneWire
#include "oneWire.h"

/*****************************************************************************************************
 * Defines
 ****************************************************************************************************/
// SWITCHES
#define SW_PORT 	GPIO_PORTC_BASE
#define SW_ON 		GPIO_PIN_5
#define SW_SD 		GPIO_PIN_6
#define SW_1 		GPIO_PIN_7
// LEDS
#define LED_PORT 	GPIO_PORTE_BASE
#define LED_ON 		GPIO_PIN_0
#define LED_1 		GPIO_PIN_1
#define LED_2 		GPIO_PIN_2
// SD Card
#define SD_PORT		GPIO_PORTA_BASE
#define SD_IN		GPIO_PIN_6
#define SD_RX		GPIO_PIN_4
#define SD_TX		GPIO_PIN_5
#define SD_CLK		GPIO_PIN_2
#define SD_FSS		GPIO_PIN_3
// Timer0
#define TOGGLE_FREQUENCY 1
// Tg
#define SLAVE_ADDRESS_TG 0x4B
#define TG_REG_READ 0x00
#define TG_REG_LOWPOW 0x01
// Ev
#define SLAVE_ADDRESS_EV 0x29
// RTC
#define SLAVE_ADDR_RTC 0x68
#define SEC 0x00
#define MIN 0x01
#define HRS 0x02
#define DAY 0x03
#define DATE 0x04
#define MONTH 0x05
#define YEAR 0x06
#define CNTRL 0x07
// Va ADC slave address
#define SLAVE_ADDR_VA 0x48
#define VA_REG_READ 0x00
// I2C3
#define GPIO_PD0_I2C3SCL        0x00030003
#define GPIO_PD1_I2C3SDA        0x00030403
// SSI1

/*********************************************************************************************
 * Global variables
 * ******************************************************************************************/
//
int SWRead;
//
int SWRead, SD_Read;
// Estado del modulo hibernacion
unsigned long ulStatus;
unsigned long ulPeriod;
//Datos a mantener durante la hibernacion
unsigned long HibReg[12] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//0: sensor flag
//1: hibernate flag
//2-3: year
//4-5: month
//6-7: day
//8-9: hour
//10-11: time
//
//
// SD Card variables
//
FATFS FatFs;    	/* Work area (file system object) for logical drive */
FIL fil;        	/* File object */
//
FRESULT fr;     	/* FatFs return code */
FILINFO fno;
UINT br;    		/* File read count */
//
DIR *dir;
//static FILINFO fileInfo;
//
int i=0;
const char null[]="0";
const char newline[] = "\r\n";
const char comma[] = ";";
char line[512];
int filesize;
char dir_date[9];
char dir_time[6];
char usbdate[9];
char usbtime[9];
//
GPIOIntState;
//
int FirstlineFlag = 0;
int USBDataloggingFlag = 0;
int USBProcessDataFlag = 0;
int USBStartLoggingFlag = 0;
unsigned char RxDataFrame[9];
int j=0;
unsigned char PeriodBytes[4];
//

/*********************************************************************************************
 * Main loop
 *
 * Usando USB sin tarjeta de memoria
 *
 ********************************************************************************************/
int main(void) {
	//FPULazyStackingEnable();
	Initialize();
	//
	//leds_init();
	//Enable hardware interrupt
	IntEnable(INT_GPIOC);
	//
	// While loop
	//
	while(1)
	{
		//
		// Enable USB
		//
		if(USBDataloggingFlag == 1)
		{
			// Configure and initialize USB
			USBSerialInit();
			// Flag to process USB Rx data
			USBDataloggingFlag = 2;
			//
			// Initialize OneWire module
			InitOneWire(1);
			// Get OneWire ROM addresses
			OneWiregetROMs();
			delayMS(5);
		}
		//End enable USB

		//
		// Receive USB Data
		//
		if((USBDataloggingFlag == 2) && (USBProcessDataFlag == 1))
		{
			// Set flag to down, process Rx data one time
			USBProcessDataFlag = 0;
			//
			// Switch frame received
			//
			switch (RxDataFrame[1])
			{
			case 0x33:
				// Configure clock
				SetTimeDate(RxDataFrame[7], RxDataFrame[6], RxDataFrame[5],1,RxDataFrame[4],RxDataFrame[3],RxDataFrame[2]);
				//
				break;
				// end case 0x33
			case 0x34:
				// Get period from USB variable
				for(j=0;j<=3;j++)
				{
					PeriodBytes[j] = RxDataFrame[j+2];
				}
				/*
				ulPeriod = PeriodBytes[0] << 24;
				ulPeriod += PeriodBytes[1] << 16;
				ulPeriod += PeriodBytes[2] << 8;
				ulPeriod += PeriodBytes[3];
				*/
				// TODO write period in FLASH
				// end case 0x34
				break;
			case 0x35:
				//USBStartLoggingFlag = 1;
				USBSendLine();
				break;
			}
			// end switch
		}
		// end if
	}
	//end while loop
}

/*********************************************************************************************
 * GPIOPortC_IRQHandler
 * ******************************************************************************************/
void GPIOPortC_IRQHandler(void)
{
	//Check interrupt source
	GPIOIntState = GPIOIntStatus(SW_PORT,true);
	//
	if((GPIOIntState & SW_1) == SW_1)
	{
		// IF USB NOT CONNECTED
		//if(!isCDCConnected()){
		if(!USBDataloggingFlag){
			//Flag de que estoy en la interrupcion por GPIO
			GPIOPinWrite(LED_PORT,LED_2, LED_2);//
			delayMS(500);
			GPIOPinWrite(LED_PORT,LED_2, 0);
			//
			// Nuevo archivo
			//
			FirstlineFlag = 1;
			//WriteFirstLine(&dir_date, &dir_time);
			//
			// Flag
			HibReg[1]=1;
			//end if
		}
		//end if
	}
	if((GPIOIntState & SW_SD) == SW_SD)
	{
		// Check if USB is active
		if(USBDataloggingFlag){
			//
			GPIOPinWrite(LED_PORT,LED_1, 0);
			GPIOPinWrite(LED_PORT,LED_1, LED_1);
			delayMS(200);
			GPIOPinWrite(LED_PORT,LED_1, 0);
			delayMS(100);
			GPIOPinWrite(LED_PORT,LED_1, LED_1);
			delayMS(200);
			//
			USBDataloggingFlag = 0;
			//Disable USB
			USBSerialKaput();
			//
			GPIOPinWrite(LED_PORT,LED_1, 0);
		}
		else if (!USBDataloggingFlag)
		{
			//
			GPIOPinWrite(LED_PORT,LED_1, LED_1);//
			delayMS(500);
			// Flag to configure and initialize USB
			USBDataloggingFlag = 1;
			//
			GPIOPinWrite(LED_PORT,LED_1, 0);
		}
	}
	// Clear interrupt source
	GPIOIntClear(SW_PORT, GPIOIntState);
}

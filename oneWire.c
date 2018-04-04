#include "oneWire.h"
#include <stdbool.h>
#include <stdint.h>

#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>

volatile timerCfg usedTimer;

const timerHW timerList[6] = {
		{SYSCTL_PERIPH_TIMER0, TIMER0_BASE, INT_TIMER0A},
		{SYSCTL_PERIPH_TIMER1, TIMER1_BASE, INT_TIMER1A},
		{SYSCTL_PERIPH_TIMER2, TIMER2_BASE, INT_TIMER2A},
		{SYSCTL_PERIPH_TIMER3, TIMER3_BASE, INT_TIMER3A},
		{SYSCTL_PERIPH_TIMER4, TIMER4_BASE, INT_TIMER4A},
		{SYSCTL_PERIPH_TIMER5, TIMER5_BASE, INT_TIMER5A}};

static void timmerISR(void)
{
	usedTimer.onTimer = true;
	TimerIntClear(timerList[usedTimer.timerId].timerBase, TIMER_TIMA_TIMEOUT);
}

static unsigned int __FMeg = 40;
void timerInit(uint8_t timerId)
{
	__FMeg = SysCtlClockGet() / 1000000;
	usedTimer.timerId = timerId;
	SysCtlPeripheralEnable(timerList[usedTimer.timerId].timerPeriph);
	while(!SysCtlPeripheralReady(timerList[usedTimer.timerId].timerPeriph));
	TimerConfigure(timerList[usedTimer.timerId].timerBase, TIMER_CFG_A_ONE_SHOT);
	TimerIntRegister(timerList[usedTimer.timerId].timerBase, TIMER_A, timmerISR);
}

void timerDelayMs(uint32_t delay)
{
	double delayS = delay / 1000.0;
	unsigned long counterVal = (unsigned long)(SysCtlClockGet() * delayS) / 2;
	usedTimer.onTimer = false;
	TimerLoadSet(timerList[usedTimer.timerId].timerBase, TIMER_A, counterVal - 1);
	TimerEnable(timerList[usedTimer.timerId].timerBase, TIMER_A);
	IntEnable(timerList[usedTimer.timerId].intTimer);
	TimerIntEnable(timerList[usedTimer.timerId].timerBase, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	while(!usedTimer.onTimer);
	TimerIntDisable(timerList[usedTimer.timerId].timerBase, TIMER_TIMA_TIMEOUT);
	IntDisable(timerList[usedTimer.timerId].intTimer);
}

inline void timerDelayUs(uint32_t delay)
{
	unsigned long counterVal = delay * __FMeg;
	usedTimer.onTimer = false;
	TimerLoadSet(timerList[usedTimer.timerId].timerBase, TIMER_A, counterVal - 1);
	TimerEnable(timerList[usedTimer.timerId].timerBase, TIMER_A);
	//
	TimerIntClear(timerList[usedTimer.timerId].timerBase, TimerIntStatus(timerList[usedTimer.timerId].timerBase, false));
	//
	IntEnable(timerList[usedTimer.timerId].intTimer);
	TimerIntEnable(timerList[usedTimer.timerId].timerBase, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	while(!usedTimer.onTimer);
	TimerIntDisable(timerList[usedTimer.timerId].timerBase, TIMER_TIMA_TIMEOUT);
	IntDisable(timerList[usedTimer.timerId].intTimer);
}


void oneWireInit(oneWireCfg *cfg)
{
	timerInit(cfg->pinTimerId);
	SysCtlPeripheralEnable(cfg->periphGPIO);
	GPIOPadConfigSet(cfg->portGPIO, cfg->pinGPIO, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
	GPIOPinTypeGPIOInput(cfg->portGPIO, cfg->pinGPIO);
}

bool oneWireResetPulse(oneWireCfg *cfg)
{
	uint32_t ok;
	GPIOPinTypeGPIOOutput(cfg->portGPIO, cfg->pinGPIO);
	GPIOPinWrite(cfg->portGPIO, cfg->pinGPIO, 0);
	timerDelayUs(477 + OFFSET);
	GPIOPadConfigSet(cfg->portGPIO, cfg->pinGPIO, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
	GPIOPinTypeGPIOInput(cfg->portGPIO, cfg->pinGPIO);
	timerDelayUs(77 + OFFSET);
	ok = GPIOPinRead(cfg->portGPIO, cfg->pinGPIO);
    timerDelayUs(397 + OFFSET);

	return ok == 0;
}

inline void _oneWireWrite(oneWireCfg *cfg, uint8_t val)
{
	GPIOPinTypeGPIOOutput(cfg->portGPIO, cfg->pinGPIO);
	if(val)
	{
  	  GPIOPinWrite(cfg->portGPIO, cfg->pinGPIO, 0);
	  timerDelayUs(1);
	  GPIOPadConfigSet(cfg->portGPIO, cfg->pinGPIO, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
	  GPIOPinTypeGPIOInput(cfg->portGPIO, cfg->pinGPIO);
	  timerDelayUs(56 + OFFSET);
	}
	else
	{ // Está hecho de este modo (redundante) para asegurarse que ceros y unos duren el mismo tiempo
	   GPIOPinWrite(cfg->portGPIO, cfg->pinGPIO, 0);
	   timerDelayUs(56 + OFFSET);
	   GPIOPadConfigSet(cfg->portGPIO, cfg->pinGPIO, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
	   GPIOPinTypeGPIOInput(cfg->portGPIO, cfg->pinGPIO);
	   timerDelayUs(1);
	}
}

inline uint8_t _oneWireRead(oneWireCfg *cfg)
{
	uint8_t bit = 0;
	GPIOPinTypeGPIOOutput(cfg->portGPIO, cfg->pinGPIO);
	GPIOPinWrite(cfg->portGPIO, cfg->pinGPIO, 0);
	timerDelayUs(1);
	GPIOPadConfigSet(cfg->portGPIO, cfg->pinGPIO, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
	GPIOPinTypeGPIOInput(cfg->portGPIO, cfg->pinGPIO);
	timerDelayUs(5 + OFFSET);
	if(GPIOPinRead(cfg->portGPIO, cfg->pinGPIO) & cfg->pinGPIO)
			bit = 1;
	timerDelayUs(40 + OFFSET);
	return bit;
}

inline void _oneWriteSendByte(oneWireCfg *cfg, unsigned char cmd)
{
   	unsigned char i;
	for(i=8;i>0;i--)
    {
		_oneWireWrite(cfg, cmd & 0x01);
		cmd = cmd >> 1;
    }
}

void oneWireConvertTemp(oneWireCfg *cfg)
{
	_oneWriteSendByte(cfg, CMD_CONVERT_TEMP);
	while(_oneWireRead(cfg) == 0);
}

bool oneWireReadRom(oneWireCfg *cfg, unsigned char code[])
{
	uint8_t i;
	uint8_t bit;
	uint8_t mod;
	uint8_t j;
	uint8_t crc8 = 0;
	for(i=0; i < 8; i++)
		code[i] = 0;

	_oneWriteSendByte(cfg, CMD_READ_ROM);
	for(i=0; i < 64; i++)
	{
		bit = _oneWireRead(cfg);
		oneWireCRC8(&crc8, bit);
		j   = i / 8;
		mod = i % 8;
		code[j] = code[j] | (bit << mod);
	}
	return crc8 == 0;
}

bool oneWireReadMem(oneWireCfg *cfg, unsigned char data[])
{
	uint8_t i;
	uint8_t bit;
	uint8_t mod;
	uint8_t j;
	uint8_t crc8 = 0;
	for(i=0; i < 9; i++)
		data[i] = 0;
	_oneWriteSendByte(cfg, CMD_READ_MEM);
	for(i=0; i < 72; i++)
	{
		bit = _oneWireRead(cfg);
		oneWireCRC8(&crc8, bit);
		j   = i / 8;
		mod = i % 8;
		data[j] = data[j] | (bit << mod);
	}
	return crc8 == 0;
}

inline void oneWireSkipRom(oneWireCfg *cfg)
{
	_oneWriteSendByte(cfg, CMD_SKIP_ROM);
}

void oneWireSelectDev(oneWireCfg *cfg, unsigned char romCode[])
{
	uint8_t i;
	uint8_t bit;
	uint8_t mod;
	uint8_t j;

	_oneWriteSendByte(cfg, CMD_MATCH_ROM);
	for(i=0; i < 64; i++)
	{
		j   = i / 8;
		mod = i % 8;
		bit = romCode[j] & (1 << mod);
		_oneWireWrite(cfg, bit);
	}
}

void oneWireSearchInit(searchRomCfg *romCfg, uint8_t maxDevs)
{
	uint8_t i, j;
	romCfg->cantRoms = 0;
	romCfg->maxRoms = maxDevs;
	for(i=0;i<maxDevs; i++)
	{
	  romCfg->roms[i].ok = 0;
	  for(j=0;j<8;j++)
		  romCfg->roms[i].rom[j] = 0;
	}
}

void oneWireSearchDev(oneWireCfg *cfg, searchRomCfg *romCfg, uint8_t lastDev)
{
	uint8_t i, byteNro, bitNro;
	uint8_t bit1, bit2, bitW;
	uint8_t newRomId;

	if(romCfg->roms[lastDev].ok == 64)
		return;

	oneWireResetPulse(cfg);
	_oneWriteSendByte(cfg, CMD_SEARCH_ROM);

	for(i=0; i < romCfg->roms[lastDev].ok; i++)
	{
		bit1 = _oneWireRead(cfg);
		bit2 = _oneWireRead(cfg);
		byteNro = i / 8;
		bitNro = i % 8;
		bitW = romCfg->roms[lastDev].rom[byteNro] & (1 << bitNro);
		_oneWireWrite(cfg, bitW);
	}

	bit1 = _oneWireRead(cfg);
	bit2 = _oneWireRead(cfg);

    if((bit1 == 1) && (bit2 == 1))
    {
    	return;
    }

    else if((bit1 == 0) && (bit2 == 0))
    {
    	if(romCfg->roms[lastDev].ok == 0)
    	{
		    if(romCfg->cantRoms == romCfg->maxRoms)
			    return;
		    romCfg->cantRoms++;
    	}
		byteNro = romCfg->roms[lastDev].ok / 8;
		bitNro  = romCfg->roms[lastDev].ok % 8;

		romCfg->roms[lastDev].rom[byteNro] = romCfg->roms[lastDev].rom[byteNro] & ~(1 << bitNro);
		romCfg->roms[lastDev].ok++;

		if(romCfg->cantRoms <= romCfg->maxRoms)
		{
			newRomId = romCfg->cantRoms;
			romCfg->cantRoms++;
			for(i = 0; i < 8; i++)
			  romCfg->roms[newRomId].rom[i] = romCfg->roms[lastDev].rom[i];
			romCfg->roms[newRomId].ok = romCfg->roms[lastDev].ok;
			romCfg->roms[newRomId].rom[byteNro] = romCfg->roms[newRomId].rom[byteNro] | (1 << bitNro);
			oneWireSearchDev(cfg, romCfg, lastDev);
			oneWireSearchDev(cfg, romCfg, newRomId);
		}
		else
			return;
    }
    else
    {
    	if(romCfg->roms[lastDev].ok == 0)
    	{
		    if(romCfg->cantRoms == romCfg->maxRoms)
			    return;
		    romCfg->cantRoms++;
    	}
    	while((bit1 != bit2) && (romCfg->roms[lastDev].ok < 64))
    	{
    	  byteNro = romCfg->roms[lastDev].ok / 8;
    	  bitNro = romCfg->roms[lastDev].ok % 8;
    	  romCfg->roms[lastDev].rom[byteNro] = romCfg->roms[lastDev].rom[byteNro] | (bit1 << bitNro);
    	  romCfg->roms[lastDev].ok++;
    	  _oneWireWrite(cfg, bit1);
    	  bit1 = _oneWireRead(cfg);
    	  bit2 = _oneWireRead(cfg);
    	}
    	if(romCfg->roms[lastDev].ok < 64)
   	      oneWireSearchDev(cfg, romCfg, lastDev);
    }

}


inline void oneWireCRC8 (unsigned char *shift_reg, unsigned char data_bit)
{
   unsigned char fb;
   fb = (*shift_reg & 0x01) ^ data_bit;
   *shift_reg = *shift_reg >> 1;
   if (fb == 1)
   {
      (*shift_reg) = (*shift_reg) ^ 0x8C;
   }
}

bool oneWireValidatePkg(unsigned char romCode[], uint8_t len)
{
	uint8_t i, j;
	uint8_t bit;
	uint8_t crc = 0;
	for(i = 0; i < len; i++)
	{
		for(j = 0; j < 8; j++)
		{
			bit = (romCode[i] >> j) & 1;
		    oneWireCRC8(&crc, bit);
		}
	}
	return crc == 0;
}

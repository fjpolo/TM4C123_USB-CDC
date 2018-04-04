/****************************************************************************
 * Copyright (C) 2016 by Sergio Burgos                                      *
 *                                                                          *
 * This file is part of 1 Wire Library for TIVA C.                          *
 *                                                                          *
 *   This library is free software: you can redistribute it and/or modify it*
 *   under the terms of the GNU Lesser General Public License as published  *
 *   by the Free Software Foundation, either version 3 of the License, or   *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This library is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU Lesser General Public License for more details.                    *
 *                                                                          *
 *   You should have received a copy of the GNU Lesser General Public       *
 *   License along with Box.  If not, see <http://www.gnu.org/licenses/>.   *
 ****************************************************************************/

/**
 * @file oneWire.h
 * @author Serigo Burgos
 * @date 18/08/2016
 * @brief Este archivo contiene las declaraciones utilizadas por la librería
 *
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>

/*! \def MAXDEVS
    \brief Esta constante simbólica define la cantidad máxima de nodos a auto descubrir.

    El algoritmo de auto detección de nodos requiere de cierta información alojada de modo estático.
    Esta información está relacionada en parte con un vector de estructuras y esta constante establece
    la cantidad total de estructuras asociadas a los nodos que se alojarán. Desde un punto de vista de conveniencia
    esta constante debería tener un valor grande, sin embargo, la cantidad de memoria que utiliza puede resultar
    significativa en muchas arquitecturas.

*/
#define MAXDEVS 6

#define OFFSET (-3)

#define CMD_SEARCH_ROM   0xF0
#define CMD_READ_ROM     0x33
#define CMD_MATCH_ROM    0x55		//Match ROM
#define CMD_SKIP_ROM     0xCC		//Skip ROM
#define CMD_ALARM_SEARCH 0xEC
#define CMD_CONVERT_TEMP 0x44		//Convert Temperature
#define CMD_WRITE_MEM    0x4E
#define CMD_READ_MEM     0xBE		//Read scratchpad
#define CMD_COPY_MEM     0x48
#define CMD_RECALL_EE    0xB8
#define CMD_READ_PWR     0xB4

/*! \struct timerHW
 *
 * \brief Estructura utilizada para inicializar el timer deseado
 *
 * Esta librería depende del uso de timers para fijar bases de tiempo.
 * Para simplificar su uso, la configuración del timer a utilizar, se
 * realiza indicando solo el número de timer a usar. Luego, una función
 * utiliza este valor como índice de una estructura inicializada en flash
 * con los valores de los registros según el número de timer indicado.
 * Esta estructura no es utilizada desde la aplicación, solo por la librería
 * durante el proceso de configuración.
 */
typedef struct
{
	uint32_t timerPeriph;
	uint32_t timerBase;
	uint32_t intTimer;
} timerHW;

/*! \struct timerCfg
 *
 * \brief Contiene configuración asociada al timer utilizado
 *
 * Las bases de tiempo se determinan utilizando interrupciones. Para esto,
 * se utiliza una bandera que es actualizada cuando se dispara la interrupción.
 * Esta bandera es el campo onTimer. El otro campo, timerId es utilizado
 * para identificar el timer usado, de este modo es posible blanquear la
 * bandera de interrupción.
 */
typedef struct
{
	uint8_t timerId;
	bool onTimer;
} timerCfg;

typedef struct
{
	uint32_t periphGPIO;
	uint32_t portGPIO;
	uint8_t pinGPIO;
	uint8_t pinTimerId;
} oneWireCfg;

typedef struct
{
  uint8_t rom[8];
  uint8_t ok;
} oneRom;

typedef struct
{
	uint8_t maxRoms;
	uint8_t cantRoms;
	oneRom roms[MAXDEVS];
} searchRomCfg;


void timerInit(uint8_t timerId);
void timerDelayUs(uint32_t delay);
void timerDelayMs(uint32_t delay);

void _oneWireWrite(oneWireCfg *cfg, uint8_t val);
uint8_t _oneWireRead(oneWireCfg *cfg);
void _oneWriteSendByte(oneWireCfg *cfg, unsigned char cmd);

void oneWireInit(oneWireCfg *cfg);
bool oneWireResetPulse(oneWireCfg *cfg);
bool oneWireReadRom(oneWireCfg *cfg, unsigned char code[]);
void oneWireConvertTemp(oneWireCfg *cfg);
bool oneWireReadMem(oneWireCfg *cfg, unsigned char data[]);
void oneWireSkipRom(oneWireCfg *cfg);
void oneWireSelectDev(oneWireCfg *cfg, unsigned char romCode[]);
void oneWireSearchInit(searchRomCfg *romCfg, uint8_t maxDevs);
void oneWireSearchDev(oneWireCfg *cfg, searchRomCfg *romCfg, uint8_t lastDev);
void oneWireCRC8 (unsigned char *shift_reg, unsigned char data_bit);
bool oneWireValidatePkg(unsigned char romCode[], uint8_t len);

#endif /* ONEWIRE_H_ */


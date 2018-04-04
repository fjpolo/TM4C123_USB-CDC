#ifndef leds_h
#define leds_h

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

#define ROJO  GPIO_PIN_1
#define AZUL  GPIO_PIN_2
#define VERDE GPIO_PIN_3

void leds_init(void);
void led_on(unsigned int);
void led_off(unsigned int);
#endif

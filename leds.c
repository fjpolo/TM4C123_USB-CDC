#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "leds.h"

void leds_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1);
}

void led_on(unsigned int ledId)
{
	GPIOPinWrite(GPIO_PORTF_BASE, ledId, ledId);
}
void led_off(unsigned int ledId)
{
	GPIOPinWrite(GPIO_PORTF_BASE, ledId, ~ledId);
}

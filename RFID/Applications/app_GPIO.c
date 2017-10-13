/*
 * app_GPIO.c
 *
 *  Created on: 10/09/2017
 *      Author: OmarSevilla
 */

#include "stdint.h"
#include "stdtypedef.h"
#include "MKL25Z4.h"
#include "fsl_clock.h"
#include "fsl_port.h"
#include "app_GPIO.h"

void app_GPIO_Init(void)
{

	port_pin_config_t ls_PINCfgTemp;
	gpio_pin_config_t ls_GPIOPINCfg;

	ls_PINCfgTemp.mux = kPORT_MuxAsGpio;

	/* PORTB*/
	/*Enable Clock*/
	CLOCK_EnableClock(kCLOCK_PortB); //LED RED & GREEN Port

	/*Set PIN MUX as GPIO*/
	PORT_SetPinConfig(LED_RED_PORT_BASE, LED_RED_PIN_NUMBER, &ls_PINCfgTemp);//LED RED PORT CONFIG
	PORT_SetPinConfig(LED_GREEN_PORT_BASE, LED_GREEN_PIN_NUMBER, &ls_PINCfgTemp);//LED GREEN PORT CONFIG

	/* Init port direction and logical value */
	ls_GPIOPINCfg.pinDirection = kGPIO_DigitalOutput; //PIN as output
	ls_GPIOPINCfg.outputLogic = TRUE;

	GPIO_PinInit(LED_RED_GPIO_BASE, LED_RED_PIN_NUMBER, &ls_GPIOPINCfg); //LED GREEN Initialization
	GPIO_PinInit(LED_GREEN_GPIO_BASE, LED_GREEN_PIN_NUMBER, &ls_GPIOPINCfg); //LED GREEN Initialization

	/* PORTD */
	/*Enable Clock*/
	CLOCK_EnableClock(kCLOCK_PortD); //LED BLUE Port

	/*Set PIN MUX as GPIO*/
	PORT_SetPinConfig(LED_BLUE_PORT_BASE, LED_BLUE_PIN_NUMBER, &ls_PINCfgTemp);//LED BLUE PORT CONFIG

	/* Init port direction and logical value */
	ls_GPIOPINCfg.pinDirection = kGPIO_DigitalOutput; //PIN as output
	ls_GPIOPINCfg.outputLogic = TRUE;

	GPIO_PinInit(LED_BLUE_GPIO_BASE, LED_BLUE_PIN_NUMBER, &ls_GPIOPINCfg); //LED GREEN Initialization

	return;
}

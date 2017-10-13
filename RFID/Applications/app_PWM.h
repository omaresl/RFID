/*
 * app_PWM.h
 *
 *  Created on: 27/09/2017
 *      Author: uidj2522
 */

#ifndef APP_PWM_H_
#define APP_PWM_H_

#include "stdtypedef.h"

#define APP_PWM_PIN_NUMBER_LED_GREEN	19U
#define APP_PWM_PORT_LED_GREEN			PORTB
#define APP_PWM_CHANNEL					1U
#define APP_PWM_DUTYCYCLE_INITVALUE		rub_DutyCycle
#define APP_PWM_BASE_ADDRESS			TPM2

/* Get source clock for TPM driver */
#define APP_PWM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_PllFllSelClk)

#define APP_PWM_MIN_DUTY_CYCLE	0U
#define APP_PWM_MAX_DUTY_CYCLE	20U

/***************************************
 * Prototypes						   *
 ***************************************/
extern void app_PWM_Init(void);
extern void app_PWM_Update_DutyCycle(T_UBYTE lub_DutyCycle);
extern void app_PWM_Mngr(void);
extern void app_PWM_Task(void);

#endif /* APP_PWM_H_ */

/*
 * app_PWM.c
 *
 *  Created on: 27/09/2017
 *      Author: uidj2522
 */

/***************************************
 * Interfaces						   *
 ***************************************/
#include "stdtypedef.h"
#include "app_PWM.h"
#include "fsl_tpm.h"
#include "fsl_clock.h"
#include "fsl_port.h"

/***************************************
 * Types							   *
 ***************************************/
typedef enum
{
	PWM_COUNT_UP,
	PWM_COUNT_DOWN
}E_PWM_COUNT_DIRECTION;

/***************************************
 * Variables						   *
 ***************************************/
static T_UBYTE rub_DutyCycle;
E_PWM_COUNT_DIRECTION re_PWMCountDirection;

/***************************************
 * Prototypes						   *
 ***************************************/

/***************************************
 * Code								   *
 ***************************************/

/**********************************************************
 * Name: app_PWM_Init
 * Description: This function initializes the PWM module
 **********************************************************/
void app_PWM_Init(void)
{
	rub_DutyCycle = 0U;

	/* Prepare config structures */
	tpm_config_t tpmInfo;
	tpm_chnl_pwm_signal_param_t tpmParam;

	/* Configure tpm params with frequency 24kHZ */
	tpmParam.chnlNumber = (tpm_chnl_t)APP_PWM_CHANNEL;
	tpmParam.level = kTPM_LowTrue;
	tpmParam.dutyCyclePercent = APP_PWM_DUTYCYCLE_INITVALUE;

	/* Init Pins */
	CLOCK_EnableClock(kCLOCK_PortB);                           /* Port B Clock Gate Control: Clock enabled */
	PORT_SetPinMux(APP_PWM_PORT_LED_GREEN, APP_PWM_PIN_NUMBER_LED_GREEN, kPORT_MuxAlt3);           /* PORTB19 (pin 54) is configured as TPM2_CH1 */

	/* Select the clock source for the TPM counter as kCLOCK_PllFllSelClk */
	CLOCK_SetTpmClock(1U);

	TPM_GetDefaultConfig(&tpmInfo);
	/* Initialize TPM module */
	TPM_Init(APP_PWM_BASE_ADDRESS, &tpmInfo);

	TPM_SetupPwm(APP_PWM_BASE_ADDRESS, &tpmParam, 1U, kTPM_CenterAlignedPwm, 24000U, APP_PWM_SOURCE_CLOCK);

	TPM_StartTimer(APP_PWM_BASE_ADDRESS, kTPM_SystemClock);
}

/**********************************************************
 * Name: app_PWM_Update_DutyCycle
 * Description: This function update the DutyCycle of the PWM
 **********************************************************/
void app_PWM_Update_DutyCycle(T_UBYTE lub_DutyCycle)
{
	/* Disable channel output before updating the dutycycle */
	TPM_UpdateChnlEdgeLevelSelect(APP_PWM_BASE_ADDRESS, (tpm_chnl_t)APP_PWM_CHANNEL, 0U);

	/* Update PWM duty cycle */
	TPM_UpdatePwmDutycycle(APP_PWM_BASE_ADDRESS, (tpm_chnl_t)APP_PWM_CHANNEL, kTPM_CenterAlignedPwm,
			lub_DutyCycle);

	/* Start channel output with updated dutycycle */
	TPM_UpdateChnlEdgeLevelSelect(APP_PWM_BASE_ADDRESS, (tpm_chnl_t)APP_PWM_CHANNEL, kTPM_LowTrue);
}


/**********************************************************
 * Name: app_PWM_Task
 * Description: This function manages the task for the PWM
 **********************************************************/
void app_PWM_Task(void)
{
	app_PWM_Update_DutyCycle(rub_DutyCycle);

	if(re_PWMCountDirection == PWM_COUNT_UP)
	{
		if(rub_DutyCycle < APP_PWM_MAX_DUTY_CYCLE)
		{
			rub_DutyCycle++;
		}
		else
		{
			re_PWMCountDirection = PWM_COUNT_DOWN;
		}
	}
	else
	{
		if(rub_DutyCycle > APP_PWM_MIN_DUTY_CYCLE)
		{
			rub_DutyCycle--;
		}
		else
		{
			re_PWMCountDirection = PWM_COUNT_UP;
		}
	}
}

/*
 * app_PIT.c
 *
 *  Created on: 10/09/2017
 *      Author: OmarSevilla
 */

#include "stdtypedef.h"
#include "fsl_clock.h"
#include "app_PIT.h"
#include "fsl_pit.h"
#include "OS_SEBELL_sched.h"
#include "app_RC522.h"

/****************************************
 * Name:
 * Description:
 * Parameters:
 * **************************************/
void app_PIT_Init(void)
{
	/* Structure of initialize PIT */
	pit_config_t pitConfig;

	/*
	 * pitConfig.enableRunInDebug = false;
	 */
	PIT_GetDefaultConfig(&pitConfig);

	/* Init pit module */
	PIT_Init(PIT, &pitConfig);

	/* Set timer period for channel 0 */
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(2500U, PIT_SOURCE_CLOCK));

	/* Enable timer interrupts for channel 0 */
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

	/* Enable at the NVIC */
	EnableIRQ(PIT_IRQ_ID);

	/* Start channel 0 */
	PIT_StartTimer(PIT, kPIT_Chnl_0);

	return;

}

void PIT_INTERRUPT_HANDLER(void)
{
	/* Clear interrupt flag.*/
	PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
	sch_alarm = TRUE;
	if(APP_RC522_TIMER_IS_STOPPED(rub_RC522WatchDog) == TRUE)
	{
		/* Do nothing */
	}
	else
	{
		rub_RC522WatchDog--;
	}
}

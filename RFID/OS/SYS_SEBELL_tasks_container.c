/*
 * SYS_SEBELL_tasks_container.c
 *
 *  Created on: 10/08/2015
 *      Author: uidj2522
 */

#include "stdtypedef.h"
#include "SYS_SEBELL_task_container.h"
#include "Applications/app_PWM.h"
#include "Applications/app_RC522.h"

/*Task Executed as fast that the module allows it*/
void TASK_AllTime(void)
{

}

/*Task Executed every 5 ms*/
void TASK_5ms(void)
{
	app_PWM_Task();
}

/*Task Executed every 10 ms*/
void TASK_10ms(void)
{

}

/*Task Executed every 80 ms*/
void TASK_80ms(void)
{
	app_RC522_TaskMng();
}

/*Task Executed every 1280 ms*/
void TASK_1280ms(void)
{
	//app_LEDMngr_ToogleColor(re_CurrentLedColor);
}

void TASK_DUMMY(void)
{

}

/*
 * OS_SEBELL_schedule.c
 *
 *  Created on: 10/08/2015
 *      Author: uidj2522
 */


#include "OS_SEBELL_sched.h"
#include "OS_SEBELL_sched_util.h"
#include "OS_SEBELL_sched_cfg.h"
#include "SYS_SEBELL_task_container.h"

unsigned char sch_alarm;
T_SCH_CNT scheduler_counter;
unsigned char Execute_task;

const FCT_POINTER tasks_list[MAX_TASKS] =
{
		&TASK_5ms,
		&TASK_10ms,
		&TASK_DUMMY,
		&TASK_DUMMY,
		&TASK_80ms,
		&TASK_DUMMY,//160
		&TASK_DUMMY,//320
		&TASK_DUMMY,//640
		&TASK_1280ms,//1280
		&TASK_DUMMY,
		&TASK_DUMMY,
		&TASK_DUMMY,
		&TASK_DUMMY,
		&TASK_DUMMY,
		&TASK_DUMMY,
		&TASK_DUMMY
};

/************************************************************
 * Name: scheduler_loop
 * Description: Loop that will search and execute the task
 * Parameters: Void
 * Return: Void
 *************************************************************/

void scheduler_loop(void){
	/*Start the loop*/
	for(;;){
		TASK_AllTime();
		if( ALARM_ACTIVE == sch_alarm){
			/*Time to work! , increment the counter and turn off the alarm.*/
			sch_alarm = ALARM_INACTIVE;
			scheduler_counter++;
			Execute_task = EXECUTE;
			/*See if we already over passed the value!
			  Note: Checking after count increment is safe since the max value is 254
			 */
			if(SCH_MAX_COUNT < scheduler_counter){
				scheduler_counter = SCH_MIN_COUNT; //Clear counter
			}//End of check counter
			else{
				/*Do nothing*/
			}
		}//End of Alarm Active check
		else{
			if(EXECUTE == Execute_task  ){
				/*Time to execute*/
				Execute_task = IDLE; //Now rest for a while
				tasks_list[sched_util_task_finder(scheduler_counter)](); //Execute task
			}
		}//End of executing

	}//End of infinite loop
}//End of scheduler_loop function





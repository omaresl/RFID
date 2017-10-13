/*
 * app_PIT.h
 *
 *  Created on: 10/09/2017
 *      Author: OmarSevilla
 */

#ifndef APP_PIT_H_
#define APP_PIT_H_

/* Interrupt Handler */
#define PIT_INTERRUPT_HANDLER	PIT_IRQHandler
#define PIT_IRQ_ID 				PIT_IRQn
/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

extern void app_PIT_Init(void);

#endif /* APP_PIT_H_ */

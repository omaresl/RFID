/*
 * Copyright (c) 2016, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    Desencriptador.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "Applications/app_RC522.h"

/* TODO: insert other include files here. */
#include "Applications/app_Init.h"
#include "OS_SEBELL_sched.h"

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
int main(void)
{
	/* Init board hardware. */
#ifdef APP_RC522_UART_CHANNEL
	BOARD_InitBootPins();
#endif
	BOARD_BootClockRUN();
#ifdef APP_RC522_UART_CHANNEL
	//Debug Console
	BOARD_InitDebugConsole();
#endif

	/*User Initialization*/
	app_Init_Manager();

	/*Scheduler*/
	scheduler_loop();

	return 0 ;
}

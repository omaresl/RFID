/*
 * app_LEDMngr.h
 *
 *  Created on: 12/09/2017
 *      Author: OmarSevilla
 */

#ifndef APP_LEDMNGR_H_
#define APP_LEDMNGR_H_
#include "stdtypedef.h"

typedef enum
{
	BLACK_LEDCOLOR,
	RED_LEDCOLOR,
	GREEN_LEDCOLOR,
	BLUE_LEDCOLOR,
	YELLOW_LEDCOLOR,
	CYAN_LEDCOLOR,
	PURPLE_LEDCOLOR,
	WHITE_LEDCOLOR,
	N_LEDCOLORS
}E_LEDCOLOR;

extern E_LEDCOLOR re_CurrentLedColor;

extern void app_LEDMngr_ToogleColor(E_LEDCOLOR le_LedColor);
extern void app_LEDMngr_ColorSelect(T_UBYTE lub_char);

#endif /* APP_LEDMNGR_H_ */

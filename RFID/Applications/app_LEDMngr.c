/*
 * app_LEDMngr.c
 *
 *  Created on: 12/09/2017
 *      Author: OmarSevilla
 */

#include "stdtypedef.h"
#include "app_GPIO.h"
#include "app_LEDMngr.h"

E_LEDCOLOR re_CurrentLedColor = BLACK_LEDCOLOR;

void app_LEDMngr_ToogleColor(E_LEDCOLOR le_LedColor)
{
	switch(le_LedColor)
	{
	case BLACK_LEDCOLOR:
	default:
	{
		LED_RED_OFF();
		LED_GREEN_OFF();
		LED_BLUE_OFF();
	}break;
	case RED_LEDCOLOR:
	{
		LED_BLUE_OFF();
		LED_GREEN_OFF();
		LED_RED_TOGGLE();
	}break;
	case GREEN_LEDCOLOR:
	{
		LED_BLUE_OFF();
		LED_RED_OFF();
		LED_GREEN_TOGGLE();
	}break;
	case BLUE_LEDCOLOR:
	{
		LED_RED_OFF();
		LED_GREEN_OFF();
		LED_BLUE_TOGGLE();
	}break;

	case YELLOW_LEDCOLOR:
	{
		LED_BLUE_OFF();
		LED_YELLOW_TOGGLE();
	}break;
	case CYAN_LEDCOLOR:
	{
		LED_RED_OFF();
		LED_CYAN_TOGGLE();
	}break;
	case PURPLE_LEDCOLOR:
	{
		LED_GREEN_OFF();
		LED_PURPLE_TOGGLE();
	}break;
	case WHITE_LEDCOLOR:
	{
		LED_WHITE_TOGGLE();
	}break;
	}
}

void app_LEDMngr_ColorSelect(T_UBYTE lub_char)
{
	switch(lub_char)
	{
	case 'r':
	case 'R':
	{
		re_CurrentLedColor = RED_LEDCOLOR;
	}break;
	case 'g':
	case 'G':
	{
		re_CurrentLedColor = GREEN_LEDCOLOR;
	}break;
	case 'b':
	case 'B':
	{
		re_CurrentLedColor = BLUE_LEDCOLOR;
	}break;
	case 'y':
	case 'Y':
	{
		re_CurrentLedColor = YELLOW_LEDCOLOR;
	}break;
	case 'n':
	case 'N':
	{
		re_CurrentLedColor = BLACK_LEDCOLOR;
	}break;
	case 'C':
	case 'c':
	{
		re_CurrentLedColor = CYAN_LEDCOLOR;
	}break;
	case 'p':
	case 'P':
	{
		re_CurrentLedColor = PURPLE_LEDCOLOR;
	}break;
	case 'w':
	case 'W':
	{
		re_CurrentLedColor = WHITE_LEDCOLOR;
	}break;
	default:
	{
		//Color does not change
	}break;
	}
}

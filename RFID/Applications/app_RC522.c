/*
 * app_RC522.c
 *
 *  Created on: 28/09/2017
 *      Author: uidj2522
 */

#include "stdio.h"
#include "fsl_common.h"
#include "MKL25Z4.h"
#include "stdtypedef.h"
#include "app_RC522.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_clock.h"
#include "fsl_uart.h"
#include "app_GPIO.h"

#define SOPT5_UART0RXSRC_UART_RX      0x00u   /*!< UART0 receive data source select: UART0_RX pin */
#define SOPT5_UART0TXSRC_UART_TX      0x00u   /*!< UART0 transmit data source select: UART0_TX pin */
#define SOPT5_UART1RXSRC_UART_RX      0x00u   /*!< UART0 receive data source select: UART0_RX pin */
#define SOPT5_UART1TXSRC_UART_TX      0x00u   /*!< UART0 transmit data source select: UART0_TX pin */

/***************************************
 * Variables						   *
 ***************************************/
T_UBYTE raub_RC522_FIFOData[APP_RC522_BUFFER_MAX_LENGTH];
T_UWORD ruw_RC522_FIFOReceivedLength;
T_UBYTE rub_RC522WatchDog;
static E_RC522_STATES re_RC522State = RC522_STATE_INIT;
static E_RC522_STATES re_RC522_NextState = RC522_STATE_CARD_SEARCH;

/***************************************
 * Prototypes						   *
 ***************************************/
static void app_RC522_AntennaOn(void);
static void app_RC522_SetRegisterBitMask(T_UBYTE lub_Address,T_UBYTE lub_mask);
static void app_RC522_ClearRegisterBitMask(T_UBYTE lub_Address,T_UBYTE lub_mask);
static T_UBYTE app_RC522_RequestA(void);
static T_UBYTE app_RC522_ToCard(T_UBYTE lub_command, T_UBYTE *lpub_sendData, T_UBYTE lub_sendLen, T_UBYTE *lpub_backData, T_UWORD *lpuw_backLen);
static T_UBYTE app_RC522_ReadBlock(T_UBYTE lub_BlockAddr, T_UBYTE *lpub_recvData, T_UWORD *luw_unLen);
static void app_RC522_CalculateCRC(T_UBYTE *lpub_Indata, T_UBYTE lub_len, T_UBYTE *lpub_OutData);
static T_UBYTE app_RC522_Anticoll(T_UBYTE *lpub_serNum);


/***************************************
 * Code								   *
 ***************************************/

/**********************************************************
 * Name: app_RC522_TaskMng
 * Description: This function manage the RC522 task
 **********************************************************/
void app_RC522_TaskMng(void)
{
	switch(re_RC522State)
	{
	case RC522_STATE_INIT:
	{
		app_RC522_Init();
		re_RC522State = re_RC522_NextState;
	}break;
	case RC522_STATE_CARD_SEARCH:
	{
		if(app_RC522_IsANewCardPresent() == STATUS_OK)
		{
			LED_RED_ON();

			printf("\n****Card Present****\n ");

			/* Go to next state after INIT */
			re_RC522_NextState = RC522_STATE_GET_ID_CARD;
		}
		else
		{
			LED_RED_OFF();

			/* Go to next state after INIT */
			re_RC522_NextState = RC522_STATE_CARD_SEARCH;
		}

		/* Reset the Transceiver */
		re_RC522State = RC522_STATE_GET_ID_CARD;//RC522_STATE_INIT;


	}break;

	case RC522_STATE_GET_ID_CARD:
	{
		if(app_RC522_Anticoll(raub_RC522_FIFOData) == STATUS_OK)
		{
			printf("ID:");
			for(T_UBYTE i = 0; i < ruw_RC522_FIFOReceivedLength; i++)
			{
				if(raub_RC522_FIFOData[i] < 0x10)
				{
					printf(" 0");
					printf("%X",raub_RC522_FIFOData[i]);
				}
				else
				{
					printf(" %X",raub_RC522_FIFOData[i]);
				}
			}
			printf("\n");

			/* Go to next state after INIT */
			re_RC522_NextState = RC522_STATE_READ_PAGE1;
		}
		else
		{
			/* Go to next state after INIT */
			re_RC522_NextState = RC522_STATE_CARD_SEARCH;
		}
		/* Reset the Transceiver */
		re_RC522State = RC522_STATE_INIT;
	}break;
	case RC522_STATE_READ_PAGE1:
	{
		if(app_RC522_ReadBlock(1, raub_RC522_FIFOData, &ruw_RC522_FIFOReceivedLength) == STATUS_OK)
		{

			printf("\nDATA PAGE 1:\n ");
			for(T_UBYTE i = 0; i < ruw_RC522_FIFOReceivedLength; i++)
			{
				printf("%X ",raub_RC522_FIFOData[i]);
			}
			printf("\n");
		}
		else
		{
			printf("\nError Reading Page 1\n");
		}

		/* Reset the Transceiver */
		re_RC522State = RC522_STATE_INIT;
		/* Go to next state after INIT */
		re_RC522_NextState = RC522_STATE_CARD_SEARCH;

	}break;

	default:
	{
		re_RC522State = RC522_STATE_INIT;
		re_RC522_NextState = RC522_STATE_CARD_SEARCH;
	}break;
	}
}

/**********************************************************
 * Name: app_RC522_Init
 * Description: This function initializes the RC_522 module
 **********************************************************/
void app_RC522_Init(void)
{
	/* Set GPIO configuration for TC522 assigned pines */

	//Structure used for save configuration
	static gpio_pin_config_t 	ls_PinConfig;
	//Pointer to GPIO Structure
	gpio_pin_config_t* 	lps_PinConfig;
	//Pointing to the structure
	lps_PinConfig = &ls_PinConfig;

	static port_pin_config_t  	ls_PortConfig;

	port_pin_config_t* 	lps_PortConfig;

	lps_PortConfig = (port_pin_config_t*)&ls_PortConfig;

	ls_PortConfig.mux = kPORT_MuxAsGpio;

	//Config RST PIN
	ls_PinConfig.pinDirection = kGPIO_DigitalOutput;
	ls_PinConfig.outputLogic = APP_RC_522_RESET_ENABLE; //Init in RST mode disabled

	CLOCK_EnableClock(APP_RC_522_RESET_CLOCK_PORT);
	GPIO_PinInit(APP_RC_522_RESET_GPIO_BASE_ADDR, APP_RC_522_RESET_PIN_NUM, lps_PinConfig);
	PORT_SetPinConfig(APP_RC_522_RESET_PORT_BASE_ADDR, APP_RC_522_RESET_PIN_NUM, lps_PortConfig);

	//Config MX PIN
	ls_PinConfig.pinDirection = kGPIO_DigitalInput;
	ls_PinConfig.outputLogic = FALSE;

	CLOCK_EnableClock(APP_RC_522_MX_CLOCK_PORT);
	GPIO_PinInit(APP_RC_522_MX_GPIO_BASE_ADDR, APP_RC_522_MX_PIN_NUM, lps_PinConfig);
	PORT_SetPinConfig(APP_RC_522_MX_PORT_BASE_ADDR, APP_RC_522_MX_PIN_NUM, lps_PortConfig);

	//Config DTRQ PIN
	ls_PinConfig.pinDirection = kGPIO_DigitalInput;
	ls_PinConfig.outputLogic = FALSE;

	GPIO_PinInit(APP_RC_522_DTRQ_GPIO_BASE_ADDR, APP_RC_522_DTRQ_PIN_NUM, lps_PinConfig);
	PORT_SetPinConfig(APP_RC_522_DTRQ_PORT_BASE_ADDR, APP_RC_522_DTRQ_PIN_NUM, lps_PortConfig);

#ifndef APP_RC522_UART_CHANNEL
	//Config RX
	//PORTA Already Enabled
	ls_PortConfig.mux = kPORT_MuxAlt2;
	PORT_SetPinConfig(APP_RC_522_RX_PORT_BASE_ADDR, APP_RC_522_RX_PIN_NUM, lps_PortConfig);

	//Config TX
	ls_PortConfig.mux = kPORT_MuxAlt2;
	PORT_SetPinConfig(APP_RC_522_TX_PORT_BASE_ADDR, APP_RC_522_TX_PIN_NUM, lps_PortConfig);

	/* UART Configuration */

	SIM->SOPT5 = ((SIM->SOPT5 &
			(~(SIM_SOPT5_UART0TXSRC_MASK | SIM_SOPT5_UART0RXSRC_MASK))) /* Mask bits to zero which are setting */
			| SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX)       /* UART0 transmit data source select: UART0_TX pin */
			| SIM_SOPT5_UART0RXSRC(SOPT5_UART0RXSRC_UART_RX)       /* UART0 receive data source select: UART0_RX pin */
	);

	CLOCK_SetLpsci0Clock(1);

	lpsci_config_t ls_UartConfig;
	lpsci_config_t *lps_UartConfig;

	lps_UartConfig = &ls_UartConfig;

	//Get default config
	LPSCI_GetDefaultConfig(lps_UartConfig);

	ls_UartConfig.baudRate_Bps = 9600U;
	ls_UartConfig.enableTx = TRUE;
	ls_UartConfig.enableRx = TRUE;

	//Init UART0
	LPSCI_Init(UART0, lps_UartConfig, CLOCK_GetPllFllSelClkFreq());

	//Enable Interrupts
	LPSCI_EnableInterrupts(UART0, kLPSCI_RxDataRegFullInterruptEnable | kLPSCI_RxOverrunInterruptEnable);
	EnableIRQ(UART0_IRQn);

	//Disable Reset
	GPIO_SetPinsOutput(APP_RC_522_RESET_GPIO_BASE_ADDR, 1 << APP_RC_522_RESET_PIN_NUM);

	T_UBYTE lub_Result = 0U;

	//Set baud rate to 115200
	lub_Result |= app_RC522_WriteRegister(SerialSpeedReg, 0x7AU);

	LPSCI_SetBaudRate(UART0, 115200, CLOCK_GetPllFllSelClkFreq());

#else
	//Config RX
	//PORTD Already Enabled
	ls_PortConfig.mux = kPORT_MuxAlt3;
	PORT_SetPinConfig(APP_RC_522_RX_PORT_BASE_ADDR, APP_RC_522_RX_PIN_NUM, lps_PortConfig);

	//Config TX
	ls_PortConfig.mux = kPORT_MuxAlt3;
	PORT_SetPinConfig(APP_RC_522_TX_PORT_BASE_ADDR, APP_RC_522_TX_PIN_NUM, lps_PortConfig);

	/* UART Configuration */

	SIM->SOPT5 = ((SIM->SOPT5 &
			(~(SIM_SOPT5_UART1TXSRC_MASK | SIM_SOPT5_UART1RXSRC_MASK))) /* Mask bits to zero which are setting */
			| SIM_SOPT5_UART1TXSRC(SOPT5_UART1TXSRC_UART_TX)       /* UART1 transmit data source select: UART1_TX pin */
			| SIM_SOPT5_UART1RXSRC(SOPT5_UART1RXSRC_UART_RX)       /* UART1 receive data source select: UART1_RX pin */
	);

	CLOCK_EnableClock(kCLOCK_Uart2);

	uart_config_t ls_UartConfig;
	uart_config_t *lps_UartConfig;

	lps_UartConfig = &ls_UartConfig;

	//Get default config
	UART_GetDefaultConfig(lps_UartConfig);

	ls_UartConfig.baudRate_Bps = 9600U;
	ls_UartConfig.enableTx = TRUE;
	ls_UartConfig.enableRx = TRUE;

	//Init UART0
	UART_Init(APP_RC522_UART_CHANNEL, lps_UartConfig, CLOCK_GetFreq(BUS_CLK));

	//Enable Interrupts
	UART_EnableInterrupts(APP_RC522_UART_CHANNEL, kLPSCI_RxDataRegFullInterruptEnable | kLPSCI_RxOverrunInterruptEnable);
	EnableIRQ(UART2_IRQn);

	//Disable Reset
	GPIO_SetPinsOutput(APP_RC_522_RESET_GPIO_BASE_ADDR, 1 << APP_RC_522_RESET_PIN_NUM);

	T_UBYTE lub_Result = 0U;

	//Set baud rate to 115200
	lub_Result |= app_RC522_WriteRegister(SerialSpeedReg, 0x7AU);

	UART_SetBaudRate(APP_RC522_UART_CHANNEL, 115200, CLOCK_GetFreq(BUS_CLK));
#endif
	// Reset ModWidthReg
	lub_Result |= app_RC522_WriteRegister(ModWidthReg, 0x26);
	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	lub_Result |= app_RC522_WriteRegister(TModeReg, 0x8D);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	lub_Result |= app_RC522_WriteRegister(TPrescalerReg, 0x3E);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	lub_Result |= app_RC522_WriteRegister(TReloadRegH, 0x030);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	lub_Result |= app_RC522_WriteRegister(TReloadRegL, 0x00);

	lub_Result |= app_RC522_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	lub_Result |= app_RC522_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	//lub_Result |= app_RC522_WriteRegister(RFCfgReg, 0x00);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	app_RC522_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
}


/**********************************************************
 * Name: app_RC522_RequestReadRegister
 * Description: This function read a register from RC522 chip and
 * return the value
 **********************************************************/
T_UBYTE app_RC522_ReadRegister(T_UBYTE lub_Address)
{
	T_UBYTE lub_DataToSend;
	T_UBYTE lub_RXData;;

	//Adjust data
	lub_DataToSend = ((lub_Address) | 0x80);

	APP_RC522_COMM_INTERFACE_SEND(lub_DataToSend);

	lub_RXData = 0U;

	/*Start TImeout Timer*/
	APP_RC522_TIMER_LOAD(rub_RC522WatchDog);
	while(lub_RXData == 0U && APP_RC522_TIMER_IS_STOPPED(rub_RC522WatchDog) == FALSE)
	{
		lub_RXData = app_UART_GetBytesReceived();
	}

	return APP_RC522_COMM_INTERFACE_RECEIVE();
}

/**********************************************************
 * Name: app_RC522_RequestReadRegister
 * Description: This function writes a register from RC522 chip and
 * return the value
 **********************************************************/
T_UBYTE app_RC522_WriteRegister(T_UBYTE lub_Address, T_UBYTE lub_Value)
{
	T_UBYTE lub_DataToSend;
	T_UBYTE lub_Return;
	T_UBYTE lub_TempMX;

	lub_Return = TRUE;

	//Adjust data
	lub_DataToSend = (lub_Address);

	//Start written
	APP_RC522_COMM_INTERFACE_SEND(lub_DataToSend);

	/*Start TImeout Timer*/
	APP_RC522_TIMER_LOAD(rub_RC522WatchDog);
	while(app_UART_GetBytesReceived() == 0U && APP_RC522_TIMER_IS_STOPPED(rub_RC522WatchDog) == FALSE)
	{
		/* Wait for reception */
	}

	//Confirm address
	if(APP_RC522_COMM_INTERFACE_RECEIVE() == lub_DataToSend)
	{
		APP_RC522_COMM_INTERFACE_SEND(lub_Value);
		lub_TempMX = TRUE;
		while(lub_TempMX == TRUE)
		{
			/* Wait Write */
			lub_TempMX =  GPIO_ReadPinInput(APP_RC_522_MX_GPIO_BASE_ADDR, APP_RC_522_MX_PIN_NUM);
		}

		lub_Return = FALSE;
	}
	else
	{
		//Reset RC522
		GPIO_ClearPinsOutput(APP_RC_522_RESET_GPIO_BASE_ADDR, 1U << APP_RC_522_RESET_PIN_NUM);

		GPIO_SetPinsOutput(APP_RC_522_RESET_GPIO_BASE_ADDR, 1U << APP_RC_522_RESET_PIN_NUM);
	}

	return lub_Return;
}

/**********************************************************
 * Name: app_RC522_RequestReadRegister
 * Description: This function writes a register from RC522 chip and
 * return the value
 **********************************************************/
static void app_RC522_AntennaOn(void)
{
	app_RC522_SetRegisterBitMask(TxControlReg, 0x03);
}

/**********************************************************
 * Name: app_RC522_IsANewCardPresent
 * Description: This function writes a register from RC522 chip and
 * return the value
 **********************************************************/
T_UBYTE app_RC522_IsANewCardPresent(void)
{
	T_UBYTE result;

	result = app_RC522_RequestA();

	return result;
} // End PICC_IsNewCardPresent()

/**********************************************************
 * Name: app_RC522_RequestID
 * Description: TBD
 **********************************************************/
T_UBYTE app_RC522_RequestA(void)
{
	T_UBYTE status;
	T_UBYTE sendData;

	app_RC522_WriteRegister(BitFramingReg, 0x07);		//TxLastBists = BitFramingReg[2..0]	???

	sendData = PICC_CMD_REQA;

	status = app_RC522_ToCard(PCD_Transceive, &sendData, 1U, raub_RC522_FIFOData, &ruw_RC522_FIFOReceivedLength);
	//	app_RC522_WriteRegister(CommIEnReg, 0x00);	//De solicitud de interrupciÃƒÂ³n
	//	app_RC522_ClearRegisterBitMask(CommIrqReg, 0x80);			// Borrar todos los bits de peticiÃƒÂ³n de interrupciÃƒÂ³n
	//	app_RC522_SetRegisterBitMask(FIFOLevelReg, 0x80);			//FlushBuffer=1, FIFO de inicializaciÃƒÂ³n
	//	app_RC522_WriteRegister(CommandReg, PCD_Idle);	//NO action;Y cancelar el comando
	//
	//	//Escribir datos en el FIFO
	//	app_RC522_WriteRegister(FIFODataReg, sendData);
	//
	//	//Ejecutar el comando TRANSCEIVE
	//	app_RC522_WriteRegister(CommandReg, PCD_Transceive);
	//	app_RC522_SetRegisterBitMask(BitFramingReg, 0x80);		//StartSend=1,transmission of data starts
	//
	//	static T_UBYTE lub_IrqFlags;
	//	lub_IrqFlags = 0;
	//	while(!(lub_IrqFlags & 0x01) && !((lub_IrqFlags & 0x60) != 0))
	//	{
	//		lub_IrqFlags = app_RC522_ReadRegister(CommIrqReg);
	//	}
	//
	//	app_RC522_ClearRegisterBitMask(BitFramingReg, 0x80);
	//	app_RC522_WriteRegister(CommandReg, PCD_Idle);
	//
	//	T_UBYTE lub_Error = app_RC522_ReadRegister(ErrorReg) & 0x1B;
	//	T_UBYTE lub_LastBits = app_RC522_ReadRegister(ControlReg) & 0x07;
	//	T_UBYTE lub_BackLen;
	//	if(lub_Error == FALSE)	//BufferOvfl Collerr CRCErr ProtecolErr
	//	{
	//		if(lub_LastBits != 0U)
	//		{
	//			lub_BackLen = ((app_RC522_ReadRegister(FIFOLevelReg) - 1)*8) + lub_LastBits;
	//		}
	//		else
	//		{
	//			lub_BackLen = app_RC522_ReadRegister(FIFOLevelReg)*8;
	//		}
	//
	//		if(lub_BackLen > 0U)
	//		{
	//			status = TRUE;
	//		}
	//		else
	//		{
	//			status = FALSE;
	//		}
	//	}
	//	else
	//	{
	//		status = FALSE;
	//	}

	return status;
}

/**********************************************************
 * Name: app_RC522_ClearRegisterBitMask
 * Description: Clears the bits given in mask from register reg.
 **********************************************************/
void app_RC522_ClearRegisterBitMask(
		T_UBYTE lub_Address,	///< The register to update. One of the PCD_Register enums.
		T_UBYTE lub_mask			///< The bits to clear.
) {
	T_UBYTE tmp;
	tmp = app_RC522_ReadRegister(lub_Address);
	app_RC522_WriteRegister(lub_Address, tmp & (~lub_mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()

/**********************************************************
 * Name: app_RC522_SetRegisterBitMask
 * Description: Sets the bits given in mask from register reg.
 **********************************************************/
void app_RC522_SetRegisterBitMask(
		T_UBYTE lub_Address,	///< The register to update. One of the PCD_Register enums.
		T_UBYTE lub_mask			///< The bits to set.
)
{
	T_UBYTE tmp;
	tmp = app_RC522_ReadRegister(lub_Address);
	app_RC522_WriteRegister(lub_Address, tmp | (lub_mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()

/**********************************************************
 * Name: app_RC522_ToCard
 * Description: TBD
 **********************************************************/
static T_UBYTE app_RC522_ToCard(T_UBYTE lub_command, T_UBYTE *lpub_sendData, T_UBYTE lub_sendLen, T_UBYTE *lpub_backData, T_UWORD *lpuw_backLen)
{
	T_UBYTE lub_status = STATUS_ERROR;
	T_UBYTE lub_irqEn = 0x00;
	T_UBYTE lub_waitIRq = 0x00;
	//	T_UBYTE lub_lastBits;
	T_UBYTE lub_n;
	T_UWORD lub_i;

	switch (lub_command)
	{
	case PCD_MFAuthent:
	{
		lub_irqEn = 0x12;
		lub_waitIRq = 0x10;
		break;
	}
	case PCD_Transceive:
	{
		lub_irqEn = 0x77;
		lub_waitIRq = 0x70;
		break;
	}
	default:
		break;
	}

	/* Set Interrupt Configuration */
	app_RC522_WriteRegister(CommIEnReg, lub_irqEn | 0x80);
	app_RC522_ClearRegisterBitMask(CommIrqReg, 0x80);
	app_RC522_SetRegisterBitMask(FIFOLevelReg, 0x80);

	app_RC522_WriteRegister(CommandReg, PCD_Idle);

	/* Store Data in FIFO Register */
	for(lub_i = 0; lub_i < lub_sendLen; lub_i++)
	{
		app_RC522_WriteRegister(FIFODataReg, *(lpub_sendData + lub_i));
	}

	/* Command Execution */
	app_RC522_WriteRegister(CommandReg, lub_command);
	if(lub_command == PCD_Transceive)
	{
		app_RC522_SetRegisterBitMask(BitFramingReg, 0x80);
	}
	else
	{
		/* Do Nothing */
	}

	/* Wait for Data */
	APP_RC522_TIMER_LOAD(rub_RC522WatchDog); //Load WatchDog
	do
	{
		lub_n = app_RC522_ReadRegister(CommIrqReg);
	}
	while(((lub_n & 0x01) == 0U) &&
			((lub_n & lub_waitIRq) == 0U) &&
			(APP_RC522_TIMER_IS_STOPPED(rub_RC522WatchDog) == FALSE)); //Check watch dog

	/* Stop Transmission */
	app_RC522_ClearRegisterBitMask(BitFramingReg, 0x80);

	T_UBYTE lub_Error;

	lub_Error = (app_RC522_ReadRegister(ErrorReg) & 0x1B);//BufferOvfl Collerr CRCErr ProtecolErr
	lub_Error |= (lub_n & 0x01);
	lub_Error |= APP_RC522_TIMER_IS_STOPPED(rub_RC522WatchDog);//Timeout

	/* Check for errors */
	if(lub_Error == 0U)
	{//No errors

		/* Transceive Actions*/
		if(lub_command == PCD_Transceive)
		{
			lub_n = app_RC522_ReadRegister(FIFOLevelReg);
			//			lub_lastBits = app_RC522_ReadRegister(ControlReg) & 0x07; //Check the last bits validity
			//			if(lub_lastBits != 0U)
			//			{
			//				*lpuw_backLen = (lub_n-1)*8 + lub_lastBits;
			//			}
			//			else
			//			{
			*lpuw_backLen = lub_n;//*8;
			//			}

			if(lub_n > 0U)
			{
				lub_status = STATUS_OK;
			}
			else
			{
				/*Do Nothing*/
			}

			if(lub_n == 0)
			{
				//				lub_n = 1U;
				lub_status = STATUS_ERROR;
			}
			else if(lub_n > APP_RC522_BUFFER_MAX_LENGTH)
			{
				lub_n = APP_RC522_BUFFER_MAX_LENGTH;
			}
			else
			{
				/* Do Nothing */
			}

			/* Read FIFO Data */
			for(lub_i = 0; lub_i < lub_n; lub_i++)
			{
				*(lpub_backData + lub_i) = app_RC522_ReadRegister(FIFODataReg);
			}

		}
		else
		{
			/* Do nothing */
		}
	}
	else
	{//Error found
		lub_status = STATUS_ERROR;
	}

	return lub_status;
}

/**********************************************************
 * Name: app_RC522_ReadBlock
 * Description: TBD
 **********************************************************/
static T_UBYTE app_RC522_ReadBlock(T_UBYTE lub_BlockAddr, T_UBYTE *lpub_recvData, T_UWORD *luw_unLen)
{
	T_UBYTE lub_status;

	lpub_recvData[0] = PICC_CMD_MF_READ;
	lpub_recvData[1] = lub_BlockAddr;
	app_RC522_CalculateCRC(lpub_recvData,2, &lpub_recvData[2]);
	lub_status = app_RC522_ToCard(PCD_Transceive, lpub_recvData, 4, lpub_recvData, luw_unLen);

	return lub_status;
}

/**********************************************************
 * Name: app_RC522_CalculateCRC
 * Description: TBD
 **********************************************************/
static void app_RC522_CalculateCRC(T_UBYTE *lpub_Indata, T_UBYTE lub_len, T_UBYTE *lpub_OutData)
{
	T_UBYTE i, n;

	app_RC522_ClearRegisterBitMask(DivIrqReg, 0x04);			//CRCIrq = 0
	app_RC522_SetRegisterBitMask(FIFOLevelReg, 0x80);			//Clear FIFO
	app_RC522_WriteRegister(CommandReg, PCD_Idle);

	/* Write Input Data in FIFO Buffer */
	for(i = 0; i < lub_len; i++)
	{
		app_RC522_WriteRegister(FIFODataReg, *(lpub_Indata + i));
	}

	/* CRC Calculation Command */
	app_RC522_WriteRegister(CommandReg, PCD_CalcCRC);

	//APP_RC522_TIMER_LOAD(rub_RC522WatchDog);
	do
	{
		n = app_RC522_ReadRegister(DivIrqReg);
	}
	while( ((n & 0x04) == 0) );//&& //Wait for CRC Calculation
	//(APP_RC522_TIMER_IS_STOPPED(rub_RC522WatchDog) == FALSE));

	//Guarda el calculo del CRC
	lpub_OutData[0] = app_RC522_ReadRegister(CRCResultRegL);
	lpub_OutData[1] = app_RC522_ReadRegister(CRCResultRegH);
}

static T_UBYTE app_RC522_Anticoll(T_UBYTE *lpub_serNum)
{
	T_UBYTE lub_status;
	T_UBYTE i;
	T_UBYTE lub_serNumCheck=0;


	//ClearBitMask(Status2Reg, 0x08);		//TempSensclear
	//ClearBitMask(CollReg,0x80);			//ValuesAfterColl
	app_RC522_WriteRegister(BitFramingReg, 0x00);		//TxLastBists = BitFramingReg[2..0]

	lpub_serNum[0] = PICC_CMD_SEL_CL1;
	lpub_serNum[1] = 0x20;
	lub_status = app_RC522_ToCard(PCD_Transceive, lpub_serNum, 2, lpub_serNum, &ruw_RC522_FIFOReceivedLength);

	if (lub_status == STATUS_OK)
	{
		//?????? Compruebe el nÃƒÂºmero de serie de la tarjeta
		for (i=0; i<4; i++)
		{
			lub_serNumCheck ^= lpub_serNum[i];
		}
		if (lub_serNumCheck != lpub_serNum[i])
		{
			lub_status = STATUS_ERROR;
		}
	}

	//SetBitMask(CollReg, 0x80);		//ValuesAfterColl=1

	return lub_status;
}

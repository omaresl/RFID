/*
 * app_uart.c
 *
 *  Created on: 11/09/2017
 *      Author: OmarSevilla
 */

#include <string.h>
#include "MKL25Z4.h"
#include "fsl_common.h"
#include "fsl_lpsci.h"
#include "board.h"
#include "app_uart.h"
#include "fsl_uart.h"
#include "app_RC522.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ECHO_BUFFER_LENGTH 1

/*******************************************************************************
 * Variables
 ******************************************************************************/
lpsci_handle_t g_lpsciHandle;

uint8_t g_tipString[] = "UART0 Initialized\n";

uint8_t g_txBuffer[ECHO_BUFFER_LENGTH] = {0};
uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH] = {0};

lpsci_transfer_t xfer;
lpsci_transfer_t sendXfer;
lpsci_transfer_t receiveXfer;

volatile bool rxBufferEmpty = true;
volatile bool txBufferFull = false;
volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;

volatile static T_UBYTE rub_UartBytesToSend = 0;
volatile static T_UBYTE rub_UartBytesReceived = 0;
volatile static T_UBYTE raub_SendBytes[16];
volatile static T_UBYTE raub_ReceiveBytes[16];

void app_UART_Task(void)
{
	/* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
	if ((!rxOnGoing) && rxBufferEmpty)
	{
		rxOnGoing = true;
		LPSCI_TransferReceiveNonBlocking(UART0, &g_lpsciHandle, &receiveXfer, NULL);

	}

	/* If TX is idle and g_txBuffer is full, start to send data. */
	if ((!txOnGoing) && txBufferFull)
	{
		txOnGoing = true;
		LPSCI_TransferSendNonBlocking(UART0, &g_lpsciHandle, &sendXfer);
	}

	/* If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer. */
	if ((!rxBufferEmpty) && (!txBufferFull))
	{
		memcpy(g_txBuffer, g_rxBuffer, ECHO_BUFFER_LENGTH);
		rxBufferEmpty = true;
		txBufferFull = true;
	}
	return;
}

void app_UART_UserCallback(UART0_Type *base, lpsci_handle_t *handle, status_t status, void *userData)
{
	userData = userData;

	if (kStatus_LPSCI_TxIdle == status)
	{
		txBufferFull = false;
		txOnGoing = false;
	}

	if (kStatus_LPSCI_RxIdle == status)
	{
		rxBufferEmpty = false;
		rxOnGoing = false;
	}

	return;
}

void app_UART_Init(void)
{
	LPSCI_TransferCreateHandle(UART0, &g_lpsciHandle, app_UART_UserCallback, NULL);

	/*Send Init Message*/
	xfer.data = g_tipString;
	xfer.dataSize = sizeof(g_tipString) - 1;
	txOnGoing = true;
	/*UART Initialized*/
	LPSCI_TransferSendNonBlocking(UART0, &g_lpsciHandle, &xfer);

	/* Start to echo. */
	sendXfer.data = g_txBuffer;
	sendXfer.dataSize = ECHO_BUFFER_LENGTH;
	receiveXfer.data = g_rxBuffer;
	receiveXfer.dataSize = ECHO_BUFFER_LENGTH;

	return;
}

void app_UART_SendByte(T_UBYTE lub_Data)
{
	if(rub_UartBytesToSend < 16U)
	{
#ifndef APP_RC522_UART_CHANNEL
		LPSCI_DisableInterrupts(UART0, kLPSCI_TxDataRegEmptyInterruptEnable);
		rub_UartBytesToSend++;
		raub_SendBytes[rub_UartBytesToSend - 1] = lub_Data;
		LPSCI_EnableInterrupts(UART0, kLPSCI_TxDataRegEmptyInterruptEnable);
#else
		UART_DisableInterrupts(UART2, kUART_TxDataRegEmptyInterruptEnable);
		rub_UartBytesToSend++;
		raub_SendBytes[rub_UartBytesToSend - 1] = lub_Data;
		UART_EnableInterrupts(UART2, kUART_TxDataRegEmptyInterruptEnable);
#endif
	}
	else
	{
		//Data cannot be send
		/* Do Nothing */
	}
}

void app_UART_ReceiveByte(T_UBYTE lub_Data)
{
	if(rub_UartBytesReceived < 16U)
	{
		rub_UartBytesReceived++;
		raub_ReceiveBytes[rub_UartBytesReceived - 1] = lub_Data;
	}
	else
	{
		//Cannot be received
		/* Do nothing */
	}
}

T_UBYTE app_UART_ReadReceiveData(void)
{
	T_UBYTE lub_Data;

	if(rub_UartBytesReceived > 0)
	{
		/* Get first input */
		lub_Data = raub_ReceiveBytes[0];
		rub_UartBytesReceived--;

		/* Re order buffer */
		for(T_UBYTE i = 0; i < rub_UartBytesReceived; i++)
		{
			raub_ReceiveBytes[i] = raub_ReceiveBytes[i + 1];
			raub_ReceiveBytes[i + 1] = 0U;
		}
	}
	else
	{
		lub_Data = 0x5A;
	}

	return lub_Data;
}

void app_UART_SendDataTask(void)
{
#ifndef APP_RC522_UART_CHANNEL
	/* Send data only when UART TX register is empty and ring buffer has data to send out. */
	if ((kLPSCI_TxDataRegEmptyFlag & LPSCI_GetStatusFlags(UART0)) && (rub_UartBytesToSend > 0U))
	{
		LPSCI_WriteByte(UART0, raub_SendBytes[0]);
		rub_UartBytesToSend--;

		/* Re order buffer */
		for(T_UBYTE i = 0; i < rub_UartBytesToSend; i++)
		{
			raub_SendBytes[i] = raub_SendBytes[i + 1];
			raub_SendBytes[i + 1] = 0U;
		}
	}
	else
	{
		if(rub_UartBytesToSend == 0U)
		{
			LPSCI_DisableInterrupts(UART0, kLPSCI_TxDataRegEmptyInterruptEnable);
		}
	}
#else

	/* Send data only when UART TX register is empty and ring buffer has data to send out. */
	if ((kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(UART2)) && (rub_UartBytesToSend > 0U))
	{
		UART_WriteByte(UART2, raub_SendBytes[0]);
		rub_UartBytesToSend--;

		/* Re order buffer */
		for(T_UBYTE i = 0; i < rub_UartBytesToSend; i++)
		{
			raub_SendBytes[i] = raub_SendBytes[i + 1];
			raub_SendBytes[i + 1] = 0U;
		}
	}
	else
	{
		if(rub_UartBytesToSend == 0U)
		{
			UART_DisableInterrupts(UART2, kUART_TxDataRegEmptyInterruptEnable);
		}
	}

#endif
}

T_UBYTE app_UART_GetBytesToSend(void)
{
	return rub_UartBytesToSend;
}

T_UBYTE app_UART_GetBytesReceived(void)
{
	return rub_UartBytesReceived;
}

/**********************************************************
 * Name: UART0_IRQHandler
 * Description: TBD
 **********************************************************/
void UART0_IRQHandler_User(void)
{
#ifndef APP_RC522_UART_CHANNEL
	app_UART_SendDataTask();
	/* If new data arrived. */
	if ((kUART_RxDataRegFullFlag) & LPSCI_GetStatusFlags(UART0))
	{
		/* If ring buffer is not full, add data to ring buffer. */
		app_UART_ReceiveByte(LPSCI_ReadByte(UART0));

	}
	else
	{

	}

#else
#endif
}

/**********************************************************
 * Name: UART0_IRQHandler
 * Description: TBD
 **********************************************************/
void UART2_IRQHandler_User(void)
{
#ifdef APP_RC522_UART_CHANNEL
	app_UART_SendDataTask();
	/* If new data arrived. */
	if ((kUART_RxDataRegFullFlag) & UART_GetStatusFlags(UART2))
	{
		/* If ring buffer is not full, add data to ring buffer. */
		app_UART_ReceiveByte(UART_ReadByte(UART2));

	}
	else
	{

	}
#endif
}

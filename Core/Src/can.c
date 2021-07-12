/*
 * can.c
 *
 *  Created on: Jul 10, 2021
 *      Author: Marc
 */


#include "main.h"


uint8_t Can_TxMessage(CAN_HandleTypeDef* phcan, uint8_t ide, uint32_t id, uint8_t len, uint8_t* data)
{
	// Send info header structure
	CAN_TxHeaderTypeDef tx_header;

	if (ide == 0)
	{
		// Standard frame
		tx_header.IDE = CAN_ID_STD;
		tx_header.StdId = id;
	}
	else
	{
		// Extended frame
		tx_header.IDE = CAN_ID_EXT;
		tx_header.ExtId = id;
	}

	// The frame type of the message data frame
	tx_header.RTR = CAN_RTR_DATA;
	// Length of the message
	tx_header.DLC = len;

	tx_header.TransmitGlobalTime = DISABLE;		// Do not capture time

	// Get a free mailbox to send the data
	uint16_t i = 0;
	while(HAL_CAN_GetTxMailboxesFreeLevel(phcan) == 0)
	{
		++i;
		// Timewout, sending failed
		if (i >= 0xFFFE)
			return 1;
	}
	// TODO: (Marc) If using FreeRTOS, can delay by a microsecond (or millisecond?) before sending
	//osDelay(1);

	// HAL will write to tx_mailbox the mailbox used for sending the CAN frame
	uint32_t tx_mailbox;
	// Send frame
	HAL_StatusTypeDef hal_retval = HAL_CAN_AddTxMessage(phcan, &tx_header, data, &tx_mailbox);

	if (hal_retval != HAL_OK)
		return 1;

	return 0;

}

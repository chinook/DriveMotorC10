/*
 * can.h
 *
 *  Created on: Jul 10, 2021
 *      Author: Marc
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

// **
// Can_TxMessage
//
// [IN] phcan  	-  Pointer to handle of can peripheral
// [IN] ide		-  CAN Frame type. 0 -> standard or 1 -> extended
// [IN] id		-  Frame ID  (standard frame[0,0x7FF] extended frame[0,0x1FFFFFFF])
// [IN] len		-  Length in bytes of the data to send, range: 0-8
// [IN] data	-  Pointer to data to send
//
// returns 0 on success, 1 otherwise
// **
uint8_t Can_TxMessage(CAN_HandleTypeDef* phcan, uint8_t ide, uint32_t id, uint8_t len, uint8_t* data);


#endif /* INC_CAN_H_ */

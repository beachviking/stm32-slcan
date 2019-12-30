/*
 * can_fifo.h
 *
 * Simple application FIFO for received CAN messages using the new HAL CAN driver from ST
 *
 *
 *
 *  Created on: Dec 29, 2019
 *      Author: Gunnar L. Larsen
 */

#ifndef CAN_FIFO_H_
#define CAN_FIFO_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32l4xx_hal.h"

#define CAN_FIFO_SIZE 32
#define CAN_MESSAGE_SIZE 8

#define CAN_ID_STD             (0x00000000U)  /*!< Standard Id */
#define CAN_ID_EXT             (0x00000004U)  /*!< Extended Id */


typedef struct
{
  uint32_t id;
  uint32_t ide;	// CAN_ID_STD or CAN_ID_EXT
  uint32_t dlc;
  uint8_t data[8];
} st_can_message_t;

typedef struct{
	st_can_message_t	can_msg[CAN_FIFO_SIZE];
	uint8_t 			head;
	uint8_t 			tail;
} st_can_message_fifo_t;

extern void can_fifo_init(st_can_message_fifo_t *);

extern uint8_t can_fifo_empty(st_can_message_fifo_t *);
extern uint8_t can_fifo_full(st_can_message_fifo_t *);
extern uint8_t can_fifo_size(st_can_message_fifo_t *);

extern st_can_message_t* can_fifo_dequeue(st_can_message_fifo_t *);
extern st_can_message_t* can_fifo_peek(st_can_message_fifo_t *);
extern uint8_t can_fifo_enqueue(st_can_message_fifo_t *pt, CAN_RxHeaderTypeDef *msg, uint8_t *data);

extern uint32_t hex2int(char c);

#endif /* CAN_FIFO_H_ */

/*
 * can_fifo.c
 *
 *  Created on: Dec 29, 2019
 *      Author: Gunnar L. Larsen
 */

#include "can_fifo.h"


void can_fifo_init(st_can_message_fifo_t *pt)
{
	uint8_t i,j;

	pt->head = 0;
	pt->tail = 0;

	for(j=0; j<CAN_FIFO_SIZE; j++) {
		for(i=0; i<CAN_MESSAGE_SIZE; i++)
		{
			// clear message object data buffer
			pt->can_msg[j].data[i] = 0;
		}
		pt->can_msg[j].dlc = 0;
	}
}

uint8_t can_fifo_empty(st_can_message_fifo_t *pt)
{
	return (can_fifo_size(pt)==0);
}

uint8_t can_fifo_full(st_can_message_fifo_t *pt)
{
	return (can_fifo_size(pt) == (CAN_FIFO_SIZE-1));
}

uint8_t can_fifo_size(st_can_message_fifo_t *pt)
{
	return ((CAN_FIFO_SIZE + pt->head - pt->tail) % (CAN_FIFO_SIZE));
}

st_can_message_t* can_fifo_dequeue(st_can_message_fifo_t *pt)
{
	uint8_t tmp;

	if (!can_fifo_empty(pt)) {
		tmp = pt->tail;
		pt->tail = (pt->tail + 1) % CAN_FIFO_SIZE;
		return(&pt->can_msg[tmp]);
	}

	return(0);
}

st_can_message_t* can_fifo_peek(st_can_message_fifo_t *pt)
{
	if (!can_fifo_empty(pt)) {
		return(&pt->can_msg[pt->tail]);
	} else {
		return(0);
	}
}

uint8_t can_fifo_enqueue(st_can_message_fifo_t *pt, CAN_RxHeaderTypeDef *msg, uint8_t *data)
{
	uint8_t	cpt;
	st_can_message_t *curr;

	// copy can data into fifo, if there is room
	if (!can_fifo_full(pt)) {
		curr = &pt->can_msg[pt->head];
		curr->ide = msg->IDE;

		if (msg->IDE == CAN_ID_EXT)
			curr->id = msg->ExtId;
		else
			curr->id = msg->StdId;

		curr->dlc = msg->DLC;

		for (cpt=0;cpt<msg->DLC;cpt++)
			curr->data[cpt] = data[cpt];

		pt->head = (pt->head + 1) % CAN_FIFO_SIZE;
		return (1);
	}

	return (0);
}

uint32_t hex2int(char c)
{
  if (c >= '0' && c <= '9')
  {
    return c - '0';
  }
  else if (c >= 'a' && c <= 'f')
  {
    return c - 'a' + 10;
  }
  else if (c >= 'A' && c <= 'F')
  {
    return c - 'A' + 10;
  }

  return 0;
}

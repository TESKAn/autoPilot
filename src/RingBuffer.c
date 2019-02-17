/*
 * RingBuffer.c
 *
 *  Created on: 10. feb. 2019
 *      Author: Jure
 */

#include "allinclude.h"
#include <string.h>
#include <stdlib.h>
#include "sensors/altimeter.h"

// Ring buffer functions

int16_t RB_full(RING_BUFFER* rb)
{
    if(rb->count == rb->size) return 0;
    else return -1;
}

int16_t RB_Init(RING_BUFFER* rb, uint8_t *buf, int16_t size)
{
	rb->buffer = buf;

    rb->buffer_end = rb->buffer + size;
    rb->size = size;
    rb->data_start = rb->buffer;
    rb->data_end = rb->buffer;
    rb->count = 0;

	return 0;
}

int16_t RB_push(RING_BUFFER* rb, uint8_t data)
{
	if(rb->count < rb->size)
	{
		*rb->data_end = data;
		rb->data_end++;
		if (rb->data_end == rb->buffer_end)
		{
			rb->data_end = rb->buffer;
		}
		rb->count++;
	}
	else
	{
		// Return error
		return -1;
	}
	return 0;
}

uint8_t RB_pop(RING_BUFFER* rb)
{
	if(0 < rb->count)
	{
		uint8_t data = *rb->data_start;
		rb->data_start++;
		rb->count--;
		if (rb->data_start == rb->buffer_end)
		{
			rb->data_start = rb->buffer;
		}
		// Check buffer
		if(0 == rb->count)
		{
			// start equal end?
			if(rb->data_start != rb->data_end)
			{
				// If not, fix it.
				rb->count++;
				//rb->data_start = rb->data_end;
			}
		}
		return data;
	}
	return 0;
}

int16_t RB_flush(RING_BUFFER* rb)
{
    rb->data_start = rb->buffer;
    rb->data_end = rb->buffer;
    rb->count = 0;
    return 0;
}

// 32 bit ring buffer

int16_t RB32_full(RING_BUFFER32* rb)
{
    if(rb->count == rb->size) return 0;
    else return -1;
}

int16_t RB32_Init(RING_BUFFER32* rb, uint32_t *buf, int16_t size)
{
	rb->buffer = buf;
    rb->buffer_end = rb->buffer + size;
    rb->size = size;
    rb->data_start = rb->buffer;
    rb->data_end = rb->buffer;
    rb->count = 0;

	return 0;
}

int16_t RB32_push(RING_BUFFER32* rb, uint32_t data)
{
	if(rb->count < rb->size)
	{
		*rb->data_end = data;
		rb->data_end++;
		if (rb->data_end == rb->buffer_end)
		{
			rb->data_end = rb->buffer;
		}
		rb->count++;
	}
	else
	{
		// Return error
		return -1;
	}
	return 0;
}

int32_t RB32_pop(RING_BUFFER32* rb)
{
	if(0 < rb->count)
	{
		uint32_t data = *rb->data_start;
		rb->data_start++;
		rb->count--;
		if (rb->data_start == rb->buffer_end)
		{
			rb->data_start = rb->buffer;
		}
		// Check buffer
		if(0 == rb->count)
		{
			// start equal end?
			if(rb->data_start != rb->data_end)
			{
				// If not, fix it.
				rb->count++;
				//rb->data_start = rb->data_end;
			}
		}
		return data;
	}
    return -1;
}

int16_t RB32_flush(RING_BUFFER32* rb)
{
    rb->data_start = rb->buffer;
    rb->data_end = rb->buffer;
    rb->count = 0;
    return 0;
}

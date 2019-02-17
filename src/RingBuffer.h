/*
 * RingBuffer.h
 *
 *  Created on: 10. feb. 2019
 *      Author: Jure
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

typedef struct
{
     uint8_t* buffer;
     uint8_t* buffer_end;
     uint8_t* data_start;
     uint8_t* data_end;
     volatile int32_t count;
     int32_t size;
 }__attribute__((aligned(4),packed))  RING_BUFFER;

 typedef struct
 {
      uint32_t* buffer;
      uint32_t* buffer_end;
      uint32_t* data_start;
      uint32_t* data_end;
      volatile int32_t count;
      int32_t size;
  }__attribute__((aligned(4),packed))  RING_BUFFER32;

int16_t RB_full(RING_BUFFER* rb);
int16_t RB_Init(RING_BUFFER* rb, uint8_t *buf, int16_t size);
int16_t RB_push(RING_BUFFER* rb, uint8_t data);
uint8_t RB_pop(RING_BUFFER* rb);
int16_t RB_flush(RING_BUFFER* rb);

int16_t RB32_full(RING_BUFFER32* rb);
int16_t RB32_Init(RING_BUFFER32* rb, uint32_t *buf, int16_t size);
int16_t RB32_push(RING_BUFFER32* rb, uint32_t data);
int32_t RB32_pop(RING_BUFFER32* rb);
int16_t RB32_flush(RING_BUFFER32* rb);

#endif /* RINGBUFFER_H_ */

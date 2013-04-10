/*
 * AudioComm.c
 *
 *  Created on: Apr 5, 2013
 *      Author: Jure
 */

#include "allinclude.h"

// Variables
uint8_t ACBufferA[220];
uint8_t ACBufferB[220];
uint8_t ACMessage[100];
uint8_t ACMessageIndex = 0;
uint8_t ACMessageLength = 0;
volatile Flag ACflag;
volatile int8_t AC_RDisparity = -1;
uint8_t AC_NextTransition = TRANSITION_HIGH;


// Tables
// Table for next disparity
static uint8_t ACNextDisparity5b[] =
{
	1,1,1,0,1,0,0,1,1,0,0,0,0,0,0,1,
	1,0,0,0,0,0,0,1,1,0,0,1,0,1,1,1
};
static uint8_t ACNextDisparity3b[] =
{
	1,0,0,1,1,0,0
};

// Table to encode with negative RD
static uint16_t AC6bCodeMinus[] =
{
	0b1001110000,0b0111010000,0b1011010000,0b1100010000,0b1101010000,0b1010010000,
	0b0110010000,0b1110000000,0b1110010000,0b1001010000,0b0101010000,0b1101000000,
	0b0011010000,0b1011000000,0b0111000000,0b0101110000,0b0110110000,0b1000110000,
	0b0100110000,0b1100100000,0b0010110000,0b1010100000,0b0110100000,0b1110100000,
	0b1100110000,0b1001100000,0b0101100000,0b1101100000,0b0011100000,0b1011100000,
	0b0111100000,0b1010110000
} ;

// Table to encode with positive RD
static uint16_t AC6bCodePlus[] =
{
	0b0110000000,0b1000100000,0b0100100000,0b1100010000,0b0010100000,0b1010010000,
	0b0110010000,0b0001110000,0b0001100000,0b1001010000,0b0101010000,0b1101000000,
	0b0011010000,0b1011000000,0b0111000000,0b1010000000,0b1001000000,0b1000110000,
	0b0100110000,0b1100100000,0b0010110000,0b1010100000,0b0110100000,0b0001010000,
	0b0011000000,0b1001100000,0b0101100000,0b0010010000,0b0011100000,0b0100010000,
	0b1000010000,0b0101000000
} ;

static uint16_t AC4bCodeMinus[] =
{
	0b1011,	0b1001,	0b0101,	0b1100,	0b1101,	0b1010,	0b0110
};

static uint16_t AC4bCodePlus[] =
{
	0b0100,	0b1001,	0b0101,	0b0011,	0b0010,	0b1010,	0b0110
};


// Functions

// Function to put data in buffer at location bufferStartLoc,
// store end location in bufferEndLoc
// Data has 10 bits, so have to write 20 bytes
ErrorStatus ACStoreByte(uint8_t * buffer, uint16_t data, uint8_t bufferStartLoc, uint8_t * bufferEndLoc, uint8_t * transition)
{
	int i = 0;
	//
	for(i=0; i < 10; i++)
	{
		// Check if we have enough room in buffer
		if(bufferStartLoc > AC_BUFFER_LENGTH - 2 )
		{
			// Not enough room in buffer for data, return error
			return ERROR;
		}
		// Store clock transition
		buffer[bufferStartLoc] = *transition;
		// increase buffer location
		bufferStartLoc++;
		// Check if data is 1
		if((data & 0x0001) != 0)
		{
			// Data is 1, change transition
			*transition ^= TRANSITION_HIGH;
		}
		// Store same (0) or changed(1) transition
		buffer[bufferStartLoc] = *transition;
		// Increase buffer pointer
		bufferStartLoc++;
		// Shift data by 1
		data = data >> 1;
		// Change transition to indicate next clock change
		*transition ^= TRANSITION_HIGH;
	}
	// Store last free location in buffer
	*bufferEndLoc = bufferStartLoc;
	return SUCCESS;
}
uint16_t AC_EncodeByte(uint8_t data)
{
	uint16_t encoded = 0;
	uint8_t EDCBA = 0;
	uint8_t HGF = 0;
	// Store parts
	EDCBA = data & 0x1F;
	HGF = (data >> 5) & 0x07;
	// Check AC_RDisparity
	if(AC_RDisparity < 0)
	{
		// Negative disparity
		encoded = AC6bCodeMinus[EDCBA];
	}
	else
	{
		encoded = AC6bCodePlus[EDCBA];
	}
	// Check if we have to change RD
	if(ACNextDisparity5b[EDCBA] != 0)
	{
		AC_RDisparity = ~AC_RDisparity;
	}
	// Encode high bits
	if(AC_RDisparity < 0)
	{
		encoded = encoded | AC4bCodeMinus[HGF];
	}
	else
	{
		encoded = encoded | AC4bCodePlus[HGF];
	}
	if(ACNextDisparity3b[HGF] != 0)
	{
		AC_RDisparity = ~AC_RDisparity;
	}
	return encoded;
}

void AC_FillBufferCommas(uint8_t * buffer)
{
	uint8_t bufferLoc = 0;
	// Fill buffer to the end with comma message
	while(bufferLoc < AC_BUFFER_LENGTH)
	{
		// Check disparity
		if(AC_RDisparity < 0)
		{
			ACStoreByte(buffer, AC_COMMA_SEQ_NEG, bufferLoc, &bufferLoc, &AC_NextTransition);
			AC_RDisparity = ~AC_RDisparity;
		}
		else
		{
			ACStoreByte(buffer, AC_COMMA_SEQ_POS, bufferLoc, &bufferLoc, &AC_NextTransition);
			AC_RDisparity = ~AC_RDisparity;
		}
	}
}

void AC_InitBuffers(void)
{
	AC_FillBufferCommas(ACBufferA);
	AC_FillBufferCommas(ACBufferB);
}


// Function to be called for transmitting the message
void AC_Serializer(void)
{
	uint8_t bufferLoc = 0;
	// If we are not sending any messages, we are sending commas

	// Check if there is message in buffer
	if(AC_MESSAGE_IN_BUFFER)
	{
		// Check which buffer are we currently sending
		if(!AC_SENDING_BUFFER_A)
		{
			// Not A, so set data in buffer A
			bufferLoc = 0;
			while(bufferLoc < AC_BUFFER_LENGTH)
			{
				ACStoreByte(ACBufferA, AC_EncodeByte(ACMessage[ACMessageIndex]), bufferLoc, &bufferLoc, &AC_NextTransition);
				ACMessageIndex++;
				// If whole message is in buffer
				if(ACMessageIndex >= ACMessageLength)
				{
					// Fill buffer to the end with comma message
					while(bufferLoc < AC_BUFFER_LENGTH)
					{
						// Check disparity
						if(AC_RDisparity < 0)
						{
							ACStoreByte(ACBufferA, AC_COMMA_SEQ_NEG , bufferLoc, &bufferLoc, &AC_NextTransition);
							AC_RDisparity = ~AC_RDisparity;
						}
						else
						{
							ACStoreByte(ACBufferA, AC_COMMA_SEQ_POS, bufferLoc, &bufferLoc, &AC_NextTransition);
							AC_RDisparity = ~AC_RDisparity;
						}
					}
					// Mark buffer is free
					AC_MESSAGE_IN_BUFFER = 0;
					break;
				}
			}
			// Set flags
			AC_SENDING_MESSAGE = 1;
		}
		else if(!AC_SENDING_BUFFER_B)
		{
			// Not B, so set data in buffer B
			bufferLoc = 0;
			// If there is data left, fill buffer B

			while(bufferLoc < AC_BUFFER_LENGTH)
			{
				ACStoreByte(ACBufferB, AC_EncodeByte(ACMessage[ACMessageIndex]), bufferLoc, &bufferLoc, &AC_NextTransition);
				ACMessageIndex++;
				// If whole message is in buffer
				if(ACMessageIndex >= ACMessageLength)
				{
					// Fill buffer to the end with comma message
					while(bufferLoc < AC_BUFFER_LENGTH)
					{
						// Check disparity
						if(AC_RDisparity < 0)
						{
							ACStoreByte(ACBufferB, AC_COMMA_SEQ_NEG, bufferLoc, &bufferLoc, &AC_NextTransition);
							AC_RDisparity = ~AC_RDisparity;
						}
						else
						{
							ACStoreByte(ACBufferB, AC_COMMA_SEQ_POS, bufferLoc, &bufferLoc, &AC_NextTransition);
							AC_RDisparity = ~AC_RDisparity;
						}
					}
					// Mark buffer is free
					AC_MESSAGE_IN_BUFFER = 0;
					break;
				}
			}
			// Set flags
			AC_SENDING_MESSAGE = 1;
		}
		// Message is in buffer
	}
	else	// No data waiting to be transmitted
	{
		// Check and fill buffers with commas if necessary
		if(!AC_SENDING_BUFFER_A)
		{
			if(!AC_BUFFER_A_COMMAS)
			{
				AC_FillBufferCommas(ACBufferA);
				AC_BUFFER_A_COMMAS = 1;
				AC_SENDING_MESSAGE = 0;
			}
		}
		else if(!AC_SENDING_BUFFER_B)
		{
			if(!AC_BUFFER_B_COMMAS)
			{
				AC_FillBufferCommas(ACBufferB);
				AC_BUFFER_B_COMMAS = 1;
				AC_SENDING_MESSAGE = 0;
			}
		}
	}
}

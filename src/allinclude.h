/*
 * allinclude.h
 *
 *  Created on: Oct 23, 2012
 *      Author: Jure
 */

#ifndef ALLINCLUDE_H_
#define ALLINCLUDE_H_

/* Includes */

#include "macro.h"

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include "syscalls.h"



#include "arm_math.h"
#include "math.h"

#include "mavlink.h"

#include "freemaster.h"

#include "kalman.h"

#include "math/myMath.h"
#include "sensors/sensor_typedefs.h"
#include "sensors/sensors_fusion.h"

//#include"sensors/ubx.h"

#include "functions.h"
#include "SDCard.h"
#include "RingBuffer.h"

// SD card
//#include "spi_sd.h"
#include "diskio.h"
#include "ff.h"

#include "unitConversions.h"

#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
//#include "misc.h"
#include "init.h"

#include "events.h"


#include "flight.h"



#include "var.h"

#include "CAN/can.h"

#include "sensors.h"
#include "sensors_spi.h"

#endif /* ALLINCLUDE_H_ */

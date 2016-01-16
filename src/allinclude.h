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

#include "uart_comm.h"



#include "kalman.h"

#include "sensors/sensor_typedefs.h"
#include "sensors/sensors_fusion.h"

#include "functions.h"



// File with macro functions
#include "macroFunctions.h"

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

#include "sensors.h"

#include "RS485/RS485comm.h"
#include "RS485/rs485Master.h"



#endif /* ALLINCLUDE_H_ */

/*
 * allinclude.h
 *
 *  Created on: Oct 23, 2012
 *      Author: Jure
 */

#ifndef ALLINCLUDE_H_
#define ALLINCLUDE_H_

/* Includes */

#include "stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include "syscalls.h"
#include "arm_math.h"
#include "math.h"
#include "math_custom.h"

// File with macro functions
#include "macroFunctions.h"

// USB defines
#include "usbd_hid_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

// SD card
//#include "spi_sd.h"
#include "diskio.h"
#include "ff.h"

#include "unitConversions.h"
#include "ahrs.h"

#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
//#include "misc.h"
#include "init.h"
#include "macro.h"
#include "events.h"
#include "var.h"
#include "functions.h"
#include "modbus.h"
#include "powerSensor.h"
#include "gps.h"
#include "sensors.h"
#include "compass.h"

#include "AudioComm.h"

#include "flight.h"

#include "airSpeed.h"

#endif /* ALLINCLUDE_H_ */

#ifndef __LOG_H
#define __LOG_H

#include "stdint.h"
#include "stdbool.h"
#include "stm32f10x.h"

/* Exported functions ------------------------------------------------------- */

extern void  LogTest(void);
extern void  LogEncoderData(void* enc);
extern void  LogGpsData(void* gps);
extern void  LogMotorData(void* motor);

#endif /*LOG_H*/

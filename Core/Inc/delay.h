#ifndef __DELAY_H__
#define __DELAY_H__

#include "main.h"
#include "cmsis_os.h"
#include "tim.h"

void delay_us(uint32_t us);
#define USE_FREERTOS  1



#endif /* __DELAY_H__ */

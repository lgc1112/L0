#ifndef __TIMER_H
#define __TIMER_H
#include "stm32l0xx_hal.h"

void Sys10msTimerInit(void);

extern uint32_t u32TimerCnt10ms;

#endif

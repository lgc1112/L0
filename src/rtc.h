#ifndef __RTC_H
#define __RTC_H

#include "stdint.h"
#include "main.h"

//API
void RTC_Handle_init(void);
void get_real_tim(uint8_t *time);
void RTC_TimeStampConfig(uint8_t* time);
int time_request(void);
RTC_HandleTypeDef *get_RTC_Handle(void);

//内部函数
static void Error_Handler(void);













#endif

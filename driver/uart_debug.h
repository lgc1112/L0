#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#include "main.h"

#define UART_PRINT 0

typedef void (*func_uart_Write)(char *printf_buff);

void uart_debug_init(func_uart_Write uart_write);
void DB_STR(char *printf_buf);
void DB_int(int value,char * content,char *space);
void DB_HEX(uint8 *pData,int printf_len);
void DB_HEX_str(char *str,uint8 *pData,int printf_len);

#endif


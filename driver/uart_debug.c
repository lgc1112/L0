
#include "uart_debug.h"

func_uart_Write g_uart_write=NULL;

int HexToStr(char *pbDest,unsigned char *pbSrc, int nLen)
{
	char	ddl,ddh;
	int i;

	for (i=0; i<nLen; i++)
	{
		ddh = 48 + pbSrc[i] / 16;
		ddl = 48 + pbSrc[i] % 16;
		if (ddh > 57) ddh = ddh + 7;
		if (ddl > 57) ddl = ddl + 7;
		pbDest[i*3] = ddh;
		pbDest[i*3+1] = ddl;
		pbDest[i*3+2] = ' ';
	}

	return nLen*2;
}

void uart_debug_init(func_uart_Write uart_write)
{
	g_uart_write = uart_write;
}

void DB_STR(char *printf_buf)
{
#if UART_PRINT
	g_uart_write(printf_buf);
#endif
}

void DB_int(int value,char * content,char *space)
{
#if UART_PRINT
	char str[10];

	memset(str,0,10);	
	DB_STR(content);	
	sprintf(str,"%d",value);
	DB_STR(str);	
	DB_STR(space);
#endif
}

void DB_HEX(uint8 *pData,int printf_len)
{
#if UART_PRINT
	char retPacket[100];

	memset(retPacket,0,100);
	HexToStr(retPacket,pData,printf_len);
	g_uart_write(retPacket);
#endif
}

void DB_HEX_str(char *str,uint8 *pData,int printf_len)
{
#if UART_PRINT
	DB_STR("\r\n");
	DB_STR(str);
	DB_HEX(pData,printf_len);
#endif
}



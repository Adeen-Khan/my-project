#include "my_printf.h"
#include "stdarg.h" 
#include "stdio.h" 
#include "main.h" 
#include "stdint.h"


extern UART_HandleTypeDef huart2;

void myPrintf(const char *fmt, ...)
{
char buf[128]; 
va_list args;
va_start(args, fmt);
int n = vsnprintf(buf, sizeof(buf), fmt, args);
va_end(args);

if (n < 0) return; 


uint16_t len = (n < (int)sizeof(buf)) ? (uint16_t)n : (uint16_t)(sizeof(buf) - 1);

HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
}
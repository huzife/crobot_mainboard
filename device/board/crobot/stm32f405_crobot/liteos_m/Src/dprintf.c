#include <stdarg.h>
#include "los_interrupt.h"
#include <stdio.h>
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;

INT32 UartPutc(INT32 ch, VOID *file)
{
    char RL = '\r';
    if (ch =='\n') {
        HAL_UART_Transmit(&huart1, &RL, 1, 0xFFFF);
    }
    return HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
}

static void dputs(char const *s, int (*pFputc)(int n, FILE *cookie), void *cookie)
{
    unsigned int intSave;

    intSave = LOS_IntLock();
    while (*s) {
        pFputc(*s++, cookie);
    }
    LOS_IntRestore(intSave);
}

int printf(char const  *fmt, ...)
{
    char buf[1024] = { 0 };
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf_s(buf, sizeof(buf), 1024 - 1, fmt, ap);
    va_end(ap);
    if (len > 0) {
        dputs(buf, UartPutc, 0);
    } else {
        dputs("printf error!\n", UartPutc, 0);
    }
    return len;
}

/**
 * @file   syscall.c
 * @date   28.02.2016
 * @author andreas
 * @brief  
 */


#include <stdio.h>
#include <sys/stat.h>

#include "debug.h"

#include "stm32l0xx.h"

//------------------------------------------------------------------------------
int _fstat (int fd, struct stat *pStat)
{
    pStat->st_mode = S_IFCHR;
    return 0;
}

//------------------------------------------------------------------------------
int _close(int fd)
{
    return -1;
}

//------------------------------------------------------------------------------
int _write (int fd, char *buffer, int size)
{
    int i;

    for(i=0; i<size; i++)
    {
        while(!(USART1->ISR & USART_ISR_TXE));
        USART1->TDR = buffer[i];
    }

    return size;
}

//------------------------------------------------------------------------------
int _isatty (int fd)
{
    return 1;
}

//------------------------------------------------------------------------------
int _lseek(int fd, int offset, int whence)
{
    return -1;
}

//------------------------------------------------------------------------------
int _read (int fd, char *buffer, int size)
{
    int i;
    for(i = 0; i < size; i++)
    {
        while((USART1->ISR & USART_ISR_RXNE) == 0)
        {
        }

        buffer[i] = USART1->RDR;
    }
    return size;
}

//------------------------------------------------------------------------------
caddr_t _sbrk(int increment)
{
    return 0;
}

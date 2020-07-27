/*
 * utils.c
 *
 *  Created on: 02.09.2019
 *      Author: DI Andreas Auer
 */

#include "utils.h"

#include <ctype.h>
#include <string.h>

static const char HEX[16] = "0123456789ABCDEF";

//------------------------------------------------------------------------------
char *rstrstr(const char *haystack, const char *needle)
{
  size_t  s1len = strlen(haystack);
  size_t  s2len = strlen(needle);
  char *s;

  if (s2len > s1len)
    return NULL;
  for (s = haystack + s1len - s2len; s >= haystack; --s)
    if (strncmp(s, needle, s2len) == 0)
      return s;
  return NULL;
}

//------------------------------------------------------------------------------
void ltrim(char *str)
{
    uint32_t i=0;
    uint32_t j=0;

    for(i=0; str[i] != '\0'; i++)
    {
        if(str[i] > 32 && str[i] < 127)
        {
            break;
        }
    }

    if(i > 0)
    {
        for(j=0; str[i] != '\0'; j++, i++)
        {
            str[j] = str[i];
        }
        str[j] = '\0';
    }
}

//------------------------------------------------------------------------------
void rtrim(char *str)
{
    uint32_t i=0;
    uint32_t len = strlen(str);
    if(len == 0)
    {
        return;
    }

    for(i=len-1; i>0; i--)
    {
        if(str[i] > 32 && str[i] < 127)
        {
            str[i+1] = '\0';
            break;
        }
    }
}

//------------------------------------------------------------------------------
void toHexString(char *str, const uint8_t *data, uint32_t size)
{
    uint32_t i=0;
    for(i=0; i<size; i++)
    {
        uint8_t temp = data[i] / 16;
        str[2*i]   = HEX[temp];
        str[2*i+1] = HEX[data[i] & 0xF];
    }
    str[2*i] = '\0';
}

//------------------------------------------------------------------------------
uint32_t fromHexString(const char *str, uint8_t *data)
{
    uint32_t i=0;
    uint8_t temp = 0;

    for(i=0; str[i] != '\0'; i++)
    {
        char ch = toupper(str[i]);
        if(ch >= 'A' && ch <= 'F')
        {
            temp = (ch - 'A' + 10);
        }
        else
        {
            temp = (ch - '0');
        }

        if((i & 0x01) != 0)
        {
            data[i / 2] |= temp;
        }
        else
        {
            data[i / 2]  = temp << 4;
        }
    }
    data[i/2] = '\0';

    return i/2;
}

//------------------------------------------------------------------------------
uint16_t bswap16(uint16_t input)
{
    return (input >> 8) | (input << 8);
}

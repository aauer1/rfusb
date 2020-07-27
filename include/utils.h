/*
 * utils.h
 *
 *  Created on: 02.09.2019
 *      Author: DI Andreas Auer
 */
#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

char *rstrstr(const char *haystack, const char *needle);
void ltrim(char *str);
void rtrim(char *str);
void toHexString(char *str, const uint8_t *data, uint32_t size);
uint32_t fromHexString(const char *str, uint8_t *data);
uint16_t bswap16(uint16_t input);

#endif /* UTILS_H_ */

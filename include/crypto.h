/**
 * @file   crypto.h
 * @date   29.10.2018
 * @author DI Andreas Auer
 * @brief  
 */

#ifndef CRYPTO_H_
#define CRYPTO_H_

#include <stdint.h>

void cryptoInit(uint8_t *key, uint8_t *iv);
void cryptoEncrypt(uint8_t *plain, uint16_t len, uint8_t *cipher);
void cryptoDecrypt(uint8_t *cipher, uint16_t len, uint8_t *plain);

#endif /* CRYPTO_H_ */

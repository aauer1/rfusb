/**
 * @file   crypto_.c
 * @date   29.10.2018
 * @author DI Andreas Auer
 * @brief  
 */

#include "crypto.h"
#include "debug.h"

#include <stm32l0xx.h>

static CRYP_HandleTypeDef crypto_;

//------------------------------------------------------------------------------
void cryptoInit(uint8_t *key, uint8_t *iv)
{
    __HAL_RCC_AES_CLK_ENABLE();

    crypto_.Instance = AES;
    crypto_.Init.DataType = CRYP_DATATYPE_32B;
    crypto_.Init.pInitVect = iv;
    crypto_.Init.pKey = key;
    crypto_.State = HAL_CRYP_STATE_RESET;

    HAL_CRYP_Init(&crypto_);
}

//------------------------------------------------------------------------------
void cryptoEncrypt(uint8_t *plain, uint16_t len, uint8_t *cipher)
{
    HAL_StatusTypeDef ret;

    HAL_CRYP_Init(&crypto_);
    ret = HAL_CRYP_AESCBC_Encrypt(&crypto_, plain, len, cipher, 1000);
    if(ret != HAL_OK)
    {
        debug("AES encryption FAILED");
    }

    HAL_CRYP_DeInit(&crypto_);
}

//------------------------------------------------------------------------------
void cryptoDecrypt(uint8_t *cipher, uint16_t len, uint8_t *plain)
{
    HAL_StatusTypeDef ret;

    HAL_CRYP_Init(&crypto_);
    ret = HAL_CRYP_AESCBC_Decrypt(&crypto_, cipher, len, plain, 1000);
    if(ret != HAL_OK)
    {
        debug("AES decryption FAILED");
    }
    HAL_CRYP_DeInit(&crypto_);
}

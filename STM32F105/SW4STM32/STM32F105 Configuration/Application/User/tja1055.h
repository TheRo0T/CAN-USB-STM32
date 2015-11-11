#ifndef _TJA1055_H
#define _TJA1055_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct
{
    GPIO_TypeDef *enPort;
    uint16_t enPin;
    GPIO_TypeDef *stbPort;
    uint16_t stbPin;
    GPIO_TypeDef *errPort;
    uint16_t errPin;
} TJA1055_HandleTypeDef;



void TJA1055_Init(TJA1055_HandleTypeDef *tja1055);

void TJA1055_NormalMode(TJA1055_HandleTypeDef *tja1055);

void TJA1055_StanbyMode(TJA1055_HandleTypeDef *tja1055);

void TJA1055_GotoSleepMode(TJA1055_HandleTypeDef *tja1055);

void TJA1055_PoweronStanbyMode(TJA1055_HandleTypeDef *tja1055);

#endif

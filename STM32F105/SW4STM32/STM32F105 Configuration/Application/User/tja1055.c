#include "tja1055.h"

void TJA1055_Init(TJA1055_HandleTypeDef *tja1055) {
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = tja1055->enPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(tja1055->enPort, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = tja1055->stbPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(tja1055->stbPort, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = tja1055->errPin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(tja1055->errPort, &GPIO_InitStruct);
}

void TJA1055_NormalMode(TJA1055_HandleTypeDef *tja1055) {
    HAL_GPIO_WritePin(tja1055->stbPort, tja1055->stbPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(tja1055->enPort, tja1055->enPin, GPIO_PIN_SET);
}

void TJA1055_StanbyMode(TJA1055_HandleTypeDef *tja1055) {
    HAL_GPIO_WritePin(tja1055->stbPort, tja1055->stbPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(tja1055->enPort, tja1055->enPin, GPIO_PIN_RESET);
}

void TJA1055_GotoSleepMode(TJA1055_HandleTypeDef *tja1055) {
    HAL_GPIO_WritePin(tja1055->stbPort, tja1055->stbPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(tja1055->enPort, tja1055->enPin, GPIO_PIN_SET);
}

void TJA1055_PoweronStanbyMode(TJA1055_HandleTypeDef *tja1055) {
    HAL_GPIO_WritePin(tja1055->stbPort, tja1055->stbPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(tja1055->enPort, tja1055->enPin, GPIO_PIN_RESET);
}

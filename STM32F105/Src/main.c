/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include "canhacker.h"
#include "tja1055.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId defaultTaskHandle;
osMessageQId uartTxQueueHandle;
osMessageQId uartRxQueueHandle;
osMessageQId canRxQueueHandle;
osMessageQId canTxQueueHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define UART_RX_LINE_BUFFER 80
int timer = 0;
uint8_t uartLine[UART_RX_LINE_BUFFER];
int uartLineIndex = 0;
#define UART_RX_BUFFER 512

static CanTxMsgTypeDef        can1TxMessage;
static CanRxMsgTypeDef        can1RxMessage;

TJA1055_HandleTypeDef tja1055;
CanHacker_HandleTypeDef hcanhacker;

typedef struct
{
    uint8_t buffer[UART_RX_BUFFER];
    uint8_t readed;
    uint8_t filled;
    int offset;
} UART_DMA_RX_Buffer;

UART_DMA_RX_Buffer uartRxBuffer[2]= {
    { .readed = 1, .filled = 0, .offset = 0 },
    { .readed = 1, .filled = 0, .offset = 0 }
};
int currentUartRxBuffer = 0;
int currentUartRxBufferToRead = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void throwError(char *msg);

void txUart();
void txCan();
void rxCan();
void transmitErrorMessage(char *message);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
    TJA1055_Init(&tja1055);

    CanHacker_Init(&hcanhacker);

    hcan1.pTxMsg = &can1TxMessage;
    hcan1.pRxMsg = &can1RxMessage;


    CAN_FilterConfTypeDef canFilterConfig;
    canFilterConfig.FilterNumber = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig.FilterIdHigh = 0x0000;
    canFilterConfig.FilterIdLow = 0x0000;
    canFilterConfig.FilterMaskIdHigh = 0x0000 << 5;
    canFilterConfig.FilterMaskIdLow = 0x0000;
    canFilterConfig.FilterFIFOAssignment = 0;
    canFilterConfig.FilterActivation = ENABLE;
    canFilterConfig.BankNumber = 1;
    HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of uartTxQueue */
  osMessageQDef(uartTxQueue, 16, int);
  uartTxQueueHandle = osMessageCreate(osMessageQ(uartTxQueue), NULL);

  /* definition and creation of uartRxQueue */
  osMessageQDef(uartRxQueue, 16, int);
  uartRxQueueHandle = osMessageCreate(osMessageQ(uartRxQueue), NULL);

  /* definition and creation of canRxQueue */
  osMessageQDef(canRxQueue, 16, int);
  canRxQueueHandle = osMessageCreate(osMessageQ(canRxQueue), NULL);

  /* definition and creation of canTxQueue */
  osMessageQDef(canTxQueue, 16, int);
  canTxQueueHandle = osMessageCreate(osMessageQ(canTxQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      osDelay(1000);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBPLLCLK_DIV3;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/8000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK_DIV8);

  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_6TQ;
  hcan1.Init.BS2 = CAN_BS2_5TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  HAL_CAN_Init(&hcan1);

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 35999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 60000;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : TJA_EN_Pin TJA_STB_Pin */
  GPIO_InitStruct.Pin = TJA_EN_Pin|TJA_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TJA_ERR_Pin */
  GPIO_InitStruct.Pin = TJA_ERR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TJA_ERR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void throwError(char *msg)
{
    transmitErrorMessage(msg);
    while(1) {
        HAL_Delay(1000);
    };
}

void txUart() {
  HAL_UART_StateTypeDef state = HAL_UART_GetState(&huart1);

  if (state == HAL_UART_STATE_BUSY_TX_RX || state == HAL_UART_STATE_BUSY_TX) {
  } else {

    osEvent event = osMessageGet(uartTxQueueHandle, osWaitForever);
    if (event.status == osEventMessage) {
      char *str = event.value.p;
      int len = strlen(str);
      HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)str, len);
      switch (result) {
        case HAL_OK: break;
        case HAL_ERROR:
            throwError("HAL_UART_Transmit_IT: error");
            break;
        case HAL_BUSY:
            //throwError("HAL_UART_Transmit_IT: busy");
            break;
        case HAL_TIMEOUT:
            throwError("HAL_UART_Transmit_IT: timeout");
            break;
      }
    }
  }
}

/*void txUart() {
  HAL_UART_StateTypeDef state = HAL_UART_GetState(&huart1);

  if (state == HAL_UART_STATE_BUSY_TX_RX || state == HAL_UART_STATE_BUSY_TX) {
  } else {

    osEvent event = osMessageGet(uartTxQueueHandle, osWaitForever);
    if (event.status == osEventMessage) {
      char *str = event.value.p;
      int len = strlen(str);
      HAL_StatusTypeDef result = HAL_UART_Transmit_IT(&huart1, (uint8_t*)str, len);
      switch (result) {
        case HAL_OK: break;
        case HAL_ERROR:
            throwError("HAL_UART_Transmit_IT: error");
            break;
        case HAL_BUSY:
            //throwError("HAL_UART_Transmit_IT: busy");
            break;
        case HAL_TIMEOUT:
            throwError("HAL_UART_Transmit_IT: timeout");
            break;
      }
    }
  }
}*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->pTxBuffPtr) {
        free(huart->pTxBuffPtr);
    }
    txUart();
}

void startUartDmaReceive(UART_HandleTypeDef *huart, UART_DMA_RX_Buffer *uartRxBuffer) {
    uartRxBuffer->readed = 0;
    HAL_StatusTypeDef result = HAL_UART_Receive_DMA(huart, uartRxBuffer->buffer, UART_RX_BUFFER);
    switch (result) {
        case HAL_OK: break;
        case HAL_ERROR: throwError("HAL_UART_Receive_DMA: error"); break;
        case HAL_BUSY: throwError("HAL_UART_Receive_DMA: busy"); break;
        case HAL_TIMEOUT: throwError("HAL_UART_Receive_DMA: timeout"); break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART1) {
        // switch buffer
        uartRxBuffer[currentUartRxBuffer].filled = 1;
        currentUartRxBuffer = currentUartRxBuffer == 0 ? 1 : 0;
        if (!uartRxBuffer[currentUartRxBuffer].readed) {
            throwError("UART RX OVERRUN");
        }

        startUartDmaReceive(huart, &uartRxBuffer[currentUartRxBuffer]);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    uint32_t code = huart->ErrorCode;
    if (code & HAL_UART_ERROR_PE) { throwError("UART error: Parity error"); }
    if (code & HAL_UART_ERROR_NE) { throwError("UART error: Noise error"); }
    if (code & HAL_UART_ERROR_FE) { throwError("UART error: frame error"); }
    if (code & HAL_UART_ERROR_ORE) { throwError("UART error: Overrun error"); }
    if (code & HAL_UART_ERROR_DMA) { throwError("UART error: DMA error"); }
    char str[80];
    sprintf(str, "Uart error code: %X", (unsigned int)code);
    throwError(str);
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan) {
    printf("HAL_CAN_TxCpltCallback\n");
    txCan();
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    CanRxMsgTypeDef *rxMsg = malloc(sizeof(CanRxMsgTypeDef));
    memcpy(rxMsg, hcan->pRxMsg, sizeof(CanRxMsgTypeDef));

    osMessagePut (canRxQueueHandle, (uint32_t)rxMsg, osWaitForever);
    rxCan();
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    throwError("HAL_CAN_ErrorCallback");
}

void rxCan() {
    HAL_StatusTypeDef result = HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
    switch (result) {
        case HAL_OK: break;
        case HAL_ERROR:
            throwError("HAL_CAN_Receive_IT: error");
            break;
        case HAL_BUSY:
            throwError("HAL_CAN_Receive_IT: busy");
            break;
        case HAL_TIMEOUT:
            throwError("HAL_CAN_Receive_IT: timeout");
            break;
    }
    switch (hcan1.ErrorCode) {
        case HAL_CAN_ERROR_NONE: break;       /*!< No error             */
        case HAL_CAN_ERROR_EWG:               /*!< EWG error            */
        case HAL_CAN_ERROR_EPV:               /*!< EPV error            */
        case HAL_CAN_ERROR_BOF:               /*!< BOF error            */
        case HAL_CAN_ERROR_STF:               /*!< Stuff error          */
        case HAL_CAN_ERROR_FOR:               /*!< Form error           */
        case HAL_CAN_ERROR_ACK:               /*!< Acknowledgment error */
        case HAL_CAN_ERROR_BR:                /*!< Bit recessive        */
        case HAL_CAN_ERROR_BD:                /*!< LEC dominant         */
        case HAL_CAN_ERROR_CRC:
            throwError("RXCAN ERROR");
            break;
    }
}

void txCan() {
    HAL_CAN_StateTypeDef state = HAL_CAN_GetState(&hcan1);

    if (state == HAL_CAN_STATE_READY || state == HAL_CAN_STATE_BUSY_RX) {

        osEvent event = osMessageGet(canTxQueueHandle, osWaitForever);
        if (event.status == osEventMessage) {
            //memcpy(hcan.pTxMsg, event.value.p, sizeof(CanTxMsgTypeDef));
            can1TxMessage = *(CanTxMsgTypeDef *)event.value.p;
            HAL_StatusTypeDef result = HAL_CAN_Transmit_IT(&hcan1);
            free(event.value.p);
            switch (result) {
                case HAL_OK: break;
                case HAL_ERROR:
                    throwError("HAL_CAN_Transmit_IT: error");
                    break;
                case HAL_BUSY:
                    throwError("HAL_CAN_Transmit_IT: busy");
                    break;
                case HAL_TIMEOUT:
                    throwError("HAL_CAN_Transmit_IT: timeout");
                    break;
            }
            switch (hcan1.ErrorCode) {
                case HAL_CAN_ERROR_NONE: break;       /*!< No error             */
                case HAL_CAN_ERROR_EWG:               /*!< EWG error            */
                case HAL_CAN_ERROR_EPV:               /*!< EPV error            */
                case HAL_CAN_ERROR_BOF:               /*!< BOF error            */
                case HAL_CAN_ERROR_STF:               /*!< Stuff error          */
                case HAL_CAN_ERROR_FOR:               /*!< Form error           */
                case HAL_CAN_ERROR_ACK:               /*!< Acknowledgment error */
                case HAL_CAN_ERROR_BR:                /*!< Bit recessive        */
                case HAL_CAN_ERROR_BD:                /*!< LEC dominant         */
                case HAL_CAN_ERROR_CRC:
                    throwError("TXCAN error advanced");
                    break;
            }
        }
    } else {
        printf("State is %d\n", state);
        transmitErrorMessage("State");
        //Error_Handler_Period(500);
    }
}

void transmitErrorMessage(char *message) {
    int len = strlen((char *)message);
    char *str = malloc(len+3);
    memcpy(str, message, len);
    str[len] = '\r';
    str[len+1] = '\n';
    str[len+2] = '\0';


    osMessagePut (uartTxQueueHandle, (uint32_t)str, osWaitForever);
    txUart();

    return;
}

void gotoNextBufferToRead() {
    if (currentUartRxBufferToRead != currentUartRxBuffer) {
        currentUartRxBufferToRead++;
        currentUartRxBufferToRead %= 2;
    }
}

void processUartDmaBuffer(UART_HandleTypeDef *huart, UART_DMA_RX_Buffer *uartRxBuffer) {
    if (uartRxBuffer->readed) {
        return;
    }

    int dmaOffset;
    if (uartRxBuffer->filled) {
        dmaOffset = UART_RX_BUFFER;
    } else {
        dmaOffset = huart->RxXferSize - huart->hdmarx->Instance->CNDTR;
    }
    int bytesReady = dmaOffset - uartRxBuffer->offset;

    if (bytesReady > 0) {

        while(uartRxBuffer->offset < dmaOffset) {
            uint8_t c = uartRxBuffer->buffer[uartRxBuffer->offset];
            switch (c) {
                case '\0':
                case '\r':
                case '\n':
                    if (uartLineIndex > 0) {
                        uartLine[uartLineIndex++] = '\0';
                        /*uint8_t *str = malloc(uartLineIndex);
                        memcpy(str, uartLine, uartLineIndex);*/

                        CanHacker_Receive_Cmd(&hcanhacker, uartLine);

                        uartLineIndex = 0;
                        /*osStatus status = osMessagePut(uartRxQueueHandle, (uint32_t)str, osWaitForever);
                        switch (status) {
                            case osOK:
                                break;
                            default:
                                throwError("Error put to queue");
                                break;
                        }*/
                    }
                    break;
                default:
                    uartLine[uartLineIndex++] = c;
                    break;
            }

            uartRxBuffer->offset++;
        }
    }

    if (uartRxBuffer->offset >= UART_RX_BUFFER) {
        uartRxBuffer->readed = 1;
        uartRxBuffer->offset = 0;
        uartRxBuffer->filled = 0;

        gotoNextBufferToRead();
    }
}

void CanHacker_ErrorCallback(CanHacker_HandleTypeDef *canhacker, char *message) {
    throwError(message);
}

void CanHacker_CanTxMsgReadyCallback(CanHacker_HandleTypeDef *canhacker, CanTxMsgTypeDef *txMsg) {
    CanTxMsgTypeDef *txMsgQ = malloc(sizeof(CanTxMsgTypeDef));
    memcpy(txMsgQ, txMsg, sizeof(CanTxMsgTypeDef));

    osMessagePut (canTxQueueHandle, (uint32_t)txMsgQ, osWaitForever);
    txCan();

    return;
}

void CanHacker_UartMsgReadyCallback(CanHacker_HandleTypeDef *canhacker, uint8_t *line) {
    int len = strlen((char *)line);
    char *str = malloc(len+1);
    memcpy(str, line, len+1);
    osMessagePut (uartTxQueueHandle, (uint32_t)str, osWaitForever);
    txUart();
}

uint32_t CanHacker_GetTimestampCallback(CanHacker_HandleTypeDef *canhacker) {
    return htim6.Instance->CNT;
}

void CanHacker_StartTimerCallback(CanHacker_HandleTypeDef *canhacker) {
    HAL_TIM_Base_Start(&htim6);
}

void CanHacker_StopTimerCallback(CanHacker_HandleTypeDef *canhacker) {
    HAL_TIM_Base_Stop(&htim6);
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  //MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
    rxCan();

    currentUartRxBuffer = 0;
    currentUartRxBufferToRead = 0;
    startUartDmaReceive(&huart1, &uartRxBuffer[currentUartRxBuffer]);

    /* Infinite loop */
    for(;;)
    {
        processUartDmaBuffer(&huart1, &uartRxBuffer[currentUartRxBufferToRead]);

        /*osEvent uartEvent = osMessageGet(uartRxQueueHandle, 10); // osWaitForever
        if (uartEvent.status == osEventMessage) {

            CanHacker_Receive_Cmd(uartEvent.value.p, uartTxQueueHandle, canTxQueueHandle);
            free(uartEvent.value.p);

            //osMessagePut (uartTxQueueHandle, (uint32_t)event.value.p, osWaitForever);
            //txUart();
        }*/

        osEvent canEvent = osMessageGet(canRxQueueHandle, 10); // osWaitForever
        if (canEvent.status == osEventMessage) {

            CanHacker_Receive_CanMsg(&hcanhacker, (CanRxMsgTypeDef *)canEvent.value.p);
            free(canEvent.value.p);


        }
        osDelay(1);
    }
  /* USER CODE END 5 */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

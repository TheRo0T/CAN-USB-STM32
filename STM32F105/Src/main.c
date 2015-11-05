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

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include "canhacker.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osMessageQId uartTxQueueHandle;
osMessageQId uartRxQueueHandle;
osMessageQId canRxQueueHandle;
osMessageQId canTxQueueHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define UART_RX_LINE_BUFFER 80
uint8_t inCharacter;
int timer = 0;
uint8_t uartLine[UART_RX_LINE_BUFFER];
int uartLineIndex = 0;

static CanTxMsgTypeDef        can1TxMessage;
static CanRxMsgTypeDef        can1RxMessage;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void throwError(char *msg);

void txUart();
void rxUart();
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
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
//  HAL_StatusTypeDef result;
  while (1)
  {


      osDelay(1000);

      /*result = HAL_CAN_Receive(&hcan1, CAN_FIFO0, HAL_MAX_DELAY);
      switch (result) {
          case HAL_OK:
              printf("Success");
              break;
          case HAL_ERROR:
              Error_Handler_Period(500);
              break;
          case HAL_BUSY:
              Error_Handler_Period(1000);
              break;
          case HAL_TIMEOUT:
              Error_Handler_Period(250);
              break;
      }*/


          /*result = HAL_UART_Transmit_IT(&huart1, (uint8_t *)"Z", 1);
          printf("%d", result);
          HAL_Delay(100);
          result = HAL_UART_Transmit(&huart1, (uint8_t *)"z", 1, 1000);
          printf("%d", result);
          HAL_Delay(100);*/

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

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
}

void rxUart() {
  HAL_StatusTypeDef result = HAL_UART_Receive_IT(&huart1, &inCharacter, 1);
  switch (result) {
    case HAL_OK: break;
    case HAL_ERROR: throwError("rxUart: error"); break;
    case HAL_BUSY: /*throwError("rxUart: busy");*/ break;
    case HAL_TIMEOUT: throwError("rxUart: timeout"); break;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    //free(huart->pTxBuffPtr);
    txUart();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    switch (inCharacter) {
        case '\r':
        case '\n':
        case '\0':
            if (uartLineIndex > 0) {
                uartLine[uartLineIndex++] = '\0';
                char *str = malloc(uartLineIndex);
                memcpy(str, uartLine, uartLineIndex);
                uartLineIndex = 0;
                osMessagePut (uartRxQueueHandle, (uint32_t)str, osWaitForever);
            }
            break;
        default:
            if (uartLineIndex < UART_RX_LINE_BUFFER-1) {
                uartLine[uartLineIndex++] = inCharacter;
            }
            break;
    }
    rxUart();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    transmitErrorMessage("UART error");
    uint32_t code = huart->ErrorCode;
    if (code & HAL_UART_ERROR_PE) { transmitErrorMessage("Parity error"); }
    if (code & HAL_UART_ERROR_NE) { transmitErrorMessage("Noise error"); }
    if (code & HAL_UART_ERROR_FE) { transmitErrorMessage("frame error"); }
    if (code & HAL_UART_ERROR_ORE) { transmitErrorMessage("Overrun error"); }
    if (code & HAL_UART_ERROR_DMA) { transmitErrorMessage("DMA error"); }
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
            throwError("RXCAN error");
            break;
        case HAL_BUSY:
            throwError("RXCAN busy");
            break;
        case HAL_TIMEOUT:
            throwError("RXCAN timeout");
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
                    throwError("TXCAN error");
                    break;
                case HAL_BUSY:
                    throwError("TXCAN busy");
                    break;
                case HAL_TIMEOUT:
                    throwError("TXCAN timeout");
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
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
    //transmitErrorMessage("StartDefaultTask");
    rxCan();
    rxUart();

  /* Infinite loop */
    for(;;)
    {
        //int speed = timer > 0 ? i * 8 / timer : 0;
        //sprintf(formated, "%04X %05.1f %04X\r\n", i, speed, timer);
        //sprintf(formated, "%06d\r\n", speed);

        /*UBaseType_t availableRx = uxQueueSpacesAvailable(uartRxQueueHandle);
        UBaseType_t availableTx = uxQueueSpacesAvailable(uartTxQueueHandle);
        sprintf(formated, "%06d %06d\r\n", availableRx, availableTx);
        osMessagePut (uartTxQueueHandle, (uint32_t)formated, osWaitForever);

        //osMessagePut (uartTxQueueHandle, (uint32_t)str[i%3], osWaitForever);
        txUart();
        i++;

        osDelay(1000);*/

        osEvent uartEvent = osMessageGet(uartRxQueueHandle, 10); // osWaitForever
        if (uartEvent.status == osEventMessage) {

            exec_usb_cmd(uartEvent.value.p, uartTxQueueHandle, canTxQueueHandle);
            free(uartEvent.value.p);

            /*osMessagePut (uartTxQueueHandle, (uint32_t)event.value.p, osWaitForever);
            txUart();*/
        }

        osEvent canEvent = osMessageGet(canRxQueueHandle, 10); // osWaitForever
        if (canEvent.status == osEventMessage) {
            CanRxMsgTypeDef *msg = canEvent.value.p;

            char *str = malloc(22);
            sprintf(str, "t%03X%01X", msg->StdId, msg->DLC);
            int i;
            for (i=0; i<8; i++) {
                sprintf(str + 5 + i*2, "%02X ", msg->Data[i]);
            }
            str[20] = CANHACKER_CR;
            str[21] = '\0';

            free(canEvent.value.p);

            osMessagePut (uartTxQueueHandle, (uint32_t)str, osWaitForever);
            txUart();
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

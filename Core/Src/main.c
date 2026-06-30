/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "iwdg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;	//Instance of the TxHeader structure
CAN_RxHeaderTypeDef RxHeader;	//Instance of the RxHeader structure
CAN_FilterTypeDef Filter;

uint8_t TxData[8];				//TxData buffer
uint8_t RxData[8];				//RxData buffer
uint16_t ADC_Array[12];
uint32_t TxMailbox;				//TxMailbox address (Handled by HAL)
uint16_t max=0;

uint8_t RxReady = 0;			//Flag indicating that a new CAN message is ready

typedef struct
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CAN_Message_t;

#define CAN_RX_QUEUE_SIZE 32

CAN_Message_t CanRxQueue[CAN_RX_QUEUE_SIZE];

volatile uint16_t CanRxHead = 0;
volatile uint16_t CanRxTail = 0;
volatile uint32_t CanRxOverflow = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)							//Message pending callback function
//{
//    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)							//While() loop ensures all CAN frames in FIFO0 are drained (if > 1)
//    {
//    	TxData[0] = 0x11;
//    	TxData[1] = 0x11;
//    	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
//        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)		//Release one frame from FIFO0 to memory
//        {
//            RxReady = 1; 																//Message ready flag. Add Queue if dealing with multiple frames at once
//        }
//    }
//}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    while(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        uint16_t nextHead =
            (CanRxHead + 1) % CAN_RX_QUEUE_SIZE;

        if(nextHead == CanRxTail)
        {
            /* Queue full */
            CanRxOverflow++;
            break;
        }

        if(HAL_CAN_GetRxMessage(
                hcan,
                CAN_RX_FIFO0,
                &CanRxQueue[CanRxHead].header,
                CanRxQueue[CanRxHead].data) == HAL_OK)
        {
            CanRxHead = nextHead;
        }
    }
}
volatile uint32_t LastCanErr = 0;
volatile uint32_t LastCanEsr = 0;
volatile uint32_t LastCanInstance = 0;

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    LastCanErr = hcan->ErrorCode;
    LastCanEsr = hcan->Instance->ESR;
    LastCanInstance = (uint32_t)hcan->Instance;
}

uint8_t CAN_ReadMessage(CAN_Message_t *msg)
{
    if(CanRxHead == CanRxTail)
    {
        return 0; // queue empty
    }

    *msg = CanRxQueue[CanRxTail];

    CanRxTail =
        (CanRxTail + 1) % CAN_RX_QUEUE_SIZE;

    return 1;
}
void ADC_Update(uint16_t ADC_Array[])
{
    for (int i = 0; i < 2; i++)
    {
        // Set mux state
        HAL_GPIO_WritePin(GPIOA,
                          GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6,
                          i ? GPIO_PIN_SET : GPIO_PIN_RESET);

        HAL_Delay(10); // let mux settle

        HAL_ADC_Start(&hadc1);

        for (int j = 0; j < 6; j++)
        {
            HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
            ADC_Array[j + i * 6] = HAL_ADC_GetValue(&hadc1);
        }

        HAL_ADC_Stop(&hadc1);
    }
}
uint16_t Max_Adc(uint16_t ADC_Array[])
{
    uint16_t max = ADC_Array[0];

    for(int i = 1; i < 12; i++)
    {
        if(ADC_Array[i] > max)
        {
            max = ADC_Array[i];
        }
    }

    return max;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	__HAL_RCC_CLEAR_RESET_FLAGS();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  TxHeader.DLC = 2; 			//CAN frame data length (up to 8 bytes)
  TxHeader.IDE = CAN_ID_STD; 	//CAN ID length (Standard or Extended)
  TxHeader.RTR = CAN_RTR_DATA; 	//Request or Data frame
  TxHeader.StdId = 0x123; 		//Device CAN ID

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }

  Filter.FilterIdHigh = (0x123 << 5);
//  if (HAL_CAN_Start(&hcan2) != HAL_OK)
//   {
//       Error_Handler();
//   }

//  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(
      &hcan1,
      CAN_IT_RX_FIFO0_MSG_PENDING |
      CAN_IT_ERROR |
      CAN_IT_BUSOFF |
      CAN_IT_LAST_ERROR_CODE |
      CAN_IT_ERROR_WARNING |
      CAN_IT_ERROR_PASSIVE);

//  HAL_CAN_ActivateNotification(
//      &hcan2,
//      CAN_IT_RX_FIFO0_MSG_PENDING |
//      CAN_IT_ERROR |
//      CAN_IT_BUSOFF |
//      CAN_IT_LAST_ERROR_CODE |
//      CAN_IT_ERROR_WARNING |
//      CAN_IT_ERROR_PASSIVE);
  uint32_t adcValue = 0;

  // Then enter while loop
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if (RxReady == 1)
//	  	  {
//	  		  /* Clear the message ready flag */
//	  		  RxReady = 0;
//
//	  		  /* Copy and switch data bytes from RxData to TxData */
//	  		  TxData[0] = RxData[1];
//	  		  TxData[1] = RxData[0];
//
//	  		  /* Echo the modified frame back to Kev */
//	  		  HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
//
//	  	  	 }

	  CAN_Message_t msg;

	  if(CAN_ReadMessage(&msg)) {
      // --- interrupt handler ---
      // --- code from before ---
      // TxData[0] = msg.data[1];
      // TxData[1] = msg.data[0];
      // //HAL_CAN_AddTxMessage(
      //   //&hcan2,
      //   //&TxHeader,
      //   //TxData,
      //   //&TxMailbox);
      // HAL_CAN_AddTxMessage(
      //   &hcan1,
      //   &TxHeader,
      //   TxData,
      //  &TxMailbox);

      for(int i = 0; i < 12; i += 4){

        TxData[0] = ADC_Array[i] & 0xFF;
        TxData[1] = (ADC_Array[i] >> 8) & 0x0F;

        TxData[2] = ADC_Array[i+1] & 0xFF;
        TxData[3] = (ADC_Array[i+1] >> 8) & 0x0F;

        TxData[4] = ADC_Array[i+2] & 0xFF;
        TxData[5] = (ADC_Array[i+2] >> 8) & 0x0F;

        TxData[6] = ADC_Array[i+3] & 0xFF;
        TxData[7] = (ADC_Array[i+3] >> 8) & 0x0F;   
        
        // currently a dummy as well, so that the 3 can messages can have different IDs
        TxHeader.DLC = 8;
        TxHeader.StdId = 0x200 + (i/4);

        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
      }
      // return to og value
      TxHeader.DLC = 2;
      TxHeader.StdId = 0x200;
	  }
	  HAL_Delay(100);

	  ADC_Update(ADC_Array);
	  max = Max_Adc(ADC_Array);
	  TxData[1] = adcValue & 0xFF;
    TxData[0] = (adcValue >> 8) & 0x0F;
	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

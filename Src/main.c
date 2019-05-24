/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f0xx_hal.h"
#include "spi.h"
#include "nrf24.h"

/* USER CODE BEGIN Includes */
//#define TRANSMITT_TEST    1
#define RF_CHANNEL              10
#define DEV_ADDRESS              0x3131

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/**
 * RF Packet description
 * 0 - Packet signature 0x55
 * 1 - Packet signature 0xA5
 * 2..3 - Device address
 * 4 - Number of bytes in payload
 * 5 - Message Counter in transmitter
 * 6 .. 29 Payload
 * 30 - Reserved
 * 31 - Checksum (Include header)- 1 byte
 */

int iValue;

uint8_t data_array[32 +8] = {0};
uint8_t tx_data_array[32] = {   0x55,0x5A,'D', 'V', '#', 'M', '1', '2',
                                '3', '4', '5', '6', '7', '8', '9', '0',
                                'R', 'E', 'S', 'P', 'O', 'N', 'S', 'E',
                                'i', 'g', 'k', 'l', 'm', '\r', 0 ,  0 };
//uint8_t* pRequest;
CMD_RF_Request* pRequest;

uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xEE,0xE7,0xE7,0xE7,0xE7};
//uint8_t rx_address[5] = {0x67,0x67,0x67,0x67,0x67};

uint8_t temp;
uint8_t q = 0;
uint8_t iCurrentCommand;

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
//  MX_ADC_Init();
//  MX_SPI1_Init();
//  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  initSPI(8, SPI_MODE_MASTER);
	high_voltage_valve(0);
	bridge_lever_A(0);
	bridge_lever_B(0);
	show_id_out(1);


  	initSPI(8, 0);
	nrf24_init();
/* Channel #2 , payload length: 4 */
	nrf24_config(RF_CHANNEL, 32);

/* Set the device addresses */
	nrf24_tx_address(tx_address);
	nrf24_rx_address(rx_address);

  /* USER CODE END 2 */
    show_id_out(0);
    delay(100);
    show_id_out(1);
    delay(100);
    show_id_out(0);
    delay(100);
    show_id_out(1);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        if (nrf24_dataReady()) {
            nrf24_getData(data_array);
            iValue = cmd_parse_request(data_array, &pRequest);
            if (0 == iValue && DEV_ADDRESS == pRequest->DevAddr) {
            	cmd_extract(pRequest, &iCurrentCommand);
                delay(2);
                update_response(pRequest);
                transmitt_response(pRequest);
                cmd_exec(&iCurrentCommand);
//                show_id_out(1);
            }
        }
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
//  if (HAL_SPI_Init(&hspi1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    hspi1.Lock = HAL_UNLOCKED;

//    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    HAL_SPI_MspInit(&hspi1);

//  hspi->State = HAL_SPI_STATE_BUSY;

//  /* Disable the selected SPI peripheral */
//  __HAL_SPI_DISABLE(hspi);

  /* Align by default the rs fifo threshold on the data size */
//    frxth = SPI_RXFIFO_THRESHOLD_HF;

  /* CRC calculation is valid only for 16Bit and 8 Bit */
//  if ((hspi->Init.DataSize != SPI_DATASIZE_16BIT) && (hspi->Init.DataSize != SPI_DATASIZE_8BIT))
//  {
    /* CRC must be disabled */
//    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  }


  /*----------------------- SPIx CR1 & CR2 Configuration ---------------------*/
  /* Configure : SPI Mode, Communication Mode, Clock polarity and phase, NSS management,
  Communication speed, First bit and CRC calculation state */
  WRITE_REG(hspi1.Instance->CR1, (hspi1.Init.Mode | hspi1.Init.Direction |
                                  hspi1.Init.CLKPolarity | hspi1.Init.CLKPhase |
								  (hspi1.Init.NSS & SPI_CR1_SSM) |
                                  hspi1.Init.BaudRatePrescaler | hspi1.Init.FirstBit  |
								  hspi1.Init.CRCCalculation));

  /* Configure : NSS management, TI Mode, NSS Pulse, Data size and Rx Fifo Threshold */
  WRITE_REG(hspi1.Instance->CR2, (((hspi1.Init.NSS >> 16U) & SPI_CR2_SSOE) | hspi1.Init.TIMode |
                                  hspi1.Init.NSSPMode | hspi1.Init.DataSize) |
		  	  	  	  	  	  	  SPI_RXFIFO_THRESHOLD_HF);



//  hspi1.ErrorCode = HAL_SPI_ERROR_NONE;
//  hspi1.State     = HAL_SPI_STATE_READY;

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, VALVE_BRIDGE_A_Pin|VALVE_BRIDGE_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, VALVE_HV_EN_Pin|NRF_CE_Pin|ID_SHOW_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VALVE_BRIDGE_A_Pin VALVE_BRIDGE_B_Pin */
  GPIO_InitStruct.Pin = VALVE_BRIDGE_A_Pin|VALVE_BRIDGE_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : VALVE_HV_EN_Pin NRF_CE_Pin ID_SHOW_OUT_Pin */
  GPIO_InitStruct.Pin = VALVE_HV_EN_Pin|NRF_CE_Pin|ID_SHOW_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_CSN_Pin */
  GPIO_InitStruct.Pin = NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_CSN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void delay(unsigned int iDelayValueMs) {
	unsigned int iCntA, iCntB;
	for (iCntB = 0; iCntB < iDelayValueMs; iCntB++) {
		for (iCntA = 0; iCntA < 5000; iCntA++)
			asm(" nop ");
	}
}
void delay_us(unsigned int iDelayValueUs) {
	unsigned int iCntB;
	iDelayValueUs >>= 1;
	for (iCntB = 0; iCntB < iDelayValueUs; iCntB++) {
			asm(" nop ");
	}
}

void nrf24_ce_digitalWrite(uint8_t state) {
   	HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, state);
}

void nrf24_csn_digitalWrite(uint8_t state) {
   	HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, state);
}

void high_voltage_valve(uint8_t iValue) {
   	HAL_GPIO_WritePin(VALVE_HV_EN_GPIO_Port, VALVE_HV_EN_Pin, iValue);
}

void bridge_lever_A(uint8_t iValue) {
   	HAL_GPIO_WritePin(VALVE_BRIDGE_A_GPIO_Port, VALVE_BRIDGE_A_Pin, iValue);
}

void bridge_lever_B(uint8_t iValue) {
    HAL_GPIO_WritePin(VALVE_BRIDGE_B_GPIO_Port, VALVE_BRIDGE_B_Pin, iValue);
}

void show_id_out(uint8_t iValue) {
    HAL_GPIO_WritePin(ID_SHOW_OUT_GPIO_Port, ID_SHOW_OUT_Pin, iValue);
}

void charge_delay() {
    int iCnt;

    for (iCnt = 0; iCnt < 6; iCnt++) {
        delay(25);
        show_id_out(0);
        delay(25);
        show_id_out(1);
    }
}

int transmitt_response(void* pResponseData) {
    int iReturnCode = 0;

    /* Automatically goes to TX mode */
    nrf24_send(pResponseData);
    /* Wait for transmission to end */
    while(nrf24_isSending())
        ;
    /* Make analysis on last tranmission attempt */
    temp = nrf24_lastMessageStatus();
    if (temp == NRF24_TRANSMISSON_OK) {
        asm ("nop ");
    } else if(temp == NRF24_MESSAGE_LOST) {
          asm ("nop ");
    }
    /* Retranmission count indicates the tranmission quality */
    temp = nrf24_retransmissionCount();
    /* Optionally, go back to RX mode ... */
    nrf24_powerUpRx();
    return iReturnCode;
}

uint16_t get_device_addr() {
    return DEV_ADDRESS;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

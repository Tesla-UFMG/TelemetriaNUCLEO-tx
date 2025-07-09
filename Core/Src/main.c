/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "subghz.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "radio_driver.h"
#include "stm32wlxx_nucleo.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// LoRa
#define RF_FREQUENCY                                868000000 /* Hz */
#define TX_OUTPUT_POWER                             14        /* dBm */
#define LORA_BANDWIDTH                              0         /* Hz */
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */
// i2c
#define I2C_SLAVE_ADDR   (0x3C << 1)   // 7-bit 0x3C -> 8-bit 0x78
#define RX_TIMEOUT_MS    1000          // Timeout de recepção
#define POLL_DELAY_MS    100           // Delay entre tentativas


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile bool txDone, txTimeout;
uint8_t txBuf[8] = {0x01,0x23,0x45,0x67, 0x89,0xAB,0xCD,0xEF};
uint8_t rxData[8];
const char *status;

uint8_t rxByte[8];

HAL_StatusTypeDef rxStatus;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void RadioOnDioIrq(RadioIrqMasks_t radioIrq);
void radioInit(void);

int __io_putchar(int ch);
int _write(int file, char *ptr, int len);
void _i2c1_receivemsg(uint8_t* buf, uint8_t size);
void dispBanner();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  char uartBuff[100];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIO Clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // DEBUG_SUBGHZSPI_{NSSOUT, SCKOUT, MSIOOUT, MOSIOUT} pins
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_SUBGHZSPI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // DEBUG_RF_{HSE32RDY, NRESET} pins
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Alternate = GPIO_AF13_DEBUG_RF;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // DEBUG_RF_{SMPSRDY, LDORDY} pins
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // RF_BUSY pin
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Alternate = GPIO_AF6_RF_BUSY;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // RF_{IRQ0, IRQ1, IRQ2} pins
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_8;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SUBGHZ_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  radioInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                         IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                         IRQ_RADIO_NONE,
                         IRQ_RADIO_NONE);

  // Inicia recepção IT
  HAL_SPI_Receive_IT(&hspi1, rxData, 8);

  while (1)
  {
	// Faz a leitura de dados por i2c
	_i2c1_receivemsg(txBuf, 8);

	txDone = txTimeout = false;
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
	SUBGRF_SendPayload(txBuf, sizeof(txBuf), 0);

	while (!txDone && !txTimeout);

	if (txTimeout) {
		status = "TIMEOUT";
	} else if (txDone) {
		status = "OK";
	} else {
		status = "DESCONHECIDO";
	}
	int len = 0;

	len += sprintf(uartBuff + len,
	 "\r\n\r\n"
	 "Status: %s\r\n"
	 "Payload (%d bytes): ",
	 status, (int)sizeof(txBuf));

	for (int i = 0; i < sizeof(txBuf); i++) {
	 len += sprintf(uartBuff + len, "%02X ", txBuf[i]);
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)uartBuff, len, HAL_MAX_DELAY);

	if (txDone) {
	 BSP_LED_Toggle(LED_GREEN);
	}

	HAL_Delay(POLL_DELAY_MS);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void _i2c1_receivemsg(uint8_t* buf, uint8_t size){
	/* Aguarda até 1 s por 1 byte vindo do mestre */
	  HAL_StatusTypeDef status = HAL_I2C_Slave_Receive(
			 &hi2c1,
			 buf,
			 size,
			 RX_TIMEOUT_MS
		   );

	if (status == HAL_OK)
	{
		/* Byte corretamente recebido */
		//printf("\r\nRecebido: ");
		//for(uint8_t i = 0; i < size; i++){
		//	printf("%d ", buf[i]);
		//}
		//BSP_LED_On(LED_GREEN);
		//HAL_Delay(300);
		//BSP_LED_Off(LED_GREEN);
	}
	else if (status == HAL_TIMEOUT)
	{
		/* Sem dado dentro do timeout – apenas aguarda e repete */
		// opcional: nada ou debug mínimo
	}
	else
	{
		/* Outro erro I2C */
		//printf("Erro na recepcao I2C (status = %d; endereco = %li)\r\n", status, hi2c1.Init.OwnAddress1);
		//BSP_LED_On(LED_RED);
		//HAL_Delay(300);
		//BSP_LED_Off(LED_RED);
		if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_AF))     /* NACK */
			__HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF);   /* limpa */

		/* reabilita o periférico se precisar */
		__HAL_I2C_DISABLE(&hi2c1);
		__HAL_I2C_ENABLE(&hi2c1);
	}
}

void dispBanner(){
	strcpy(uartBuff, "\n\n\n\rTELEMETRIA MASTER - FORMULA TESLA UFMG\r\nVERSAO=1.0\r\n---------------\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
	sprintf(uartBuff, "LORA_MODULATION\r\nLORA_BW=%d Hz\r\nLORA_SF=%d\r\nTX_OUTPUT_POWER= %d dBm", (1 << LORA_BANDWIDTH) * 125, LORA_SPREADING_FACTOR, TX_OUTPUT_POWER);
	HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
}

/**
  * @brief  Initialize the Sub-GHz radio and dependent hardware.
  * @retval None
  */
void radioInit(void)
{
  SUBGRF_Init(RadioOnDioIrq);

  SUBGRF_SetBufferBaseAddress(0x00, 0x00);

  SUBGRF_SetRfFrequency(RF_FREQUENCY);
  SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);

  SUBGRF_SetPacketType(PACKET_TYPE_LORA);

  ModulationParams_t modParams = {
    .PacketType = PACKET_TYPE_LORA,
    .Params.LoRa = {
      .Bandwidth           = LORA_BW_125,
      .CodingRate          = LORA_CR_4_5,
      .SpreadingFactor     = LORA_SF7,
      .LowDatarateOptimize = 0
    }
  };
  SUBGRF_SetModulationParams(&modParams);

  PacketParams_t pktParams = {
    .PacketType = PACKET_TYPE_LORA,
    .Params.LoRa = {
      .CrcMode        = LORA_CRC_ON,
      .HeaderType     = LORA_PACKET_VARIABLE_LENGTH,
      .InvertIQ       = LORA_IQ_NORMAL,
      .PayloadLength  = 0xFF,          // valor “max” genérico
      .PreambleLength = LORA_PREAMBLE_LENGTH
    }
  };
  SUBGRF_SetPacketParams(&pktParams);
}

/**
  * @brief  Receive data trough SUBGHZSPI peripheral
  * @param  radioIrq  interrupt pending status information
  * @retval None
  */
void RadioOnDioIrq(RadioIrqMasks_t irq)
{
  if(irq == IRQ_TX_DONE)        txDone    = true;
  else if(irq == IRQ_RX_TX_TIMEOUT) txTimeout = true;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1)
  {
    char uartBuff[64];
    uint16_t len;

    len = sprintf(uartBuff, "RxCplt: OK\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuff, len, HAL_MAX_DELAY);

    len = sprintf(uartBuff, "Received: ");
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuff, len, HAL_MAX_DELAY);

    for (int i = 0; i < 8; i++)
    {
      len = sprintf(uartBuff, "%u ", rxData[i]);
      HAL_UART_Transmit(&huart2, (uint8_t*)uartBuff, len, HAL_MAX_DELAY);
    }

    len = sprintf(uartBuff, "\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuff, len, HAL_MAX_DELAY);

    HAL_SPI_Receive_IT(&hspi1, rxData, 8);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi1)
  {
    char uartBuff[64];
    uint16_t len;

    len = sprintf(uartBuff, "SPI Error\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)uartBuff, len, HAL_MAX_DELAY);

    HAL_SPI_Receive_IT(&hspi1, rxData, 8);
  }
}

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

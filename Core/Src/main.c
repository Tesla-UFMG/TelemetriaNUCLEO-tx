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
void _i2c1_receive(uint8_t* buf, uint8_t size);

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

  printf("\n\n\n\rTELEMETRIA MASTER - FORMULA TESLA UFMG\r\nVERSAO=1.0\r\n---------------\r\n");
  printf("LORA_MODULATION\r\nLORA_BW=%d Hz\r\nLORA_SF=%d\r\nTX_OUTPUT_POWER= %d dBm", (1 << LORA_BANDWIDTH) * 125, LORA_SPREADING_FACTOR, TX_OUTPUT_POWER);

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
	/* Recebe pacote de dados da Datalogger via I2C1 */
	_i2c1_receive(txBuf, 8);

	/* Envia os dados lidos via LoRa para a placa receptora */
	txDone = txTimeout = false;
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
	SUBGRF_SendPayload(txBuf, sizeof(txBuf), 0);
	while (!txDone && !txTimeout);

	/* Verifica flags e define status da transmissao LoRa */
	if (txTimeout) {
		status = "TIMEOUT";
	} else if (txDone) {
		status = "OK";
	} else {
		status = "DESCONHECIDO";
	}
	printf("\r\n\r\nStatus: %s", status);

	/* Se o processo foi concluido com sucesso, imprime os dados recebidos
	 * e pisca o LED verde; Senao, pisca o LED vermelho */
	if (txDone) {
	  printf("\r\nPayload (%d bytes): ", (int)sizeof(txBuf));
	  for (int i = 0; i < sizeof(txBuf); i++) {
	    printf("%02X ", txBuf[i]);
	  }
	  BSP_LED_Off(LED_RED);
	  BSP_LED_Toggle(LED_GREEN);
	} else {
	  BSP_LED_Off(LED_RED);
	  BSP_LED_Toggle(LED_GREEN);
	}

	/* Pausa */
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

/* @brief Redireciona um caracter para escrita no monitor serial, via UART2
 * @param ch Caracter
 * @retval Caracter escrito
 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* @brief Redireciona um ponteiro oara vetor de caracteres
 * @param file Stream de destino
 * @param ptr Ponteiro para o vetor de caracteres
 * @param len Comprimento do vetor de caracteres
 * @retval Tamanho do vetor de caracteres redirecionado
 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* @brief Recebe e armazena dados atraves do I2C1
 * @param buf Buffer onde os dados sao armazenados
 * @param size Tamanho do buffer
 * @retval Nenhum
 */
void _i2c1_receive(uint8_t* buf, uint8_t size){
  HAL_StatusTypeDef status = HAL_I2C_Slave_Receive(&hi2c1, buf, size, RX_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_AF))
	  __HAL_I2C_CLEAR_FLAG(&hi2c1, I2C_FLAG_AF);
	__HAL_I2C_DISABLE(&hi2c1);
	__HAL_I2C_ENABLE(&hi2c1);
  }
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

/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "deca_device_api.h"
#include "deca_regs.h"
#include "dwm_platform.h"
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// DW1000 communication configuration - simple transmitter (example 01a)
static dwt_config_t configTransmitter = {
	2,               /* Channel number. */
	DWT_PRF_64M,     /* Pulse repetition frequency. */
	DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
	DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
	9,               /* TX preamble code. Used in TX only. */
	9,               /* RX preamble code. Used in RX only. */
	1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
	DWT_BR_110K,     /* Data rate. */
	DWT_PHRMODE_STD, /* PHY header mode. */
	(1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};
// DW1000 communication configuration - single-sided two-way-ranging responder (example 06b)
static dwt_config_t configResponder = {
	2,               /* Channel number. */
	DWT_PRF_64M,     /* Pulse repetition frequency. */
	DWT_PLEN_128,    /* Preamble length. Used in TX only. */
	DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
	9,               /* TX preamble code. Used in TX only. */
	9,               /* RX preamble code. Used in RX only. */
	0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
	DWT_BR_6M8,      /* Data rate. */
	DWT_PHRMODE_STD, /* PHY header mode. */
	(129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/**
 * The frame to send - simple transmitter, An 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *  - byte 0: frame type (0xC5 for a blink)
 *  - byte 1: sequence number, incremented for each new frame
 *  - byte 2 -> 9: device ID
 *  - byte 10/11: frame check-sum, automatically set by DW1000
 */
static uint8 messageTransmitter[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};
// Frames used in two-way-ranging responder mode
static uint8 messageResponderPoll[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 messageResponderTx[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Frame sequence number, incremented after each transmission.
static uint8 frameSequenceNumber = 0;

// Buffer to store received messages. Its size is adjusted to longest frame that this example code is supposed to handle.
static uint8 receiveBuffer[RX_BUF_LEN];

// Timestamps of frames transmission/reception. They are 40-bit wide - thus 64-bit type.
static uint64_t timestampPollRx;
static uint64_t timestampResponseTx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static uint64_t get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64_t ts);

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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(SPIx_CS_GPIO, SPIx_CS, GPIO_PIN_RESET);

	#if (DWM_TRANSMITTER==1) || (DWM_RESPONDER==1)
		// Reset and init DW1000.
		reset_DW1000();
		spi_set_rate_low();
		if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
			_Error_Handler(__FILE__, __LINE__);
		}
		spi_set_rate_high();

		// Configure DW1000
		#ifdef DWM_TRANSMITTER
			dwt_configure(&configTransmitter);
		#else
			dwt_configure(&configResponder);
		#endif

		// Apply default antenna delay value.
		dwt_setrxantennadelay(RX_ANT_DLY);
		dwt_settxantennadelay(TX_ANT_DLY);
	#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  #ifdef DWM_TRANSMITTER
  	// Write frame data to DW1000 and prepare transmission.
	// Zero offset in TX buffer.
	dwt_writetxdata(sizeof(messageTransmitter), messageTransmitter, 0);
	// Zero offset in TX buffer, no ranging.
	dwt_writetxfctrl(sizeof(messageTransmitter), 0, 0);
	// Start transmission.
	dwt_starttx(DWT_START_TX_IMMEDIATE);

	/* Poll DW1000 until TX frame sent event set.
	 * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register,
	 * we can use this simplest API function to access it.*/
	while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {};

	// Clear TX frame sent event.
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

	// Increment the blink frame sequence number (modulo 256).
	messageTransmitter[BLINK_FRAME_SN_IDX]++;

	// Blink the led
	HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);

	// Execute a delay between transmissions.
	sleep_ms(TX_DELAY_MS);
  #endif
  #ifdef DWM_RESPONDER
	// Activate reception immediately.
	dwt_rxenable(DWT_START_RX_IMMEDIATE);

	// Poll for reception of a frame or error/timeout.
	uint32 statusRegister = dwt_read32bitreg(SYS_STATUS_ID);
	while (!(statusRegister & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
		statusRegister = dwt_read32bitreg(SYS_STATUS_ID);
	}

	if (statusRegister & SYS_STATUS_RXFCG) {
		uint32 frameLength;

		// Clear good RX frame event in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		// A frame has been received, read it into the local buffer.
		frameLength = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frameLength <= RX_BUFFER_LEN) {
			dwt_readrxdata(receiveBuffer, frameLength, 0);
		}

		/* Check that the frame is a poll sent by "SS TWR initiator" example.
		 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
		receiveBuffer[ALL_MSG_SN_IDX] = 0;
		if (memcmp(receiveBuffer, messageResponderPoll, ALL_MSG_COMMON_LEN) == 0) {
			// Retrieve poll reception timestamp.
			timestampPollRx = get_rx_timestamp_u64();

			// Compute final message transmission time.
			uint32 transmissionTime = (timestampPollRx + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(transmissionTime);

			// Response TX timestamp is the transmission time we programmed plus the antenna delay.
			timestampResponseTx = (((uint64_t)(transmissionTime & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

			// Write all timestamps in the final message.
			resp_msg_set_ts(&messageResponderTx[RESP_MSG_POLL_RX_TS_IDX], timestampPollRx);
			resp_msg_set_ts(&messageResponderTx[RESP_MSG_RESP_TX_TS_IDX], timestampResponseTx);

			// Write and send the response message.
			messageResponderTx[ALL_MSG_SN_IDX] = frameSequenceNumber;
			dwt_writetxdata(sizeof(messageResponderTx), messageResponderTx, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(sizeof(messageResponderTx), 0, 1); /* Zero offset in TX buffer, ranging. */
			int transmitStatus = dwt_starttx(DWT_START_TX_DELAYED);

			// If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one.
			if (transmitStatus == DWT_SUCCESS) {
				// Poll DW1000 until TX frame sent event set.
				while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}

				// Clear TXFRS event.
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

				// Increment frame sequence number after transmission of the poll message (modulo 256).
				frameSequenceNumber++;
			}
		}
	} else {
		// Clear RX error events in the DW1000 status register.
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

		// Reset RX to properly reinitialise LDE operation.
		dwt_rxreset();
	}
  #endif
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 5, 0);
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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_ENABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
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
  HAL_GPIO_WritePin(DWM_RST_GPIO_Port, DWM_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DWM_RST_Pin */
  GPIO_InitStruct.Pin = DWM_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DWM_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DWM_IRQ_Pin */
  GPIO_InitStruct.Pin = DWM_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DWM_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USR_LED_Pin */
  GPIO_InitStruct.Pin = USR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USR_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 * @brief Get the RX time-stamp in a 64-bit variable.
 *  /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 * @param none
 * @return 64-bit value of the read time-stamp.
 */
static uint64_t get_rx_timestamp_u64(void) {
	uint8 ts_tab[5];
	uint64_t ts = 0;
	dwt_readrxtimestamp(ts_tab);
	for (int i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 * @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
 *  response message, the least significant byte is at the lower address.
 * @param ts_field pointer on the first byte of the timestamp field to fill ts timestamp value
 * @return none
 */
static void resp_msg_set_ts(uint8 *ts_field, const uint64_t ts) {
	for (int i = 0; i < RESP_MSG_TS_LEN; i++) {
		ts_field[i] = (ts >> (i * 8)) & 0xFF;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1) {
		HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
		HAL_Delay(66);
	}
  /* USER CODE END Error_Handler_Debug */
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

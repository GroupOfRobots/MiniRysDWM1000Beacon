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
#include "dwm_platform.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include <string.h>

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);

#define DWM_SIMPLE_TX
// #define DWM_SS_TWR_RESP

#ifdef DWM_SIMPLE_TX
/** Example 01a defines/statics **/

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
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

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID,  *     - byte 10/11: frame check-sum, automatically set by DW1000.  */
static uint8 tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

/** Example 01a defines/statics end **/
#endif

#ifdef DWM_SS_TWR_RESP
/** Example 06b defines/statics **/

/* Default communication configuration. We use here EVK1000's mode 4. */
static dwt_config_t config = {
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

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code,  */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 12
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 330

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* Declaration of static functions. */
static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);

/** Example 06b defines/statics end **/
#endif

int main(void) {
	/* MCU Configuration----------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	LL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_RTC_Init();

	#ifdef DWM_SIMPLE_TX
	/* Reset and initialise DW1000.
	 * For initialisation, DW1000 clocks must be temporarily set to crystal speed.
	 * After initialisation SPI rate can be increased for optimum performance. */
	reset_DW1000();
	spi_set_rate_low();
	if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
		while (1) {};
	}
	spi_set_rate_high();

	/* Configure DW1000. */
	dwt_configure(&config);

	LL_GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	#endif

	#ifdef DWM_SS_TWR_RESP
	/* Reset and initialise DW1000.
	 * For initialisation, DW1000 clocks must be temporarily set to crystal speed.
	 * After initialisation SPI rate can be increased for optimum performance. */
	reset_DW1000();
	spi_set_rate_low();
	if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR) {
		while (1) {};
	}
	spi_set_rate_high();
	/* Configure DW1000. */
	dwt_configure(&config);

	/* Apply default antenna delay value. */
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);
	#endif

	/* Infinite loop */
	while (1) {
		#ifdef DWM_SIMPLE_TX
		/* Write frame data to DW1000 and prepare transmission. */
		/* Zero offset in TX buffer. */
		dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
		/* Zero offset in TX buffer, no ranging. */
		dwt_writetxfctrl(sizeof(tx_msg), 0, 0);

		/* Start transmission. */
		dwt_starttx(DWT_START_TX_IMMEDIATE);

		/* Poll DW1000 until TX frame sent event set.
		 * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register,
		 * we can use this simplest API function to access it.*/
		while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {};

		if (tx_msg[BLINK_FRAME_SN_IDX] % 2) {
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3);
		} else {
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);
		}

		/* Clear TX frame sent event. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

		/* Execute a delay between transmissions. */
		sleep_ms(TX_DELAY_MS);

		/* Increment the blink frame sequence number (modulo 256). */
		tx_msg[BLINK_FRAME_SN_IDX]++;
		#endif

		#ifdef DWM_SS_TWR_RESP
		/* Activate reception immediately. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);

		/* Poll for reception of a frame or error/timeout. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {}

		if (status_reg & SYS_STATUS_RXFCG) {
			uint32 frame_len;

			/* Clear good RX frame event in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
			if (frame_len <= RX_BUFFER_LEN) {
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}

			/* Check that the frame is a poll sent by "SS TWR initiator" example.
			 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			rx_buffer[ALL_MSG_SN_IDX] = 0;
			if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {
				uint32 resp_tx_time;
				int ret;

				/* Retrieve poll reception timestamp. */
				poll_rx_ts = get_rx_timestamp_u64();

				/* Compute final message transmission time. */
				resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
				resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

				/* Write all timestamps in the final message. */
				resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
				resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

				/* Write and send the response message. */
				tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
				ret = dwt_starttx(DWT_START_TX_DELAYED);

				/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
				if (ret == DWT_SUCCESS) {
					/* Poll DW1000 until TX frame sent event set. */
					while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {}

					/* Clear TXFRS event. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

					/* Increment frame sequence number after transmission of the poll message (modulo 256). */
					frame_seq_nb++;
				}
			}
		} else {
			/* Clear RX error events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

			/* Reset RX to properly reinitialise LDE operation. */
			dwt_rxreset();
		}
		#endif
	}
}

static void LL_Init(void) {
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* System interrupt init*/
	/* MemoryManagement_IRQn interrupt configuration */
	NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	/* BusFault_IRQn interrupt configuration */
	NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	/* UsageFault_IRQn interrupt configuration */
	NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	/* SVCall_IRQn interrupt configuration */
	NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	/* DebugMonitor_IRQn interrupt configuration */
	NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	/* PendSV_IRQn interrupt configuration */
	NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	/* SysTick_IRQn interrupt configuration */
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
}

/* System Clock Configuration */
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

	if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
		Error_Handler();
	}

	LL_RCC_HSE_EnableBypass();

	LL_RCC_HSE_Enable();

	/* Wait till HSE is ready */
	while(LL_RCC_HSE_IsReady() != 1) {}

	LL_RCC_LSI_Enable();

	/* Wait till LSI is ready */
	while(LL_RCC_LSI_IsReady() != 1) {}

	LL_PWR_EnableBkUpAccess();

	LL_RCC_ForceBackupDomainReset();

	LL_RCC_ReleaseBackupDomainReset();

	LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);

	LL_RCC_EnableRTC();

	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);

	LL_RCC_PLL_Enable();

	/* Wait till PLL is ready */
	while(LL_RCC_PLL_IsReady() != 1) {}

	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);

	LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);

	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

	/* Wait till System clock is ready */
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {}

	LL_Init1msTick(72000000);

	LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

	LL_SetSystemCoreClock(72000000);

	/* SysTick_IRQn interrupt configuration */
	NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
}

/* RTC init function */
static void MX_RTC_Init(void) {
	LL_RTC_InitTypeDef RTC_InitStruct;

	/* Peripheral clock enable */
	LL_RCC_EnableRTC();

	/* Initialize RTC and set the Time and Date */
	RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
	RTC_InitStruct.AsynchPrescaler = 127;
	RTC_InitStruct.SynchPrescaler = 255;
	LL_RTC_Init(RTC, &RTC_InitStruct);

	LL_RTC_SetAsynchPrescaler(RTC, 127);

	LL_RTC_SetSynchPrescaler(RTC, 255);
}

/* SPI1 init function */
static void MX_SPI1_Init(void) {
	LL_SPI_InitTypeDef SPI_InitStruct;

	LL_GPIO_InitTypeDef GPIO_InitStruct;

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

	/** SPI1 GPIO Configuration
	 * PA5 ------> SPI1_SCK
	 * PA6 ------> SPI1_MISO
	 * PA7 ------> SPI1_MOSI
	 */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* SPI1 parameter configuration*/
	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
	/// TODO CHECK IF NEEDED
	// SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_ENABLE;
	SPI_InitStruct.CRCPoly = 7;
	LL_SPI_Init(SPI1, &SPI_InitStruct);

	LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);

	LL_SPI_SetCRCWidth(SPI1, LL_SPI_CRC_8BIT);

	LL_SPI_EnableNSSPulseMgt(SPI1);
}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 * Free pins are configured automatically as Analog (this feature is enabled through
 * the Code Generation settings)
 */
static void MX_GPIO_Init(void) {
	LL_GPIO_InitTypeDef GPIO_InitStruct;
	LL_EXTI_InitTypeDef EXTI_InitStruct;

	/* GPIO Ports Clock Enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	/**/
	LL_GPIO_ResetOutputPin(DWM_RST_GPIO_Port, DWM_RST_Pin);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_4|LL_GPIO_PIN_8
		|LL_GPIO_PIN_9|LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12
		|LL_GPIO_PIN_15;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = DWM_RST_Pin;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(DWM_RST_GPIO_Port, &GPIO_InitStruct);

	/**/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_3|LL_GPIO_PIN_4
		|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/**/
	LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE3);

	/**/
	LL_GPIO_SetPinPull(DWM_IRQ_GPIO_Port, DWM_IRQ_Pin, LL_GPIO_PULL_DOWN);

	/**/
	LL_GPIO_SetPinMode(DWM_IRQ_GPIO_Port, DWM_IRQ_Pin, LL_GPIO_MODE_INPUT);

	/**/
	EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_3;
	EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
	EXTI_InitStruct.LineCommand = ENABLE;
	EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
	LL_EXTI_Init(&EXTI_InitStruct);

	/* EXTI interrupt init*/
	NVIC_SetPriority(EXTI3_IRQn, 15);
	NVIC_EnableIRQ(EXTI3_IRQn);
}

/**
 * @brief This function is executed in case of error occurrence.
 * @param None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1) {}
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
void assert_failed(uint8_t* file, uint32_t line) {
	/* User can add his own implementation to report the file name and line number,
	 * ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
	 */
}

#endif

#ifdef DWM_SS_TWR_RESP
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readrxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
 *        response message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
	int i;
	for (i = 0; i < RESP_MSG_TS_LEN; i++) {
		ts_field[i] = (ts >> (i * 8)) & 0xFF;
	}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

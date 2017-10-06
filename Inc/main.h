/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define DWM_RST_Pin GPIO_PIN_2
#define DWM_RST_GPIO_Port GPIOA
#define DWM_IRQ_Pin GPIO_PIN_3
#define DWM_IRQ_GPIO_Port GPIOA
#define DWM_IRQ_EXTI_IRQn EXTI3_IRQn
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define USR_LED_Pin GPIO_PIN_3
#define USR_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/*** DWM defines ***/

// Example 01a (transmitter) defines
// Index to access to sequence number of the blink frame in the tx_msg array.
#define BLINK_FRAME_SN_IDX 1
// Inter-frame delay period, in milliseconds.
#define TX_DELAY_MS 1000

// Example 02a (receiver) defines
#define FRAME_LEN_MAX 127

// Example 06b defines
// Default antenna delay values for 64 MHz PRF.
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
// Length of the common part of the message (up to and including the function code,
#define ALL_MSG_COMMON_LEN 10
// Index to access some of the fields in the frames involved in the process.
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
// Receive buffer length
#define RX_BUF_LEN 12
// UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
// 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu.
#define UUS_TO_DWT_TIME 65536
// Delay between frames, in UWB microseconds.
#define POLL_RX_TO_RESP_TX_DLY_UUS 330

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */

/**
  * @}
*/

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

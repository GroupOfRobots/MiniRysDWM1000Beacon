/*! ----------------------------------------------------------------------------
 * @file	dwm_platform.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#ifndef _DWM_PLATFORM_H_
#define _DWM_PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

// #include "main.h"
#include "stm32f3xx_hal.h"

extern SPI_HandleTypeDef hspi1;

#define SPIx						SPI1
#define SPIx_GPIO					GPIOA
#define SPIx_CS						GPIO_PIN_4
#define SPIx_CS_GPIO				GPIOA
#define SPIx_SCK					GPIO_PIN_5
#define SPIx_MISO					GPIO_PIN_6
#define SPIx_MOSI					GPIO_PIN_7

#define DW1000_RSTn					DWM_RST_Pin
#define DW1000_RSTn_GPIO			DWM_RST_GPIO_Port

#define DECAIRQ						DWM_IRQ_Pin
#define DECAIRQ_GPIO				DWM_IRQ_GPIO_Port
#define DECAIRQ_EXTI_IRQn			DWM_IRQ_EXTI_IRQn

#define port_SPIx_busy_sending()		(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX)
#define port_SPIx_no_data()				(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY)

ITStatus EXTI_GetITEnStatus(uint32_t x);

#define port_GetEXT_IRQStatus()             EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()               NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()                NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
int NVIC_DisableDECAIRQ(void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_low()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_low (void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_high()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_high (void);

void reset_DW1000(void);
void setup_DW1000RSTnIRQ(int enable);

/*********/
/* sleep */
/*********/

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: sleep_ms()
 *
 * Wait for a given amount of time.
 * /!\ This implementation is designed for a single threaded application and is blocking.
 *
 * param  time_ms  time to wait in milliseconds
 */
void sleep_ms(unsigned int time_ms);


#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */

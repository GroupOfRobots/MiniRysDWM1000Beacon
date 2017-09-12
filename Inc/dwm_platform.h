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

#include "main.h"
// #include "stm32f3xx_hal_conf.h"

#define SPIx						SPI1
#define SPIx_GPIO					GPIOA
#define SPIx_CS						LL_GPIO_PIN_4
#define SPIx_CS_GPIO				GPIOA
#define SPIx_SCK					LL_GPIO_PIN_5
#define SPIx_MISO					LL_GPIO_PIN_6
#define SPIx_MOSI					LL_GPIO_PIN_7

#define DW1000_RSTn					LL_GPIO_PIN_2
#define DW1000_RSTn_GPIO			GPIOA

#define DECAIRQ						LL_GPIO_PIN_3
#define DECAIRQ_GPIO				GPIOA
#define DECAIRQ_EXTI				LL_SYSCFG_EXTI_LINE5
#define DECAIRQ_EXTI_PORT			LL_SYSCFG_EXTI_PORTB
#define DECAIRQ_EXTI_IRQn			EXTI9_5_IRQn

#define port_SPIx_busy_sending()		LL_SPI_IsActiveFlag_TXE(SPIx)
#define port_SPIx_no_data()				LL_SPI_IsActiveFlag_RXNE(SPIx)
#define port_SPIx_disable()				LL_SPI_Disable(SPIx)
#define port_SPIx_enable()              LL_SPI_Enable(SPIx)
#define port_SPIx_set_chip_select()		LL_GPIO_SetOutputPin(SPIx_CS_GPIO,SPIx_CS)
#define port_SPIx_clear_chip_select()	LL_GPIO_ResetOutputPin(SPIx_CS_GPIO,SPIx_CS)

ITStatus EXTI_GetITEnStatus(uint32_t x);

#define port_GetEXT_IRQStatus()             EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()               NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()                NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
#define port_CheckEXT_IRQ()                 LL_GPIO_IsInputPinSet(DECAIRQ_GPIO, DECAIRQ)
int NVIC_DisableDECAIRQ(void);

/* DW1000 IRQ (EXTI9_5_IRQ) */
/* handler type */
typedef void (*port_deca_isr_t)(void);
/* handler declaration. */
extern port_deca_isr_t port_deca_isr;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_deca_isr()
 *
 * @brief This function is used to install the handling function for DW1000 IRQ.
 *
 * NOTE:
 *   - As EXTI9_5_IRQHandler does not check that port_deca_isr is not null, the user application must ensure that a
 *     proper handler is set by calling this function before any DW1000 IRQ occurs!
 *   - This function makes sure the DW1000 IRQ line is deactivated while the handler is installed.
 *
 * @param deca_isr function pointer to DW1000 interrupt handler to install
 *
 * @return none
 */
void port_set_deca_isr(port_deca_isr_t deca_isr);

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

unsigned long portGetTickCnt(void);

#define portGetTickCount() portGetTickCnt()

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

/*! ----------------------------------------------------------------------------
 * @file	dwm_platform.c
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

#include "dwm_platform.h"
#include "deca_types.h"
#include "deca_device_api.h"

/**
 * @brief Checks whether the specified EXTI line is enabled or not.
 * @param EXTI_Line: specifies the EXTI line to check.
 *   This parameter can be:
 *     @arg EXTI_Linex: External interrupt line x where x(0..19)
 * @retval The "enable" state of EXTI_Line (SET or RESET).
 */
ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line) {
	if ((EXTI->IMR & EXTI_Line) != (uint32_t)RESET) {
		return SET;
	} else {
		return RESET;
	}
}

void spi_set_rate_low(void) {
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

void spi_set_rate_high(void) {
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

void reset_DW1000(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	// Make the RSTn pin an output and drive it low
	GPIO_InitStruct.Pin = DWM_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(DWM_RST_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(DW1000_RSTn_GPIO, DW1000_RSTn, GPIO_PIN_RESET);

	// Wait 2ms
	sleep_ms(2);

	// Switch the pin to high impendance - input/nopull
	// Drive the RSTn pin high
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(DWM_RST_GPIO_Port, &GPIO_InitStruct);

	// Wait 2ms
	sleep_ms(2);
}

/*******/
/* SPI */
/*******/

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodyLength, const uint8 *bodyBuffer) {
	decaIrqStatus_t stat;
	stat = decamutexon();

	uint8_t headBuf[headerLength];
	for (int i = 0; i < headerLength; ++i) {
		headBuf[i] = headerBuffer[i];
	}
	uint8_t bodyBuf[bodyLength];
	for (int i = 0; i < bodyLength; ++i) {
		bodyBuf[i] = bodyBuffer[i];
	}

	HAL_GPIO_WritePin(SPIx_CS_GPIO, SPIx_CS, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, headBuf, headerLength, 5);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY) {}

	HAL_SPI_Transmit(&hspi1, bodyBuf, bodyLength, 5);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY) {}

	HAL_GPIO_WritePin(SPIx_CS_GPIO, SPIx_CS, GPIO_PIN_SET);

	decamutexoff(stat);
	return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer) {
	decaIrqStatus_t stat;
	stat = decamutexon();

	uint8_t headBuf[headerLength];
	for (int i = 0; i < headerLength; ++i) {
		headBuf[i] = headerBuffer[i];
	}
	for (int i = 0; i < readlength; ++i) {
		readBuffer[i] = 0;
	}

	HAL_GPIO_WritePin(SPIx_CS_GPIO, SPIx_CS, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi1, headBuf, headerLength, 5);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY) {}

	HAL_SPI_Receive(&hspi1, readBuffer, readlength, 5);
	while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY) {}

	HAL_GPIO_WritePin(SPIx_CS_GPIO, SPIx_CS, GPIO_PIN_SET);

	decamutexoff(stat);
	return 0;
}

/*********/
/* sleep */
/*********/

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_sleep(unsigned int time_ms) {
	sleep_ms(time_ms);
}

void sleep_ms(unsigned int time_ms) {
	HAL_Delay(time_ms);
}

/*********/
/* mutex */
/*********/

// ---------------------------------------------------------------------------
//
// NB: The purpose of this file is to provide for microprocessor interrupt enable/disable, this is used for
//     controlling mutual exclusion from critical sections in the code where interrupts and background
//     processing may interact.  The code using this is kept to a minimum and the disabling time is also
//     kept to a minimum, so blanket interrupt disable may be the easiest way to provide this.  But at a
//     minimum those interrupts coming from the decawave device should be disabled/re-enabled by this activity.
//
//     In porting this to a particular microprocessor, the implementer may choose to use #defines in the
//     deca_irq.h include file to map these calls transparently to the target system.  Alternatively the
//     appropriate code may be embedded in the functions provided below.
//
//     This mutex dependent on HW port.
//	   If HW port uses EXT_IRQ line to receive ready/busy status from DW1000 then mutex should use this signal
//     If HW port not use EXT_IRQ line (i.e. SW polling) then no necessary for decamutex(on/off)
//
//	   For critical section use this mutex instead
//	   __save_intstate()
//     __restore_intstate()
// ---------------------------------------------------------------------------

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexon()
 *
 * Description: This function should disable interrupts. This is called at the start of a critical section
 * It returns the irq state before disable, this value is used to re-enable in decamutexoff call
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
decaIrqStatus_t decamutexon(void) {
	decaIrqStatus_t s = port_GetEXT_IRQStatus();

	// disable the external interrupt line
	if (s) {
		port_DisableEXT_IRQ();
	}

	// return state before disable, value is used to re-enable in decamutexoff call
	return s;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexoff()
 *
 * Description: This function should re-enable interrupts, or at least restore their state as returned(&saved) by decamutexon
 * This is called at the end of a critical section
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 * @param s - the state of the DW1000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
void decamutexoff(decaIrqStatus_t s) {
	// put a function here that re-enables the interrupt at the end of the critical section
	// need to check the port state as we can't use level sensitive interrupt on the STM ARM
	if (s) {
		port_EnableEXT_IRQ();
	}
}

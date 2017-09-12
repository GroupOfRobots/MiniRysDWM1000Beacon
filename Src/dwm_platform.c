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

#include "stm32_assert.h"
#include "deca_types.h"
#include "deca_device_api.h"

/* DW1000 IRQ handler definition. */
port_deca_isr_t port_deca_isr = NULL;

/* System tick 32 bit variable defined by the platform */
extern volatile unsigned long time32_incr;

unsigned long portGetTickCnt(void)
{
	return time32_incr;
}

/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line)
{
  ITStatus bitstatus = RESET;
  uint32_t enablestatus = 0;
  /* Check the parameters */
  assert_param(IS_GET_EXTI_LINE(EXTI_Line));

  enablestatus =  EXTI->IMR & EXTI_Line;
  if (enablestatus != (uint32_t)RESET)
  {
	bitstatus = SET;
  }
  else
  {
	bitstatus = RESET;
  }
  return bitstatus;
}

void port_set_deca_isr(port_deca_isr_t deca_isr)
{
	/* Check DW1000 IRQ activation status. */
	ITStatus en = port_GetEXT_IRQStatus();

	/* If needed, deactivate DW1000 IRQ during the installation of the new handler. */
	if (en)
	{
		port_DisableEXT_IRQ();
	}
	port_deca_isr = deca_isr;
	if (en)
	{
		port_EnableEXT_IRQ();
	}
}

void spi_set_rate_low (void)
{
	LL_SPI_SetBaudRatePrescaler(SPIx, LL_SPI_BAUDRATEPRESCALER_DIV32);
}

void spi_set_rate_high (void)
{
	LL_SPI_SetBaudRatePrescaler(SPIx, LL_SPI_BAUDRATEPRESCALER_DIV4);
}

void reset_DW1000(void)
{
	//drive the RSTn pin low
	LL_GPIO_ResetOutputPin(DW1000_RSTn_GPIO, DW1000_RSTn);

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
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
	int i=0;

	decaIrqStatus_t  stat;

	stat = decamutexon();

	SPIx_CS_GPIO->BRR = SPIx_CS;

	for(i=0; i<headerLength; i++) {
		SPIx->DR = headerBuffer[i];

		while (port_SPIx_no_data()) {}

		SPIx->DR;
	}

	for(i=0; i<bodylength; i++) {
		SPIx->DR = bodyBuffer[i];

		while (port_SPIx_no_data()) {}

		SPIx->DR;
	}

	SPIx_CS_GPIO->BSRR = SPIx_CS;

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
// #pragma GCC optimize ("O3")
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
	int i=0;

	decaIrqStatus_t stat;

	stat = decamutexon();

	/* Wait for SPIx Tx buffer empty */
	while (port_SPIx_busy_sending()) {}

	SPIx_CS_GPIO->BRR = SPIx_CS;

	for(i=0; i<headerLength; i++)
	{
		SPIx->DR = headerBuffer[i];

		while (port_SPIx_no_data()) {}

		// Dummy read as we write the header
		readBuffer[0] = SPIx->DR;
	}

	for(i=0; i<readlength; i++)
	{
		// Dummy write as we read the message body
		SPIx->DR = 0;

		while (port_SPIx_no_data()) {}

		//this clears RXNE bit
		readBuffer[i] = SPIx->DR;
	}

	SPIx_CS_GPIO->BSRR = SPIx_CS;

	decamutexoff(stat);

	return 0;
}

/*********/
/* sleep */
/*********/

/* Wrapper function to be used by decadriver. Declared in deca_device_api.h */
void deca_sleep(unsigned int time_ms)
{
	sleep_ms(time_ms);
}

void sleep_ms(unsigned int time_ms)
{
	/* This assumes that the tick has a period of exactly one millisecond. */
	unsigned long end = portGetTickCount() + time_ms;
	while ((signed long)(portGetTickCount() - end) <= 0)
	   ;
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
decaIrqStatus_t decamutexon(void)
{
	decaIrqStatus_t s = port_GetEXT_IRQStatus();

	if(s) {
		port_DisableEXT_IRQ(); //disable the external interrupt line
	}
	return s;   // return state before disable, value is used to re-enable in decamutexoff call
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
void decamutexoff(decaIrqStatus_t s)        // put a function here that re-enables the interrupt at the end of the critical section
{
	if(s) { //need to check the port state as we can't use level sensitive interrupt on the STM ARM
		port_EnableEXT_IRQ();
	}
}

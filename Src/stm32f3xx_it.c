/**
	******************************************************************************
	* @file    stm32f3xx_it.c
	* @brief   Interrupt Service Routines.
	******************************************************************************
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
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"

/* External variables --------------------------------------------------------*/
/* Tick timer count. */
volatile unsigned long time32_incr;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
	time32_incr++;
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line 3 interrupt.
* Unused in this code.
*/
void EXTI3_IRQHandler(void)
{
	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3) != RESET)
	{
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

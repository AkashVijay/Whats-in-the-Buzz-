/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */

#include "stm32f429i_discovery.h"
#include "arm_math.h"

double process(double); // fiview-generated bandpass filter
void graph_sample(float); // function to plot a sample point on the LCD
void graph_fft(void); // function to draw the FFT on the LCD

#define FFT_LEN 128

extern float32_t fft_input_buffer[FFT_LEN * 2];
extern float32_t fft_output_buffer[FFT_LEN];
extern uint8_t fft_ready;

#define FREQUENCIES 5

extern uint8_t frequency_index;
extern uint16_t frequency[];

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_dac2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  frequency_index++; // go to next frequency
  if(frequency_index >= FREQUENCIES) {
	  frequency_index = 0; // roll over
  }

  TIM6->ARR = (90000000 / 256) / frequency[frequency_index] - 1;

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream6 global interrupt.
*/
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dac2);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
* @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
*/
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

	float sample, filtered_sample;
	static uint16_t decimator_prescaler = 0; // prescaler for sample rate converter
	static uint16_t input_buffer_index = 0;

	  BSP_LED_On(LED3); // green LED on

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  sample = HAL_ADC_GetValue(&hadc1); // get converted value
  sample -= 2048.0; // normalize about zero
  sample /= 2048.0; // normalize to -1.0 to +1.0

  filtered_sample = process(sample); // apply bandpass filter

  //graph_sample(sample); // graph the incoming sample value
  graph_sample(filtered_sample); // graph the filtered sample value

  // drop 29/30 samples, effectively dropping sample rate from 48KHz to 1.6KHz

  decimator_prescaler++;
  if(decimator_prescaler >= 30) {

	  decimator_prescaler = 0; // reset prescaler



	  fft_input_buffer[input_buffer_index] = filtered_sample; // store filtered sample in FFT input buffer
	  input_buffer_index++; // increment input buffer index
	  fft_input_buffer[input_buffer_index] = 0; // imaginary component
	  input_buffer_index++; // increment input buffer index

	  if(input_buffer_index >= (2 * FFT_LEN)) {

		  input_buffer_index = 0; // reset input buffer index

		  //arm_cfft_f32(&arm_cfft_sR_f32_len128, fft_input_buffer, ifftFlag, doBitReverse); // perform FFT
		  //arm_cmplx_mag_f32(fft_input_buffer, fft_output_buffer, FFT_LEN);

		  //graph_fft(); // display the FFT
		  fft_ready = 1; // set flag to indicate that FFT is ready to be plotted
	  }
  }

  BSP_LED_Off(LED3); // green LED off

  /* USER CODE END ADC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

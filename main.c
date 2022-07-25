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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include <math.h> // sin()

#include "arm_math.h"
#include "arm_const_structs.h"
#include "stm32f429i_discovery_lcd.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac2;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint16_t sine[256]; // sine wave table

void draw_border(void);

#define FFT_LEN 128

float32_t fft_input_buffer[FFT_LEN * 2];
float32_t fft_output_buffer[FFT_LEN];

uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;

volatile uint8_t clear_flag = 0; // time to clear display area
volatile uint8_t fft_ready = 0; // FFT is ready to be plotted
volatile uint8_t stop_normal=0; //stop the normal routine
#define FREQUENCIES 5

uint8_t frequency_index = 0;
uint16_t frequency[FREQUENCIES] = {100, 200, 300, 400, 550 };

enum {
	STAT_MOSQUITO_1,
	STAT_MOSQUITO_2,
	STAT_MOSQUITO_3,
	STAT_MOSQUITO_4,
	STAT_MAX
};

struct {
	uint32_t samples;
	double sum_of_samples;
	double sum_of_squares;
	float max;

} stats[STAT_MAX];
volatile uint8_t mosquito_score_1 =0;
volatile uint8_t mosquito_score_2=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void stat_collect(uint16_t channel, float sample) {

	stats[channel].samples++; // count samples
	stats[channel].sum_of_samples += sample; // sum
	stats[channel].sum_of_squares += (sample * sample); // sum of squares
	if(sample > stats[channel].max) {
		stats[channel].max = sample; // new maximum
	}
}

float stat_mean(uint16_t channel) {

	// average = sum / samples

	return stats[channel].sum_of_samples / stats[channel].samples; // average
}

float stat_std_dev(uint16_t channel) {

	// sqrt((sumOfSquares - sum2 / blockSize) / (blockSize - 1))

	uint32_t samples = stats[channel].samples;
	double sum_of_samples = stats[channel].sum_of_samples;
	double sum_of_squares = stats[channel].sum_of_squares;

	double sum_squared = sum_of_samples * sum_of_samples;

	double variance = (sum_of_squares - sum_squared / samples) / (samples - 1);

	double std_dev = sqrt(variance);

	//return sqrt((stats[channel].sum_of_squares) - (stats[channel].sum_of_samples * stats[channel].sum_of_samples / stats[channel].samples) / (stats[channel].samples - 1));

	return std_dev;
}



void graph_sample(float sample) {

	// sample graph area (labeled 'Function Generator') on LCD is bounded by 16,16 and 223,143

	static float previous_sample = 0.0;
	static uint16_t x = 0;
	static uint8_t triggered = 0;
	int16_t y;

	if (stop_normal==1)return;


	//BSP_LCD_DrawPixel(223, 143, LCD_COLOR_GREEN); // *** debug *** test upper-left corner coordinates

	y = sample * 64; // scale sample value to pixels
	if(y > 63) y = 63; // clip max y
	if(y < -64) y = -64; // clip min y
	y += 64; // center y

	if(triggered == 0) {

		// not yet triggered

		if((previous_sample < 0.0) && sample >= 0.0 ) {
			triggered = 1; // trigger on rising edge
		} else {
			previous_sample = sample; // remember for next time
			return; // wait until triggered to plot
		}
	}

	BSP_LCD_DrawPixel(x + 16, 143 - y, LCD_COLOR_YELLOW);

	x++; // increment horizontal position
	if(x >= 207) {

		x = 0; // reset horizontal position
		triggered = 0; // reset triggered flag

		//BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		//BSP_LCD_FillRect(16, 16, 208, 128); // clear background

		clear_flag = 1; // set flag to clear display area
	}

	previous_sample = sample; // remember for next time
}

void graph_fft(void) {

	// FFT on the LCD (labeled 'Spectrogram') is bounded by 16,160 and 223,303

	//BSP_LCD_DrawPixel(16, 160, LCD_COLOR_GREEN); // *** debug *** test upper-left corner coordinates
	//BSP_LCD_DrawPixel(223, 303, LCD_COLOR_GREEN); // *** debug *** test lower-right corner coordinates}

	uint8_t x;
	uint16_t y;
	if (stop_normal==1)return;

	BSP_LCD_SetTextColor(LCD_COLOR_BLACK); // clear background
	BSP_LCD_FillRect(16, 160, 207, 143);

	//BSP_LCD_SetTextColor(LCD_COLOR_RED); // color for FFT

	for(x = 0; x < 63; x++) {

		y = fft_output_buffer[x] * 10; // scale value to LCD



	/*	if((x == 37)  || (x == 38) || (x == 39) || (x == 36)) {
			stat_collect(STAT_MOSQUITO_1, fft_output_buffer[x]);
			if (fft_output_buffer[x]>stat_mean(STAT_MOSQUITO_1) + stat_std_dev(STAT_MOSQUITO_1)/2){
				mosquito_score_1++;
			}else{
				if (mosquito_score_1){
					mosquito_score_1--;

				}
			}

			BSP_LCD_SetTextColor(LCD_COLOR_RED); // color for aedes albopictus

		}else*/


			if ((x==28)||(x==29) ||(x==30) ||(x==31)){
			stat_collect(STAT_MOSQUITO_2, fft_output_buffer[x]);
			if (fft_output_buffer[x]>stat_mean(STAT_MOSQUITO_2) + stat_std_dev(STAT_MOSQUITO_2)/4){
							mosquito_score_2++;
			}else{
				if (mosquito_score_2){
					mosquito_score_2--;
				}
			}

			BSP_LCD_SetTextColor(LCD_COLOR_BLUE); // color for fruit fly
		} else {
			BSP_LCD_SetTextColor(LCD_COLOR_YELLOW); // color for other frequencies
				}

		BSP_LCD_DrawLine(2 * x + 56, 300, 2 * x + 56, 300 - y); // draw line for FFT bin
		BSP_LCD_DrawLine(2 * x + 57, 300, 2 * x + 57, 300 - y); // draw line for FFT bin
	}


/*	if (mosquito_score_1>=12){
		stop_normal=1;
		BSP_LCD_Clear(LCD_COLOR_RED);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetBackColor(LCD_COLOR_RED);
		 BSP_LCD_DisplayStringAt(200,150, (uint8_t *) "  Aedes Albopictus",CENTER_MODE);
		HAL_Delay(3000);
		draw_border();//redraw the border
		stop_normal=0;
	}*/

	if (mosquito_score_2>=12){
			stop_normal=1;
			BSP_LCD_Clear(LCD_COLOR_BLUE);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
			BSP_LCD_SetFont(&Font12);
			BSP_LCD_DisplayStringAt(200,150, (uint8_t *) " Culex quinquefasciatus",RIGHT_MODE);
			HAL_Delay(3000);
			draw_border();//redraw the border
			stop_normal=0;
		}

}






void update_frequency(uint16_t frequency) {

	uint8_t f_string[10];

	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

//	sprintf((char *)f_string, "%i Hz", frequency);
	//BSP_LCD_SetFont(&Font12);
//	BSP_LCD_DisplayStringAt(24, 306, f_string, LEFT_MODE);
}


void draw_border(void){
	  BSP_LCD_SetTextColor(LCD_COLOR_WHITE); // border color
	  BSP_LCD_FillRect(0, 16, 16, 304); // left edge
	  BSP_LCD_FillRect(16, 304, 208, 16); // bottom edge
	  BSP_LCD_FillRect(224, 0, 16, 320); // right edge
	  BSP_LCD_SetFont(&Font16);
	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	  BSP_LCD_DisplayStringAtLine(0, (uint8_t*) "  Function Generator   "); // top edge
	  BSP_LCD_DisplayStringAtLine(9, (uint8_t *) "Frequency Spectrogram                   ");
//	  BSP_LCD_SetFont(&Font8);
	//  BSP_LCD_DisplayStringAt(30,142, (uint8_t *) "     Frequency               ", LEFT_MODE);      // new frequency

//	  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	//  BSP_LCD_DisplayStringAtLine(11, (uint8_t*) "     Spectrogram  "); //added line

	 // BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  BSP_LCD_SetFont(&Font8);
	  BSP_LCD_DisplayStringAt(55,304, (uint8_t *) "200",LEFT_MODE);
	  BSP_LCD_DisplayStringAt(98,304, (uint8_t *) "350-387.5",LEFT_MODE);
	  BSP_LCD_DisplayStringAt(171,304, (uint8_t *) "750",LEFT_MODE);
	  BSP_LCD_DisplayStringAt(70,312, (uint8_t *) "  Frequency(Hz)         ",LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,172, (uint8_t *) "R", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,182, (uint8_t *) "E", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,192, (uint8_t *) "L", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,212, (uint8_t *) "A", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,222, (uint8_t *) "M", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,232, (uint8_t *) "P", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,242, (uint8_t *) "L", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,252, (uint8_t *) "I", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,262, (uint8_t *) "T", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,272, (uint8_t *) "U", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,282, (uint8_t *) "D", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,292, (uint8_t *) "E", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,18, (uint8_t *) "R", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,28, (uint8_t *) "E", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,38, (uint8_t *) "L", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,56, (uint8_t *) "A", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,66, (uint8_t *) "M", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,76, (uint8_t *) "P", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,86, (uint8_t *) "L", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,96, (uint8_t *) "I", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,106, (uint8_t *) "T", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,116, (uint8_t *) "U", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,126, (uint8_t *) "D", LEFT_MODE);
	  BSP_LCD_DisplayStringAt(6,136, (uint8_t *) "E", LEFT_MODE);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

	uint32_t i = 0; // *** debug *** test variable

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // populate sine wave table

   double a;
   for(i = 0; i < 256; i++) {

 	  a = (double)i;
 	  a /= 256.0;
 	  a *= (2 * M_PI);
 	  sine[i] = sin(a) * 2048 + 2047;
   }

   // initialize statistics

   for(i = 0; i < STAT_MAX; i++) {
	   stats[i].samples = 0;
	   stats[i].sum_of_samples = 0.0;
	   stats[i].sum_of_squares = 0.0;
	   stats[i].max = 0.0;
   }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */



  // configure LCD

   BSP_LCD_Init();
   BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
   BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
   BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
   BSP_LCD_DisplayOn();
   BSP_LCD_Clear(LCD_COLOR_BLACK);
  // enable LEDs on board

  //BSP_LED_Init(LED3);
  //BSP_LED_Init(LED4);


  // draw border and fixed legends

draw_border();

  HAL_TIM_Base_Start(&htim6); // start TIM6 as function generator clock
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t *)sine, 256, DAC_ALIGN_12B_R);

  // to change output frequency, set TIM6->ARR to (90000000 / 256) / frequency - 1

  //uint32_t frequency = 200; // in Hz
  TIM6->ARR = (90000000 / 256) / frequency[frequency_index] - 1;
  update_frequency(frequency[frequency_index]); // display frequency on LCD

  HAL_TIM_Base_Start(&htim3); // start TIM3 as input sample rate clock
  HAL_ADC_Start_IT(&hadc1); // start ADC1 as input sampler with interrupts

  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  if(clear_flag) {

		  // time to clear waveform display area

		  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		  BSP_LCD_FillRect(16, 16, 208, 128); // clear background

		  clear_flag = 0; // reset flag
	  }

/*	  if(fft_ready) {

		  arm_cfft_f32(&arm_cfft_sR_f32_len128, fft_input_buffer, ifftFlag, doBitReverse); // perform FFT
		  arm_cmplx_mag_f32(fft_input_buffer, fft_output_buffer, FFT_LEN);

		  stat_collect(STAT_MOSQUITO_1, fft_output_buffer[37]); //525-600 HZ

		  graph_fft();

		  stat_collect(STAT_MOSQUITO_1, fft_output_buffer[38]);
		  stat_collect(STAT_MOSQUITO_1, fft_output_buffer[39]);
		  stat_collect(STAT_MOSQUITO_1, fft_output_buffer[36]);
		  fft_ready = 0; // reset flag
	  }*/
	  if(fft_ready) {

	  		  arm_cfft_f32(&arm_cfft_sR_f32_len128, fft_input_buffer, ifftFlag, doBitReverse); // perform FFT
	  		  arm_cmplx_mag_f32(fft_input_buffer, fft_output_buffer, FFT_LEN);

	  		  stat_collect(STAT_MOSQUITO_2, fft_output_buffer[28]); //350-387.5Hz

	  		  graph_fft();

	  		  stat_collect(STAT_MOSQUITO_2, fft_output_buffer[29]);

	  		  stat_collect(STAT_MOSQUITO_2, fft_output_buffer[30]);
	  		  stat_collect(STAT_MOSQUITO_2, fft_output_buffer[31]);
	  		  fft_ready = 0; // reset flag
	  }


	  i++; // *** debug *** increment test variable
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 432;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DMA2D init function */
static void MX_DMA2D_Init(void)
{

  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* LTDC init function */
static void MX_LTDC_Init(void)
{

  LTDC_LayerCfgTypeDef pLayerCfg;
  LTDC_LayerCfgTypeDef pLayerCfg1;

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 7;
  hltdc.Init.VerticalSync = 3;
  hltdc.Init.AccumulatedHBP = 14;
  hltdc.Init.AccumulatedVBP = 5;
  hltdc.Init.AccumulatedActiveW = 334;
  hltdc.Init.AccumulatedActiveH = 245;
  hltdc.Init.TotalWidth = 340;
  hltdc.Init.TotalHeigh = 247;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 0;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 0;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg1.Alpha = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1874;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 350;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}
/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_SDRAM_TimingTypeDef SdramTiming;

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
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
     PB12   ------> USB_OTG_HS_ID
     PB13   ------> USB_OTG_HS_VBUS
     PB14   ------> USB_OTG_HS_DM
     PB15   ------> USB_OTG_HS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

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
  while(1) 
  {
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

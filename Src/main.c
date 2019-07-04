/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void EMS_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
// PINA for Motor control
#define N_ENABLE GPIO_PIN_1
#define MS1     GPIO_PIN_2
#define MS2     GPIO_PIN_3
#define MS3     GPIO_PIN_4
#define STEP    GPIO_PIN_5
#define DIR     GPIO_PIN_6
// PINB for EMS22A
//#define PIN_DI GPIO_PIN_11
#define PIN_CLK	GPIO_PIN_10
#define PIN_DO 	GPIO_PIN_1
#define PIN_CS 	GPIO_PIN_0

extern uint8_t VCP_write(uint8_t* Buf, uint16_t Len);
extern int8_t VCP_read(uint8_t* Buf, uint32_t Len);

uint16_t read_from_EMS22()
{
	HAL_GPIO_WritePin(GPIOB, PIN_CS, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, PIN_CS, GPIO_PIN_RESET);
	bool read_bit[16];
	uint16_t pos = 0;
	uint8_t iter;
	//uint8_t parity = 0;

	for (iter = 0; iter < 16; iter++)
	{
		HAL_GPIO_WritePin(GPIOB, PIN_CLK, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, PIN_CLK, GPIO_PIN_SET);
		read_bit[iter] = HAL_GPIO_ReadPin(GPIOB, PIN_DO);

		//if (iter < 15)
		//	parity += read_bit[iter];
	}

	//parity %= 2;

	//if (read_bit[15] == parity)
	//{
	for (iter = 0; iter < 10; iter++)
	{
		//pos += read_bit[iter] * pow(2, 10 - (iter));
		pos += read_bit[iter] * (1 << (10 - iter));
	}
	HAL_GPIO_WritePin(GPIOB, PIN_CS, GPIO_PIN_RESET);
	return pos;
	//}

	//HAL_GPIO_WritePin(GPIOB, PIN_CS, GPIO_PIN_RESET);
	//VCP_write(" BAD ", 6);
	//return -1;
}

void writeIntro()
{
	VCP_write("\n\rWrite 1 for stepper motor driver ", 33);
	VCP_write("\n\rWrite 2 for encoder position ", 33);
	//VCP_write("\n\rN_ENABLE MS1 MS2 MS3 STEP DIR: ", 33);
}

void bitHandler(uint8_t bit, uint16_t GPIO_Pin)
{
	if(bit & 0x1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_Pin, GPIO_PIN_RESET);
	}
}

void stepHandler(uint8_t run, uint16_t step_cnt, uint16_t delay)
{
	int i = 0;

	if (run & 0x1) {
		for(i = 0; i < step_cnt; i++)
		{
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA, STEP, GPIO_PIN_SET);

			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOA, STEP, GPIO_PIN_RESET);

			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

			//HAL_Delay(delay);
		}
	}
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	uint8_t rx_buf[RX_BUF_SIZE];
	uint32_t rx_buf_len;
	char str[11];
	memset(str, 0, 11);

	memset(rx_buf, 0, RX_BUF_SIZE);

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	EMS_GPIO_Init();
	MX_USB_DEVICE_Init();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, N_ENABLE, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, MS1,      GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MS2, 	   GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, MS3,      GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, STEP, 	   GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, DIR,      GPIO_PIN_RESET);

	HAL_Delay(1000);

	while (rx_buf[0] != '0')
	{
		VCP_read(rx_buf, 1);
	}

	writeIntro();

	while (1)
	{
		//read_from_EMS22();
		///sprintf(str, "%d\n\r", read_from_EMS22());
		///VCP_write(str, 15);
		//VCP_write("END\n\r", 6);
		//VCP_write(read_from_EMS22(), 15);
		VCP_read(rx_buf, 1);

		if (rx_buf[0] != 0)
		{
			if (rx_buf[0] == '1')
			{
				VCP_write("\n\rN_ENABLE MS1 MS2 MS3 STEP DIR: ", 33);
				VCP_read(rx_buf, 6);

				if(rx_buf[0] != 0) {

					bitHandler(rx_buf[0], N_ENABLE);
					bitHandler(rx_buf[1], MS1);
					bitHandler(rx_buf[2], MS2);
					bitHandler(rx_buf[3], MS3);
					bitHandler(rx_buf[5], DIR);

					stepHandler(rx_buf[4], 200, 500);
					VCP_write("\n\rEncoder position: ", 20);
					sprintf(str, "\n\r%d", read_from_EMS22());
					VCP_write(str, 6);
				}
			}
			else if (rx_buf[0] == '2')
			{
				sprintf(str, "\n\r%d", read_from_EMS22());
				VCP_write(str, 6);
				//VCP_write("\n\rEncoder position: ", 33);
				//sprintf(str, "%d\n\r", read_from_EMS22());
				//VCP_write(str, 15);
			}

			writeIntro();
		}

		HAL_Delay(500);
	}

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 
                           PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

static void EMS_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = PIN_CS|PIN_CLK;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = PIN_DO;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOB, PIN_CLK, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, PIN_CS, GPIO_PIN_SET);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

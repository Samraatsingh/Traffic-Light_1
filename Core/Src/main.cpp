/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
extern "C"{
	#include "main.h"
	#include "gpio.h"
	#include "tim.h"
	#include "adc.h"
	#include "tm1637.h"
	#include "4x4_keypad.h"
	#include "lcd.h"
	#include <cstring>
	#include <stdlib.h>
	#include <stdio.h>
	#include "FreeRTOS.h"
	#include "task.h"
	#include "dht11.h"
}

#include "LDRSensor.hpp"
#include "LEDBarGraph.hpp"

TaskHandle_t passwordtask_t;
TaskHandle_t numbergenerator_t;

int disp_counter = 0;
int sens_counter = 0;

char random_num[5];				// array to store 4 digit number

Keypad_TypeDef keypad1 = {
    .row_ports = {GPIOA, GPIOA, GPIOA, GPIOA},
    .row_pins  = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11},
    .col_ports = {GPIOC, GPIOC, GPIOC, GPIOC},
    .col_pins  = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7}
};

TM1637_Display display = {
		.clk_port = tm1637_do_GPIO_Port,
		.clk_pin = tm1637_clk_Pin,
		.dio_port = tm1637_do_GPIO_Port,
		.dio_pin = tm1637_do_Pin,
		.brightness = 7
};

LdrSensor ldr(&hadc1, ADC_CHANNEL_4);

GPIO_TypeDef* led_ports[10] = {
		led_0_GPIO_Port, led_1_GPIO_Port, led_2_GPIO_Port, led_3_GPIO_Port, led_4_GPIO_Port,
		led_5_GPIO_Port, led_6_GPIO_Port, led_7_GPIO_Port, led_8_GPIO_Port, led_9_GPIO_Port
};

uint16_t led_pins[10] = {
		led_0_Pin, led_1_Pin, led_2_Pin, led_3_Pin, led_4_Pin,
		led_5_Pin, led_6_Pin, led_7_Pin, led_8_Pin, led_9_Pin
};

LEDBarGraph led_bargraph(led_ports, led_pins);

DHT11_t dht11;
DHT11_Status status;
const int temp_threshold = 25;
bool isTempHigh = false;
float temperature = 0;
float humidity = 0;
float light = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PasswordGenerator(void *param)
{
	TickType_t xLastWakeTime;
	const TickType_t _20sec = 20000 / portTICK_PERIOD_MS; // 20 seconds
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
	    int number = 1000 + (rand() % 9000);  	 // Range: 1000 to 9999
	    sprintf(random_num, "%04d", number);     // Format as a 4-digit string
	    TM1637_DisplayString(&display, random_num);
	    vTaskDelayUntil(&xLastWakeTime, _20sec);
	}
}



void SensorTask(void *parem)
{
	while(1)
	{
		sens_counter++;
		status = DHT11_Read(&dht11, &humidity, &temperature);
		light = ldr.getLightPercentage();

		if(temperature > temp_threshold)
		{
			isTempHigh = true;
		}else
		{
			isTempHigh = false;
		}



		vTaskDelay(pdMS_TO_TICKS(200));
	}

}

void ActuatorTask(void *param){
	while(1)
	{
		led_bargraph.setLevel(light);
	}
}

void DisplayTask(void *param)
{
    char upper_line[17];

	while(1)
	{
		disp_counter++;
		if (isTempHigh) {
			snprintf(upper_line, 17, "Mild Hot        ");
		}else
		{
			snprintf(upper_line, 17, "Cool            ");
		}
	    lprint(0x80, upper_line);

	    vTaskDelay(pdMS_TO_TICKS(500));
	}
;
}


void AuthenticatorTask(void *pv)
{
	char passkey[5] = {0};
//	const char password[] = "2255";

	LcdFxn(0, 0x01); // Clear screen
	lprint(0x80, "Welcome");

	while (1) {
		char key = Keypad_Scan(&keypad1);
		char temp_pass[5];
		taskENTER_CRITICAL(); // Disable context switch temporarily
		strcpy(temp_pass, random_num);
		taskEXIT_CRITICAL();

		if (key == '*') {
			LcdFxn(0, 0x01);
			lprint(0x80, "Enter Passkey:");
			memset(passkey, 0, sizeof(passkey));
			int i = 0;

			while (1) {
				key = Keypad_Scan(&keypad1);
				if (key) {
					if (key == '#') {
						passkey[i] = '\0';
						LcdFxn(0, 0x01);

						if (strcmp(passkey, temp_pass) == 0) {
							lprint(0x80, "Please come in");

							 TM1637_DisplayString(&display, "0000");

						} else {
							lprint(0x80, "Wrong");
						}

						vTaskDelay(pdMS_TO_TICKS(1000));
						LcdFxn(0, 0x01);
						lprint(0x80, "Welcome");
						break;
					}

					if (i < 4) {
						passkey[i++] = key;
						lprint(0xC0 + i - 1, "*");
					}
					while (Keypad_Scan(&keypad1)); // Wait for release
					vTaskDelay(pdMS_TO_TICKS(100)); // Debounce
				}

				vTaskDelay(pdMS_TO_TICKS(10));
			}
		}

		vTaskDelay(pdMS_TO_TICKS(50)); // Polling delay
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  Keypad_Init(&keypad1);
  TM1637_Init(&display);
  LcdInit();
  DHT11_Init(&dht11, dht_do_GPIO_Port, dht_do_Pin, &htim1);
  HAL_TIM_Base_Start(&htim1);



  // 2. Initialize the specific ADC channel for the LDR
  if (ldr.init() != HAL_OK) {
      Error_Handler();
  }

  /* USER CODE END 2 */
  srand(HAL_GetTick());
  xTaskCreate(PasswordGenerator, "PassGen", 128, NULL, 1, &passwordtask_t);
  xTaskCreate(AuthenticatorTask, "authenticator", 128, NULL, 1, &numbergenerator_t);
  xTaskCreate(SensorTask, "sensor task", 128, NULL, 3, NULL);
  xTaskCreate(ActuatorTask, "actuator task", 128, NULL, 2, NULL);
  xTaskCreate(DisplayTask, "display task", 128, NULL, 3, NULL);
  vTaskStartScheduler();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

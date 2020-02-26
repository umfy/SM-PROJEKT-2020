/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "eth.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bh1750.h"
#include "stdio.h"
#include "stdlib.h"
#include "String.h"
#include "arm_math.h"
#include "i2c-lcd.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* define PID parameters */
// Ku = 30 +  ; 7 15 0.1 ; 24 0.1 0 ; 32 6 0.2-2 ; 32 2 0.1
#define PID_PARAM_KP        30
#define PID_PARAM_KI        4
#define PID_PARAM_KD        0.1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
GPIO_PinState button_state, button_state_last;  // Actual and previous status of user button
int edge_detection;   // edge detection of the user button
float sensor_max; // Sensor maximal value
arm_pid_instance_f32 PID;  // PID instance

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/**
  * @brief  Displays data on LCD.
  * @param  sensor_value is current value read by the sensor.
  * @param  requested_value is reference value set by the user.
  * @retval None
  */
void Send_Output(float sensor_value, float requested_value)
{
	char sensor_value_local[16];
	char requested_value_local[16];

	sprintf(sensor_value_local, "%g", sensor_value);
	sprintf(requested_value_local, "%g", requested_value);

	lcd_clear ();
	lcd_send_cmd(0x80);
	lcd_send_string("VAL:");
	lcd_send_string((uint8_t*)sensor_value_local);
	lcd_send_string(" lux");

	lcd_put_cur(1, 0);
	lcd_send_string("REQ:");
	lcd_send_string((uint8_t*)requested_value_local);
	lcd_send_string(" lux");
	HAL_Delay(200);
}
/**
  * @brief  Button_Pressed activates when button is pressed in order to set the sensor_max value.
  * @retval None
  */
void Button_Pressed()
{
	 // set edge detection value
		  	  if((button_state == GPIO_PIN_SET)&&(button_state_last == GPIO_PIN_RESET))
		  	  {
		  		  edge_detection = 1;
		  	  }
		  	  else
		  	  {
		  		  edge_detection = 0;
		  	  }
		  	  button_state_last = button_state;


		  	  // set new maximal value of the sensor
		  	  if(edge_detection)
		  	  {
		  		sensor_max=Sensor_Range();
		  	  }
}
/**
  * @brief Sensor_Range sets the maximum value of a sensor as reference for PID.
  * It lights a diode with it's maximum power to measure the max range.
  * @retval max_value (Maximum value of the sensor in lux)
  */
float Sensor_Range()
{
	float max_value=0;
	TIM3->CCR1=100000;
	HAL_Delay(500);
	BH1750_ReadLight(&max_value);
	TIM3->CCR1=0;
	HAL_Delay(300);
	return max_value;
}

Calculate_PID(float sensor_value, float requested_value)
{
	float light_delta;     // Difference between requested value and actual value of the sensor
	int PWM_duty=10;       // PWM duty value
	float32_t PID_duty=0;  // PID duty value
	light_delta=requested_value-sensor_value;
	PID_duty=arm_pid_f32(&PID, light_delta);
	PWM_duty+=(int)(PID_duty*sensor_max/100000.0);// calculate PWM value
	// check if PWM duty is in range
	if(PWM_duty>100000){
		PWM_duty=100000;
	}
	else if(PWM_duty<0){
		PWM_duty=0;
	}
	// set calculated PWM value
	TIM3->CCR1=PWM_duty;
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t UartIn;  				// UART data receiving
	uint8_t UartOut; 				// UART data sending
	char terminal_setpoint[40];  	// UART receiving buffer
	uint8_t terminal_setpoint_size; // size of the UART message
	char buffer[40]; 				// UART sending buffer
	float sensor_value; 			// sensor actual value
	float requested_value=0; 		// Sensor requested value
	/* Rewrite defined PID parameters to PID instance */
	PID.Kp = PID_PARAM_KP;
	PID.Ki = PID_PARAM_KI;
	PID.Kd = PID_PARAM_KD;

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* Start PWM */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* Initialize BH1750 sensor */
  BH1750_Init(&hi2c2);
  /* Sets resolution mode to continuous high */
  BH1750_SetMode(CONTINUOUS_HIGH_RES_MODE_2);
  /* Initialize LCD */
  lcd_init ();
  /* Initialize PID */
  arm_pid_init_f32(&PID, 1);
  /* Adjusts maximum range of sensor based on current light level in the room*/
  sensor_max=Sensor_Range();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	  button_state = HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin);// read button state
	  	  Button_Pressed();

	  	  BH1750_ReadLight(&sensor_value); // Reads the converted value and calculates the result.

	  	  // transmit actual sensor value over UART
	  	  terminal_setpoint_size=sprintf (buffer, "%3.2f\n\r", sensor_value);
	  	  UartOut = HAL_UART_Transmit_IT(&huart3,(uint8_t*) buffer, terminal_setpoint_size);


	  	  // calculate proper PID duty value
	  	  Calculate_PID(sensor_value, requested_value);


	  	  // send current and requested values to LCD
	  	  Send_Output(sensor_value, requested_value);


	  	  // receive wanted sensor value from UART
	  	  UartIn = HAL_UART_Receive_IT( &huart3, (uint8_t*)&terminal_setpoint, terminal_setpoint_size);
	  	  requested_value= atof(terminal_setpoint);

	  	  HAL_Delay(200);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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

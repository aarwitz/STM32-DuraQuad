/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "../Inc/bno055_stm32/bno055_stm32.h" 
#include "../Inc/stm32f4xx_hal_usart.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_GPIO_Port GPIOA  // Assuming the LED is connected to GPIO Port A
#define LED_Pin GPIO_PIN_5   // Assuming the LED is connected to Pin 5 of Port A

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// PID constants (tune these based on your system)
float Kp = 1.0;  // Proportional gain
float Ki = 0.1;  // Integral gain
float Kd = 0.01; // Derivative gain

// PID variables
float setpoint_pitch = 0.0;  // Desired pitch angle
float current_pitch = 0.0;   // Current pitch angle from sensor
float error = 0.0, prev_error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float control_output = 0.0;

// Timing variables for PID calculation
// uint32_t last_time = HAL_GetTick();
float dt = 0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (fd == 1 || fd == 2) {
    hstatus = HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return len;
    else
      return -1;
  }
  return -1;
}
/* USER CODE END 0 */

/* PID constants (tune these for your quadcopter) */
float Kp_roll = 1.0, Ki_roll = 0.1, Kd_roll = 0.01;
float Kp_pitch = 1.0, Ki_pitch = 0.1, Kd_pitch = 0.01;
float Kp_yaw = 1.0, Ki_yaw = 0.1, Kd_yaw = 0.01;

/* PID state variables */
float roll_setpoint = 0.0, pitch_setpoint = 0.0, yaw_setpoint = 0.0;
float roll_error = 0.0, pitch_error = 0.0, yaw_error = 0.0;
float roll_integral = 0.0, pitch_integral = 0.0, yaw_integral = 0.0;
float roll_derivative = 0.0, pitch_derivative = 0.0, yaw_derivative = 0.0;
float prev_roll_error = 0.0, prev_pitch_error = 0.0, prev_yaw_error = 0.0;

/* Motor control variables */
float motor1_output = 0.0, motor2_output = 0.0, motor3_output = 0.0, motor4_output = 0.0;

int main(void)
{
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();

    /* Initialize peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_I2C1_Init();

    /* Start PWM on all motor timers */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Motor 1
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Motor 2
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Motor 3
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Motor 4

    /* Initialize BNO055 sensor */
    bno055_assignI2C(&hi2c1);
    HAL_Delay(100);
    bno055_setup();
    HAL_Delay(100);
    bno055_setOperationModeNDOF();

    uint32_t last_time = HAL_GetTick();

    /* Infinite loop */
    while (1)
    {
        uint32_t now = HAL_GetTick();
        float dt = (now - last_time) / 1000.0f; // Time in seconds
        last_time = now;

        /* Read Euler angles from BNO055 sensor */
        bno055_vector_t v = bno055_getVectorEuler();
        float current_yaw = v.x;   // Roll angle
        float current_pitch = v.y;  // Pitch angle
        float current_roll = v.z;    // Yaw angle

        if (current_roll > 180.0f) {
            current_roll -= 360.0f;
        }

        if (current_yaw > 180.0f) {
            current_yaw -= 360.0f;
        }

        if (current_pitch > 180.0f) {
            current_pitch = 360.0f - current_pitch;
        }


        // i am holding it like:
        /* Board layout:

                +----------+
                |         *| RST   PITCH  ROLL    YAW
            ADR |*        *| SCL
            INT |*        *| SDA     ^            /->
            PS1 |*        *| GND     |            |
            PS0 |*        *| 3VO     Y    Z-->    \-X
                |         *| VIN
                +----------+

        */



        /* Roll PID calculations */
        roll_error = roll_setpoint - current_roll;
        roll_integral += roll_error * dt;
        roll_derivative = (roll_error - prev_roll_error) / dt;
        float roll_output = Kp_roll * roll_error + Ki_roll * roll_integral + Kd_roll * roll_derivative;
        prev_roll_error = roll_error;

        /* Pitch PID calculations */
        pitch_error = pitch_setpoint - current_pitch;
        pitch_integral += pitch_error * dt;
        pitch_derivative = (pitch_error - prev_pitch_error) / dt;
        float pitch_output = Kp_pitch * pitch_error + Ki_pitch * pitch_integral + Kd_pitch * pitch_derivative;
        prev_pitch_error = pitch_error;

        /* Yaw PID calculations */
        yaw_error = yaw_setpoint - current_yaw;
        yaw_integral += yaw_error * dt;
        yaw_derivative = (yaw_error - prev_yaw_error) / dt;
        float yaw_output = Kp_yaw * yaw_error + Ki_yaw * yaw_integral + Kd_yaw * yaw_derivative;
        prev_yaw_error = yaw_error;

        /* Calculate motor outputs */
        motor1_output = 50 + roll_output + pitch_output - yaw_output;
        motor2_output = 50 - roll_output + pitch_output + yaw_output;
        motor3_output = 50 + roll_output - pitch_output + yaw_output;
        motor4_output = 50 - roll_output - pitch_output - yaw_output;

        /* Ensure motor outputs are within valid range (e.g., 1000 to 2000 for ESCs) */
        motor1_output = motor1_output < 0 ? 0 : (motor1_output > 255 ? 255 : motor1_output);
        motor2_output = motor2_output < 0 ? 0 : (motor2_output > 255 ? 255 : motor2_output);
        motor3_output = motor3_output < 0 ? 0 : (motor3_output > 255 ? 255 : motor3_output);
        motor4_output = motor4_output < 0 ? 0 : (motor4_output > 255 ? 255 : motor4_output);

        /* Update PWM duty cycle to control motor speeds */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor1_output);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, motor2_output);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, motor3_output);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor4_output);

        /* Optional: Add a small delay to avoid too frequent updates */
        HAL_Delay(10);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

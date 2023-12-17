/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hw_define.h"
#include "err_code.h"
#include "hw_intf.h"
#include "base_control.h"
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
/* USER CODE BEGIN PFP */
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
    MX_I2C2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM13_Init();
    MX_TIM14_Init();
    /* USER CODE BEGIN 2 */
    base_control_imu_cfg_t imu_cfg = {
        .mpu6050_read_bytes = hw_intf_mpu6050_read_bytes,
        .mpu6050_write_bytes = hw_intf_mpu6050_write_bytes,
        .func_delay = HAL_Delay
    };
    base_control_imu_init(imu_cfg);

    base_control_imu_filter_cfg_t imu_filter_cfg = {
        .beta = DEFAULT_MADGWICK_BETA,
        .sample_freq = DEFAULT_MADGWICK_SAMPLE_FREQ
    };
    base_control_imu_filter_init(imu_filter_cfg);

    base_control_motor_cfg_t motor_cfg = {
        .leftmotor_dir = 0,
        .leftmotor_duty = 0,
        .leftmotor_freq_hz = 0,
        .leftmotor_set_pwm_duty = hw_intf_leftmotor_set_pwm_duty,
        .leftmotor_set_pwm_freq = hw_intf_leftmotor_set_pwm_freq,
        .leftmotor_start_pwm = hw_intf_leftmotor_start,
        .leftmotor_stop_pwm = hw_intf_leftmotor_stop,
        .leftmotor_set_dir = hw_intf_leftmotor_set_dir,
        .rightmotor_dir = 0,
        .rightmotor_duty = 0,
        .rightmotor_freq_hz = 0,
        .rightmotor_set_pwm_duty = hw_intf_rightmotor_set_pwm_duty,
        .rightmotor_set_pwm_freq = hw_intf_rightmotor_set_pwm_freq,
        .rightmotor_start_pwm = hw_intf_rightmotor_start,
        .rightmotor_stop_pwm = hw_intf_rightmotor_stop,
        .rightmotor_set_dir = hw_intf_rightmotor_set_dir
    };
    base_control_motor_init(motor_cfg);

    base_control_resolver_cfg_t resolver_cfg = {
        .left_resolver_max_reload = NUM_PULSE_PER_ROUND * MICROSTEP_DIV,
        .left_resolver_start = hw_intf_left_resolver_start,
        .left_resolver_stop = hw_intf_left_resolver_stop,
        .left_resolver_set_counter = hw_intf_left_resolver_set_counter,
        .left_resolver_get_counter = hw_intf_left_resolver_get_counter,
        .left_resolver_set_mode = hw_intf_left_resolver_set_mode,
        .right_resolver_max_reload = NUM_PULSE_PER_ROUND * MICROSTEP_DIV,
        .right_resolver_start = hw_intf_right_resolver_start,
        .right_resolver_stop = hw_intf_right_resolver_stop,
        .right_resolver_set_counter = hw_intf_right_resolver_set_counter,
        .right_resolver_get_counter = hw_intf_right_resolver_get_counter,
        .right_resolver_set_mode = hw_intf_right_resolver_set_mode
    };
    base_control_resolver_init(resolver_cfg);
    /* USER CODE END 2 */
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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

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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "err_code.h"
#include "BaseControl_HwIntf.h"
#include "Periph.h"
#include "BaseControl.h"
#include "BaseControl_Define.h"
#include "serial_log.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// #define USE_SERIAL_LOG
/* Time update index */
#define CONTROL_MOTOR_TIME_INDEX                0       /*!< Time index control motor */
#define CMD_VEL_PUBLISH_TIME_INDEX              1       /*!< Time index publish velocity */
#define DRIVE_INFORMATION_PUBLISH_TIME_INDEX    2       /*!< Time index publish drive information */
#define IMU_PUBLISH_TIME_INDEX                  3       /*!< Time index publish IMU information */
#define IMU_UPDATE_TIME_INDEX 					4		/*!< Time index update IMU */
#define LOG_PUBLISH_TIME_INDEX                  5       /*!< Time index publish log information */

/* Frequency of publish/subscribe */
#define CONTROL_MOTOR_SPEED_FREQUENCY          	10      /*!< Frequency in Hz to control motor */
#define CONTROL_MOTOR_TIMEOUT                  	500     /*!< Period in ms to check control motor timeout */
#define IMU_PUBLISH_FREQUENCY                  	15      /*!< Frequency in Hz to publish IMU information */
#define IMU_UPDATE_FREQQUENCY 					1000	/*!< Frequency in Hz to update IMU information */
#define CMD_VEL_PUBLISH_FREQUENCY              	5       /*!< Frequency in Hz to publish robot velocity */
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    	5       /*!< Frequency in Hz to publish drive information */
#define DEBUG_LOG_FREQUENCY                    	10      /*!< Frequency in Hz to send log debug messages */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t base_control_time_update[10];
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
	MX_DMA_Init();
	MX_I2C2_Init();
	MX_TIM2_Init();
	MX_TIM13_Init();
	MX_TIM14_Init();
	MX_UART4_Init();
	MX_USART3_UART_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	/* Initialize serial log */
#ifdef USE_SERIAL_LOG
	serial_log_function_set(hw_intf_log_func, HAL_GetTick);
#endif

	/* Initialize IMU */
	periph_imu_cfg_t imu_cfg = {
		.clksel = MPU6050_CLKSEL_X_GYRO_REF,
		.dlpf_cfg = MPU6050_44ACCEL_42GYRO_BW_HZ,
		.sleep_mode = MPU6050_DISABLE_SLEEP_MODE,
		.gfs_sel = MPU6050_GFS_SEL_2000,
		.afs_sel = MPU6050_AFS_SEL_8G,
		.i2c_send = hw_intf_mpu6050_write_bytes,
		.i2c_recv = hw_intf_mpu6050_read_bytes,
		.delay = HAL_Delay
	};
	periph_imu_init(imu_cfg);

	/* Initialize madgwick filter*/
	periph_imu_filter_cfg_t imu_filter_cfg = {
		.beta = DEFAULT_MADGWICK_BETA,
		.sample_freq = DEFAULT_MADGWICK_SAMPLE_FREQ
	};
	periph_imu_filter_init(imu_filter_cfg);

	/* Initialize step motor */
	periph_motor_cfg_t motor_cfg = {
		.leftmotor_dir = 0,
		.leftmotor_freq_hz = 0,
		.leftmotor_duty = 0,
		.leftmotor_set_pwm_duty = hw_intf_leftmotor_set_pwm_duty,
		.leftmotor_set_pwm_freq = hw_intf_leftmotor_set_pwm_freq,
		.leftmotor_start_pwm = hw_intf_leftmotor_start,
		.leftmotor_stop_pwm = hw_intf_leftmotor_stop,
		.leftmotor_set_dir = hw_intf_leftmotor_set_dir,
		.rightmotor_dir = 0,
		.rightmotor_freq_hz = 0,
		.rightmotor_duty = 0,
		.rightmotor_set_pwm_duty = hw_intf_rightmotor_set_pwm_duty,
		.rightmotor_set_pwm_freq = hw_intf_rightmotor_set_pwm_freq,
		.rightmotor_start_pwm = hw_intf_rightmotor_start,
		.rightmotor_stop_pwm = hw_intf_rightmotor_stop,
		.rightmotor_set_dir = hw_intf_rightmotor_set_dir
	};
	periph_motor_init(motor_cfg);

	/* Initialize encoder*/
	periph_encoder_cfg_t encoder_cfg = {
		.left_encoder_max_reload = NUM_PULSE_PER_ROUND * MICROSTEP_DIV,
		.left_encoder_start = hw_intf_left_encoder_start,
		.left_encoder_stop = hw_intf_left_encoder_stop,
		.left_encoder_set_counter = hw_intf_left_encoder_set_counter,
		.left_encoder_get_counter = hw_intf_left_encoder_get_counter,
		.left_encoder_set_mode = hw_intf_left_encoder_set_mode,
		.right_encoder_max_reload = NUM_PULSE_PER_ROUND * MICROSTEP_DIV,
		.right_encoder_start = hw_intf_right_encoder_start,
		.right_encoder_stop = hw_intf_right_encoder_stop,
		.right_encoder_set_counter = hw_intf_right_encoder_set_counter,
		.right_encoder_get_counter = hw_intf_right_encoder_get_counter,
		.right_encoder_set_mode = hw_intf_right_encoder_set_mode
	};
	periph_encoder_init(encoder_cfg);

	/* Initialize ROS*/
	base_control_ros_setup();

	/* Initialize base control */
	base_control_setup();
	/* USER CODE END 2 */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
		/* Update time counter */
		uint32_t t = HAL_GetTick();

		/* Update ROS time */
		base_control_update_time();

		/* Update variable */
		base_control_update_variable(base_control_connect_status());

		/* Update TF */
		base_control_update_tf_prefix(base_control_connect_status());

		/* Control motor*/
		if ((t - base_control_time_update[CONTROL_MOTOR_TIME_INDEX] >= 1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
		{
			base_control_update_goal_vel();
			if ((t - base_control_get_time_callback_cmdvel()) > CONTROL_MOTOR_TIMEOUT)
			{
				base_control_set_zero_vel();
			}
			else
			{
				base_control_set_goal_vel();
			}
			base_control_time_update[CONTROL_MOTOR_TIME_INDEX] = t;
		}

		/* Publish motor speed to "cmd_vel_motor" topic */
		if ((t - base_control_time_update[CMD_VEL_PUBLISH_TIME_INDEX]) >= (1000 / CMD_VEL_PUBLISH_FREQUENCY))
		{
			base_control_publish_cmdvel_from_motor_msg();
			base_control_time_update[CMD_VEL_PUBLISH_TIME_INDEX] = t;
		}

		/* Publish driver information */
		if ((t - base_control_time_update[DRIVE_INFORMATION_PUBLISH_TIME_INDEX]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
		{
			/* Publish Odom, TF and JointState, */
			base_control_publish_drive_info();
			base_control_time_update[DRIVE_INFORMATION_PUBLISH_TIME_INDEX] = t;
		}

		/* Update IMU */
		if ((t - base_control_time_update[IMU_UPDATE_TIME_INDEX]) >= (1000 / IMU_UPDATE_FREQQUENCY))
		{
			/* Publish Odom, TF and JointState, */
			base_control_update_imu();
			base_control_time_update[IMU_UPDATE_TIME_INDEX] = t;
		}

		/* Publish IMU to "imu" topic */
		if ((t - base_control_time_update[IMU_PUBLISH_TIME_INDEX]) >= (1000 / IMU_PUBLISH_FREQUENCY))
		{
			base_control_publish_imu_msg();
			base_control_time_update[IMU_PUBLISH_TIME_INDEX] = t;
		}

		/* Send log message */
		base_control_send_log_msg();

		/* Spin NodeHandle to keep synchorus */
		base_control_spin_once();

		/* Keep rosserial connection */
		base_control_wait_serial_link(base_control_connect_status());
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
#ifdef USE_FULL_ASSERT
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

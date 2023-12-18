#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "hw_intf.h"
#include "mpu6050/mpu6050_register.h"

#define HW_SERIAL_LOG_UART_HANDLE 		huart3

#define HW_IMU_I2C hi2c2

#define TIMER_MAX_RELOAD 				0xFFFF

#define HW_LEFTMOTOR_TIM_HANDLE 		htim13
#define HW_LEFTMOTOR_TIM 				TIM13
#define HW_LEFTMOTOR_TIM_CCR 			CCR1
#define HW_LEFTMOTOR_TIM_CHANNEL 		TIM_CHANNEL_1
#define HW_LEFTMOTOR_TIM_CLK_FREQ 		84000000
#define HW_LEFTMOTOR_GPIO 				GPIOA
#define HW_LEFTMOTOR_GPIO_PIN 			GPIO_PIN_3

#define HW_RIGHTMOTOR_TIM_HANDLE 		htim14
#define HW_RIGHTMOTOR_TIM 				TIM14
#define HW_RIGHTMOTOR_TIM_CCR 			CCR1
#define HW_RIGHTMOTOR_TIM_CHANNEL 		TIM_CHANNEL_1
#define HW_RIGHTMOTOR_TIM_CLK_FREQ 		84000000
#define HW_RIGHTMOTOR_GPIO 				GPIOA
#define HW_RIGHTMOTOR_GPIO_PIN 			GPIO_PIN_2

#define HW_LEFT_ENCODER_TIM_HANDLE 	htim2
#define HW_LEFT_ENCODER_TIM 			TIM2

#define HW_RIGHT_ENCODER_TIM_HANDLE 	htim1
#define HW_RIGHT_ENCODER_TIM 			TIM1

#define ENCODER_COUNTER_MODE_UP  		0
#define ENCODER_COUNTER_MODE_DOWN  	1

err_code_t hw_intf_mpu6050_read_bytes(uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
	uint8_t buffer[1];
	buffer[0] = reg_addr;

	HAL_I2C_Master_Transmit(&HW_IMU_I2C, MPU6050_ADDR, buffer, 1, timeout_ms);
	HAL_I2C_Master_Receive(&HW_IMU_I2C, MPU6050_ADDR, buf, len, timeout_ms);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_mpu6050_write_bytes(uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
	uint8_t buf_send[len + 1];
	buf_send[0] = reg_addr;
	for (uint8_t i = 0; i < len; i++)
	{
		buf_send[i + 1] = buf[i];
	}

	HAL_I2C_Master_Transmit(&HW_IMU_I2C, MPU6050_ADDR, buf_send, len + 1, timeout_ms);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_leftmotor_set_pwm_duty(float duty)
{
	/* Calculate PWM compare value */
	uint32_t compare_value;
	compare_value = duty * (HW_LEFTMOTOR_TIM_HANDLE.Instance->ARR) / 100;

	/* Configure PWM compare value */
	__HAL_TIM_SET_COMPARE(&HW_LEFTMOTOR_TIM_HANDLE, HW_LEFTMOTOR_TIM_CHANNEL, compare_value);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_leftmotor_set_pwm_freq(uint32_t freq)
{
	if (freq == 0)
	{
		__HAL_TIM_SET_AUTORELOAD(&HW_LEFTMOTOR_TIM_HANDLE, 0);
		__HAL_TIM_SET_PRESCALER(&HW_LEFTMOTOR_TIM_HANDLE, 0);
		__HAL_TIM_SET_COMPARE(&HW_LEFTMOTOR_TIM_HANDLE, HW_LEFTMOTOR_TIM_CHANNEL, 0);

		return ERR_CODE_SUCCESS;
	}

	/* Calculate Timer PWM parameters. When change timer period you also
	 * need to update timer compare value to keep duty cycle stable */
	uint32_t apb_freq = HW_LEFTMOTOR_TIM_CLK_FREQ;
	uint32_t conduct = (uint32_t) (apb_freq / freq);
	uint16_t timer_prescaler = conduct / TIMER_MAX_RELOAD + 1;
	uint16_t timer_period = (uint16_t)(conduct / (timer_prescaler + 1)) - 1;

	/* Configure Timer PWM parameters */
	__HAL_TIM_SET_AUTORELOAD(&HW_LEFTMOTOR_TIM_HANDLE, timer_period);
	__HAL_TIM_SET_PRESCALER(&HW_LEFTMOTOR_TIM_HANDLE, timer_prescaler);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_leftmotor_start(void)
{
	HAL_TIM_PWM_Start(&HW_LEFTMOTOR_TIM_HANDLE, HW_LEFTMOTOR_TIM_CHANNEL);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_leftmotor_stop(void)
{
	HAL_TIM_PWM_Stop(&HW_LEFTMOTOR_TIM_HANDLE, HW_LEFTMOTOR_TIM_CHANNEL);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_leftmotor_set_dir(uint8_t dir)
{
	HAL_GPIO_WritePin(HW_LEFTMOTOR_GPIO, HW_LEFTMOTOR_GPIO_PIN, dir);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_rightmotor_set_pwm_duty(float duty)
{
	/* Calculate PWM compare value */
	uint32_t compare_value;
	compare_value = duty * (HW_RIGHTMOTOR_TIM_HANDLE.Instance->ARR) / 100;

	/* Configure PWM compare value */
	__HAL_TIM_SET_COMPARE(&HW_RIGHTMOTOR_TIM_HANDLE, HW_RIGHTMOTOR_TIM_CHANNEL, compare_value);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_rightmotor_set_pwm_freq(uint32_t freq)
{
	if (freq == 0)
	{
		__HAL_TIM_SET_AUTORELOAD(&HW_RIGHTMOTOR_TIM_HANDLE, 0);
		__HAL_TIM_SET_PRESCALER(&HW_RIGHTMOTOR_TIM_HANDLE, 0);
		__HAL_TIM_SET_COMPARE(&HW_RIGHTMOTOR_TIM_HANDLE, HW_RIGHTMOTOR_TIM_CHANNEL, 0);

		return ERR_CODE_SUCCESS;
	}

	/* Calculate Timer PWM parameters. When change timer period you also
	 * need to update timer compare value to keep duty cycle stable */
	uint32_t apb_freq = HW_RIGHTMOTOR_TIM_CLK_FREQ;
	uint32_t conduct = (uint32_t) (apb_freq / freq);
	uint16_t timer_prescaler = conduct / TIMER_MAX_RELOAD + 1;
	uint16_t timer_period = (uint16_t)(conduct / (timer_prescaler + 1)) - 1;

	/* Configure Timer PWM parameters */
	__HAL_TIM_SET_AUTORELOAD(&HW_RIGHTMOTOR_TIM_HANDLE, timer_period);
	__HAL_TIM_SET_PRESCALER(&HW_RIGHTMOTOR_TIM_HANDLE, timer_prescaler);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_rightmotor_start(void)
{
	HAL_TIM_PWM_Start(&HW_RIGHTMOTOR_TIM_HANDLE, HW_RIGHTMOTOR_TIM_CHANNEL);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_rightmotor_stop(void)
{
	HAL_TIM_PWM_Stop(&HW_RIGHTMOTOR_TIM_HANDLE, HW_RIGHTMOTOR_TIM_CHANNEL);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_rightmotor_set_dir(uint8_t dir)
{
	HAL_GPIO_WritePin(HW_RIGHTMOTOR_GPIO, HW_RIGHTMOTOR_GPIO_PIN, dir);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_left_encoder_start(void)
{
	HAL_TIM_Base_Start(&HW_LEFT_ENCODER_TIM_HANDLE);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_left_encoder_stop(void)
{
	HAL_TIM_Base_Stop(&HW_LEFT_ENCODER_TIM_HANDLE);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_left_encoder_set_counter(uint32_t value)
{
	__HAL_TIM_SET_COUNTER(&HW_LEFT_ENCODER_TIM_HANDLE, value);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_left_encoder_get_counter(uint32_t *value)
{
	*value = __HAL_TIM_GET_COUNTER(&HW_LEFT_ENCODER_TIM_HANDLE);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_left_encoder_set_mode(uint8_t mode)
{
	/* Reconfigure timer init parameters */
	HW_LEFT_ENCODER_TIM_HANDLE.Instance                 = HW_LEFT_ENCODER_TIM;
	HW_LEFT_ENCODER_TIM_HANDLE.Init.Prescaler           = 0;
	if (mode == ENCODER_COUNTER_MODE_UP) {
		HW_LEFT_ENCODER_TIM_HANDLE.Init.CounterMode         = TIM_COUNTERMODE_UP;
	} else {
		HW_LEFT_ENCODER_TIM_HANDLE.Init.CounterMode         = TIM_COUNTERMODE_DOWN;
	}
	HW_LEFT_ENCODER_TIM_HANDLE.Init.CounterMode         = TIM_COUNTERMODE_UP;
	HW_LEFT_ENCODER_TIM_HANDLE.Init.Period              = __HAL_TIM_GET_AUTORELOAD(&HW_LEFT_ENCODER_TIM_HANDLE);
	HW_LEFT_ENCODER_TIM_HANDLE.Init.ClockDivision       = TIM_CLOCKDIVISION_DIV1;
	HW_LEFT_ENCODER_TIM_HANDLE.Init.AutoReloadPreload   = TIM_AUTORELOAD_PRELOAD_DISABLE;

	/* Keep last counter value */
	uint32_t last_counter_val = __HAL_TIM_GET_COUNTER(&HW_LEFT_ENCODER_TIM_HANDLE);

	/* Set timer counter mode */
	HAL_TIM_Base_Init(&HW_LEFT_ENCODER_TIM_HANDLE);

	/* Set timer last counter value */
	__HAL_TIM_SET_COUNTER(&HW_LEFT_ENCODER_TIM_HANDLE, last_counter_val);

	return ERR_CODE_SUCCESS;
}


err_code_t hw_intf_right_encoder_start(void)
{
	HAL_TIM_Base_Start(&HW_RIGHT_ENCODER_TIM_HANDLE);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_right_encoder_stop(void)
{
	HAL_TIM_Base_Stop(&HW_RIGHT_ENCODER_TIM_HANDLE);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_right_encoder_set_counter(uint32_t value)
{
	__HAL_TIM_SET_COUNTER(&HW_RIGHT_ENCODER_TIM_HANDLE, value);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_right_encoder_get_counter(uint32_t *value)
{
	*value = __HAL_TIM_GET_COUNTER(&HW_RIGHT_ENCODER_TIM_HANDLE);

	return ERR_CODE_SUCCESS;
}

err_code_t hw_intf_right_encoder_set_mode(uint8_t mode)
{
	/* Reconfigure timer init parameters */
	HW_RIGHT_ENCODER_TIM_HANDLE.Instance                 = HW_RIGHT_ENCODER_TIM;
	HW_RIGHT_ENCODER_TIM_HANDLE.Init.Prescaler           = 0;
	if (mode == 0) {
		HW_RIGHT_ENCODER_TIM_HANDLE.Init.CounterMode         = TIM_COUNTERMODE_UP;
	} else {
		HW_RIGHT_ENCODER_TIM_HANDLE.Init.CounterMode         = TIM_COUNTERMODE_DOWN;
	}
	HW_RIGHT_ENCODER_TIM_HANDLE.Init.CounterMode         = TIM_COUNTERMODE_UP;
	HW_RIGHT_ENCODER_TIM_HANDLE.Init.Period              = __HAL_TIM_GET_AUTORELOAD(&HW_RIGHT_ENCODER_TIM_HANDLE);
	HW_RIGHT_ENCODER_TIM_HANDLE.Init.ClockDivision       = TIM_CLOCKDIVISION_DIV1;
	HW_RIGHT_ENCODER_TIM_HANDLE.Init.AutoReloadPreload   = TIM_AUTORELOAD_PRELOAD_DISABLE;

	/* Keep last counter value */
	uint32_t last_counter_val = __HAL_TIM_GET_COUNTER(&HW_RIGHT_ENCODER_TIM_HANDLE);

	/* Set timer counter mode */
	HAL_TIM_Base_Init(&HW_RIGHT_ENCODER_TIM_HANDLE);

	/* Set timer last counter value */
	__HAL_TIM_SET_COUNTER(&HW_RIGHT_ENCODER_TIM_HANDLE, last_counter_val);

	return ERR_CODE_SUCCESS;
}

void hw_intf_log_func(uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
	HAL_UART_Transmit(&HW_SERIAL_LOG_UART_HANDLE, (uint8_t*)data, len, timeout_ms);
}

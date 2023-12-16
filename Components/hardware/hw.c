#include "i2c.h"
#include "hw_intf.h"
#include "mpu6050/mpu6050_register.h"

#define HW_IMU_I2C hi2c2

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
#ifndef __HARDWARE_INTF_H__
#define __HARDWARE_INTF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

/*
 * @brief   MPU6050 read bytes function.
 *
 * @param   reg_addr Register address.
 * @param 	buf Buffer data.
 * @param 	len Length.
 * @param 	timeout_ms Timeout in miliseconds.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_mpu6050_read_bytes(uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms);

/*
 * @brief   MPU6050 write bytes function.
 *
 * @param   reg_addr Register address.
 * @param 	buf Buffer data.
 * @param 	len Length.
 * @param 	timeout_ms Timeout in miliseconds.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_mpu6050_write_bytes(uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms);


#ifdef __cplusplus
}
#endif

#endif /* __HARDWARE_INTF_H__ */
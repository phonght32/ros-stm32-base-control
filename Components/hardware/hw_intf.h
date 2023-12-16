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

/*
 * @brief   Set PWM duty cycle of left motor.
 *
 * @param   duty Duty cycle.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_leftmotor_set_pwm_duty(float duty);

/*
 * @brief   Set PWM frequency of left motor.
 *
 * @param   freq Frequency.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_leftmotor_set_pwm_freq(uint32_t freq);

/*
 * @brief   Start motor left.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_leftmotor_start(void);

/*
 * @brief   Stop motor left.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_leftmotor_stop(void);

/*
 * @brief   Set motor left direction.
 *
 * @param   dir Direction.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_leftmotor_set_dir(uint8_t dir);

/*
 * @brief   Set PWM duty cycle of right motor.
 *
 * @param   duty Duty cycle.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_rightmotor_set_pwm_duty(float duty);

/*
 * @brief   Set PWM frequency of right motor.
 *
 * @param   freq Frequency.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_rightmotor_set_pwm_freq(uint32_t freq);

/*
 * @brief   Start motor right.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_rightmotor_start(void);

/*
 * @brief   Stop motor right.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_rightmotor_stop(void);

/*
 * @brief   Set motor right direction.
 *
 * @param   dir Direction.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
err_code_t hw_intf_rightmotor_set_dir(uint8_t dir);


#ifdef __cplusplus
}
#endif

#endif /* __HARDWARE_INTF_H__ */
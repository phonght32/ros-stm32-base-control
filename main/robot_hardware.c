#include "robot_hardware.h"

#define MOTOR_INIT_ERR_STR              "motor init error"
#define IMU_INIT_ERR_STR                "imu init error"
#define RESOLVER_INIT_ERR_STR           "resolver init error"
#define MADGWICK_INIT_ERR_STR           "madgwick filter init error"

#define IMU_UPDATE_QUAT_ERR_STR         "imu update quaternion error"
#define IMU_GET_QUAT_ERR_STR            "imu get quaternion error"
#define IMU_GET_ACCEL_ERR_STR           "imu get accelerometer error"
#define IMU_GET_GYRO_ERR_STR            "imu get gyroscope error"

#define MOTORLEFT_START_ERR_STR         "motor left start error"
#define MOTORLEFT_STOP_ERR_STR          "motor left stop error"
#define MOTORLEFT_FORWARD_ERR_STR       "motor left forward error"
#define MOTORLEFT_BACKWARD_ERR_STR      "motor left backward error"
#define MOTORLEFT_SET_SPEED_ERR_STR     "motor left set speed error"
#define MOTORLEFT_GET_DIR_ERR_STR       "motor left get direction error"

#define MOTORRIGHT_START_ERR_STR        "motor right start error"
#define MOTORRIGHT_STOP_ERR_STR         "motor right stop error"
#define MOTORRIGHT_FORWARD_ERR_STR      "motor right forward error"
#define MOTORRIGHT_BACKWARD_ERR_STR     "motor right backward error"
#define MOTORRIGHT_SET_SPEED_ERR_STR    "motor right set speed error"
#define MOTORRIGHT_GET_DIR_ERR_STR      "motor right get direction error"

static const char* TAG = "ROBOT HARDWARE";
#define HARDWARE_CHECK(a, str, ret)  if(!(a)) {                                      \
        STM_LOGE(TAG,"%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str);        \
        return (ret);                                                                \
        }

stepmotor_handle_t motor_left, motor_right;
software_resolver_handle_t resolver_left, resolver_right;
mpu9250_handle_t mpu9250_handle;
ak8963_handle_t ak8963_handle;
madgwick_handle_t madgwick_handle;

stm_err_t robot_motor_init(void) 
{
    stepmotor_config_t motorleft_cfg;
    motorleft_cfg.dir_gpio_port = MOTORLEFT_DIR_GPIO_PORT;
    motorleft_cfg.dir_gpio_num = MOTORLEFT_DIR_GPIO_NUM;
    motorleft_cfg.pulse_timer_num = MOTORLEFT_PULSE_TIMER_NUM;
    motorleft_cfg.pulse_timer_pins_pack = MOTORLEFT_PULSE_TIMER_PINSPACK;
    motorleft_cfg.pulse_timer_chnl = MOTORLEFT_PULSE_TIMER_CHANNEL;
    motor_left = stepmotor_config(&motorleft_cfg);
    HARDWARE_CHECK(motor_left, MOTOR_INIT_ERR_STR, STM_FAIL);

    stepmotor_config_t motorright_cfg;
    motorright_cfg.dir_gpio_port = MOTORRIGHT_DIR_GPIO_PORT;
    motorright_cfg.dir_gpio_num = MOTORRIGHT_DIR_GPIO_NUM;
    motorright_cfg.pulse_timer_num = MOTORRIGHT_PULSE_TIMER_NUM;
    motorright_cfg.pulse_timer_pins_pack = MOTORRIGHT_PULSE_TIMER_PINSPACK;
    motorright_cfg.pulse_timer_chnl = MOTORRIGHT_PULSE_TIMER_CHANNEL;
    motor_right = stepmotor_config(&motorright_cfg);
    HARDWARE_CHECK(motor_right, MOTOR_INIT_ERR_STR, STM_FAIL);

    HARDWARE_CHECK(!stepmotor_set_pwm_duty(motor_left, STEP_DRIVER_PWM_DUTYCYCLE), MOTOR_INIT_ERR_STR, STM_FAIL);
    HARDWARE_CHECK(!stepmotor_set_pwm_freq(motor_left, 0), MOTOR_INIT_ERR_STR, STM_FAIL);
    HARDWARE_CHECK(!stepmotor_set_dir(motor_left, MOTORLEFT_DIR_FORWARD), MOTOR_INIT_ERR_STR, STM_FAIL);
    HARDWARE_CHECK(!stepmotor_start(motor_left), MOTOR_INIT_ERR_STR, STM_FAIL);

    HARDWARE_CHECK(!stepmotor_set_pwm_duty(motor_right, STEP_DRIVER_PWM_DUTYCYCLE), MOTOR_INIT_ERR_STR, STM_FAIL);
    HARDWARE_CHECK(!stepmotor_set_pwm_freq(motor_right, 0), MOTOR_INIT_ERR_STR, STM_FAIL);
    HARDWARE_CHECK(!stepmotor_set_dir(motor_right, MOTORRIGHT_DIR_FORWARD), MOTOR_INIT_ERR_STR, STM_FAIL);
    HARDWARE_CHECK(!stepmotor_start(motor_right), MOTOR_INIT_ERR_STR, STM_FAIL);

    STM_LOGD(TAG, "Configure motor success.");
    return STM_OK;
}

stm_err_t robot_imu_init(void)
{
    i2c_cfg_t i2c_cfg;
    i2c_cfg.i2c_num = IMU_I2C_NUM;
    i2c_cfg.i2c_pins_pack = IMU_I2C_PINSPACK;
    i2c_cfg.clk_speed = IMU_CLOCK_SPEED;
    HARDWARE_CHECK(!i2c_config(&i2c_cfg), IMU_INIT_ERR_STR, STM_FAIL);
        
    mpu9250_cfg_t mpu9250_cfg;
    mpu9250_cfg.afs_sel = MPU9250_AFS_RANGE;
    mpu9250_cfg.clksel = MPU9250_CLKSEL;
    mpu9250_cfg.dlpf_cfg =  MPU9250_DLPF;
    mpu9250_cfg.fs_sel = MPU9250_FS_RAGNE;
    mpu9250_cfg.sleep_mode = MPU9250_SLEEP_MODE;
    mpu9250_cfg.i2c_num = IMU_I2C_NUM;
    mpu9250_cfg.if_protocol = MPU9250_IF_PROTOCOL;
    mpu9250_handle = mpu9250_init(&mpu9250_cfg);
    HARDWARE_CHECK(mpu9250_handle, IMU_INIT_ERR_STR, STM_FAIL);

    ak8963_cfg_t ak8963_cfg;
    ak8963_cfg.opr_mode = AK8963_MODE;
    ak8963_cfg.mfs_sel = AK8963_RESOLUTION;
    ak8963_cfg.i2c_num = IMU_I2C_NUM;
    ak8963_cfg.if_protocol = AK8963_IF_PROTOCOL;
    ak8963_handle = ak8963_init(&ak8963_cfg);
    HARDWARE_CHECK(ak8963_handle, IMU_INIT_ERR_STR, STM_FAIL);

    mpu9250_auto_calib(mpu9250_handle);

    // ak8963_hard_iron_bias_t hard_iron_bias;
    // hard_iron_bias.x_axis = 75.807022;
    // hard_iron_bias.y_axis = 17.805250;
    // hard_iron_bias.z_axis = 23.861378;
    // ak8963_set_hard_iron_bias(ak8963_handle, hard_iron_bias);

    // ak8963_soft_iron_corr_t soft_iron_corr;
    // soft_iron_corr.x_axis = 1.004000;
    // soft_iron_corr.y_axis = 1.032922;
    // soft_iron_corr.z_axis = 0.965385;
    // ak8963_set_soft_iron_corr(ak8963_handle, soft_iron_corr);

    return STM_OK;
}

stm_err_t robot_madgwick_filter_init(void)
{
    madgwick_cfg_t madgwick_cfg;
    madgwick_cfg.beta = MADGWICK_BETA;
    madgwick_cfg.sample_freq = MADGWICK_SAMPLE_FREQ;
    madgwick_handle = madgwick_init(&madgwick_cfg);
    HARDWARE_CHECK(madgwick_handle, MADGWICK_INIT_ERR_STR, STM_FAIL);

    STM_LOGD(TAG, "Configure Madgwick filter success");
    return STM_OK;
}

stm_err_t robot_encoder_init(void)
{
    software_resolver_config_t resolver_left_cfg;
    resolver_left_cfg.timer_num = MOTORLEFT_TICK_TIMER_NUM;
    resolver_left_cfg.timer_pins_pack = MOTORLEFT_TICK_TIMER_PINSPACK;
    resolver_left_cfg.max_reload = NUM_PULSE_PER_ROUND * MICROSTEP_DIV;
    resolver_left_cfg.counter_mode = TIMER_COUNTER_UP;
    resolver_left = software_resolver_config(&resolver_left_cfg);

    software_resolver_config_t resolver_right_cfg;
    resolver_right_cfg.timer_num = MOTORRIGHT_TICK_TIMER_NUM;
    resolver_right_cfg.timer_pins_pack = MOTORRIGHT_TICK_TIMER_PINSPACK;
    resolver_right_cfg.max_reload = NUM_PULSE_PER_ROUND * MICROSTEP_DIV;
    resolver_right_cfg.counter_mode = TIMER_COUNTER_UP;
    resolver_right = software_resolver_config(&resolver_right_cfg);

    HARDWARE_CHECK(!software_resolver_start(resolver_left), RESOLVER_INIT_ERR_STR, STM_FAIL);
    HARDWARE_CHECK(!software_resolver_start(resolver_right), RESOLVER_INIT_ERR_STR, STM_FAIL);

    STM_LOGD(TAG, "Configure resolver success");
    return STM_OK;
}

stm_err_t robot_motor_left_start(void)
{
    HARDWARE_CHECK(!stepmotor_start(motor_left), MOTORLEFT_START_ERR_STR, STM_FAIL);

    return STM_OK;
}

stm_err_t robot_motor_left_stop(void)
{
    HARDWARE_CHECK(!stepmotor_stop(motor_left), MOTORLEFT_STOP_ERR_STR, STM_FAIL);

    return STM_OK;
}

stm_err_t robot_motor_left_set_speed(float speed)
{
    if (speed < 0)
    {
        HARDWARE_CHECK(!stepmotor_set_dir(motor_left, MOTORLEFT_DIR_BACKWARD), MOTORLEFT_SET_SPEED_ERR_STR, STM_FAIL);
        HARDWARE_CHECK(!software_resolver_set_mode(resolver_left, TIMER_COUNTER_DOWN), MOTORLEFT_SET_SPEED_ERR_STR, STM_FAIL);
        HARDWARE_CHECK(!stepmotor_set_pwm_freq(motor_left, (uint32_t)(-speed * VEL2FREQ)), MOTORLEFT_SET_SPEED_ERR_STR, STM_FAIL);
    }
    else
    {
        HARDWARE_CHECK(!stepmotor_set_dir(motor_left, MOTORLEFT_DIR_FORWARD), MOTORLEFT_SET_SPEED_ERR_STR, STM_FAIL);
        HARDWARE_CHECK(!software_resolver_set_mode(resolver_left, TIMER_COUNTER_UP), MOTORLEFT_SET_SPEED_ERR_STR, STM_FAIL);
        HARDWARE_CHECK(!stepmotor_set_pwm_freq(motor_left, (uint32_t)(speed * VEL2FREQ)), MOTORLEFT_SET_SPEED_ERR_STR, STM_FAIL);
    }

    return STM_OK;
}

stm_err_t robot_motor_right_start(void)
{
    HARDWARE_CHECK(!stepmotor_start(motor_right), MOTORRIGHT_START_ERR_STR, STM_FAIL);

    return STM_OK;
}

stm_err_t robot_motor_right_stop(void)
{
    HARDWARE_CHECK(!stepmotor_stop(motor_right), MOTORRIGHT_STOP_ERR_STR, STM_FAIL);

    return STM_OK;
}

stm_err_t robot_motor_right_set_speed(float speed)
{
    if (speed < 0)
    {
        HARDWARE_CHECK(!stepmotor_set_dir(motor_right, MOTORRIGHT_DIR_BACKWARD), MOTORRIGHT_SET_SPEED_ERR_STR, STM_FAIL);
        HARDWARE_CHECK(!software_resolver_set_mode(resolver_right, TIMER_COUNTER_DOWN), MOTORRIGHT_SET_SPEED_ERR_STR, STM_FAIL);
        HARDWARE_CHECK(!stepmotor_set_pwm_freq(motor_right, (uint32_t)(-speed * VEL2FREQ)), MOTORRIGHT_SET_SPEED_ERR_STR, STM_FAIL);
    }
    else
    {
        HARDWARE_CHECK(!stepmotor_set_dir(motor_right, MOTORRIGHT_DIR_FORWARD), MOTORRIGHT_SET_SPEED_ERR_STR, STM_FAIL);
        HARDWARE_CHECK(!software_resolver_set_mode(resolver_right, TIMER_COUNTER_UP), MOTORRIGHT_SET_SPEED_ERR_STR, STM_FAIL);
        HARDWARE_CHECK(!stepmotor_set_pwm_freq(motor_right, (uint32_t)(speed * VEL2FREQ)), MOTORRIGHT_SET_SPEED_ERR_STR, STM_FAIL);
    }

    return STM_OK;
}

stm_err_t robot_imu_update_quat(void)
{
    int ret;
    mpu9250_scale_data_t accel_scale, gyro_scale;
    ak8963_scale_data_t mag_scale;

    HARDWARE_CHECK(!mpu9250_get_accel_scale(mpu9250_handle, &accel_scale), IMU_UPDATE_QUAT_ERR_STR, STM_FAIL);
    HARDWARE_CHECK(!mpu9250_get_gyro_scale(mpu9250_handle, &gyro_scale), IMU_UPDATE_QUAT_ERR_STR, STM_FAIL);
    // HARDWARE_CHECK(!ak8963_get_mag_scale(ak8963_handle, &mag_scale), IMU_UPDATE_QUAT_ERR_STR, STM_FAIL);

    madgwick_update_6dof(madgwick_handle,
                         DEG2RAD(gyro_scale.x_axis),
                         DEG2RAD(gyro_scale.y_axis),
                         DEG2RAD(gyro_scale.z_axis),
                         accel_scale.x_axis,
                         accel_scale.y_axis,
                         accel_scale.z_axis);

    // madgwick_update_9dof(madgwick_handle,
    //                      DEG2RAD(gyro_scale.x_axis),
    //                      DEG2RAD(gyro_scale.y_axis),
    //                      DEG2RAD(gyro_scale.z_axis),
    //                      accel_scale.x_axis,
    //                      accel_scale.y_axis,
    //                      accel_scale.z_axis,
    //                      mag_scale.x_axis,
    //                      mag_scale.y_axis,
    //                      mag_scale.z_axis);

    return STM_OK;
}

stm_err_t robot_imu_get_quat(float *quat)
{
    madgwick_quat_data_t quat_data;
    madgwick_get_quaternion(madgwick_handle, &quat_data);

    quat[0] = quat_data.q0;
    quat[1] = quat_data.q1;
    quat[2] = quat_data.q2;
    quat[3] = quat_data.q3;

    return STM_OK;
}

stm_err_t robot_imu_get_accel(float *accel)
{
    mpu9250_scale_data_t accel_data;
    HARDWARE_CHECK(!mpu9250_get_accel_scale(mpu9250_handle, &accel_data), IMU_GET_ACCEL_ERR_STR, STM_FAIL);

    accel[0] = accel_data.x_axis;
    accel[1] = accel_data.y_axis;
    accel[2] = accel_data.z_axis;

    return STM_OK;
}

stm_err_t robot_imu_get_gyro(float *gyro)
{
    mpu9250_scale_data_t gyro_data;
    HARDWARE_CHECK(!mpu9250_get_gyro_scale(mpu9250_handle, &gyro_data), IMU_GET_GYRO_ERR_STR, STM_FAIL);

    gyro[0] = gyro_data.x_axis;
    gyro[1] = gyro_data.y_axis;
    gyro[2] = gyro_data.z_axis;

    return STM_OK;
}

stm_err_t robot_encoder_left_get_tick(int32_t *left_tick)
{
    uint32_t temp;
    software_resolver_get_value(resolver_left, &temp);
    software_resolver_set_value(resolver_left, NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2);
    *left_tick = temp - NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2;

    return STM_OK;
}

stm_err_t robot_encoder_right_get_tick(int32_t *right_tick)
{
    uint32_t temp;
    software_resolver_get_value(resolver_right, &temp);
    software_resolver_set_value(resolver_right, NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2);
    *right_tick = temp - NUM_PULSE_PER_ROUND * MICROSTEP_DIV / 2;
    return STM_OK;
}














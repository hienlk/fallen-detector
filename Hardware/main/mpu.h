
#ifndef MAIN_MPU_H_
#define MAIN_MPU_H_

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include "hal/mpu_types.h"
#include "math.h"
#include "mpu6050.h"  //add this component: idf.py add-dependency "espressif/mpu6050^1.2.0"
#include "soc/gpio_num.h"

#define RAD_TO_DEG 57.2958

/* I2C config */
#define I2C_MASTER_SCL_IO 9       
#define I2C_MASTER_SDA_IO 8      
#define I2C_MASTER_NUM I2C_NUM_0 
#define I2C_MASTER_FREQ_HZ 100000 

extern float angle;
extern float jerk_magnitude;
extern mpu6050_handle_t mpu6050;

extern mpu6050_temp_value_t temp;
extern mpu6050_acce_value_t acce;
extern mpu6050_gyro_value_t gyro;

void process_data(mpu6050_acce_value_t *acce_value,
                  mpu6050_gyro_value_t *gyro_value);
void read_data(mpu6050_handle_t mpu6050, mpu6050_acce_value_t *acce_value,
               mpu6050_gyro_value_t *gyro_value);
void read_temperature(mpu6050_handle_t mpu6050);
void i2c_sensor_mpu6050_init(void);
void is_fallen(float jerk, float angle);

#endif /* MAIN_MPU_H_ */

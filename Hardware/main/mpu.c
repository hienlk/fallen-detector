#include "mpu.h"
#include "server.h"

const float fall_threshold = 15000.0;  //Adjust this value to suit your needs
// const float angle_threshold = 30.0;
const int sample_interval = 10;
float prev_acc_x = 0.0, prev_acc_y = 0.0, prev_acc_z = 0.0;
float prev_angle_gyro = 0.0;
float jerk_magnitude = 0.0;
float angle_acc = 0.0;
float angle_gyro = 0.0;
float angle = 0.0;

/** 
 * 
 * ******** Configuration ********
 * 
*/
void i2c_bus_init(void) {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

  esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

  ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
  TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

void i2c_sensor_mpu6050_init(void) {
  esp_err_t ret;

  i2c_bus_init();
  mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
  TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create returned NULL");

  ret = mpu6050_config(mpu6050, ACCE_FS_2G, GYRO_FS_2000DPS);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  ret = mpu6050_wake_up(mpu6050);
  TEST_ASSERT_EQUAL(ESP_OK, ret);
}

/**
 * @brief Reads the temperature from the MPU6050 and prints it
 * @param[in] mpu6050 the handle to the MPU6050
 */
void read_temperature(mpu6050_handle_t mpu6050) {
  esp_err_t err;

  err = mpu6050_get_temp(mpu6050, &temp);

  if (err == ESP_OK) {
    // printf("Temperature: %.2f\n", temp.temp);
  } else {
    printf("Error reading temperature: %d\n", err);
  }
}

/**
 * @brief Reads accelerometer and gyroscope data from the MPU6050 sensor.
 *
 * @param[in] mpu6050 The handle to the MPU6050 sensor.
 * @param[out] acce_value Pointer to store the accelerometer measurements.
 * @param[out] gyro_value Pointer to store the gyroscope measurements.
 *
 * This function retrieves the current accelerometer and gyroscope data
 * from the MPU6050 sensor and updates the provided structures with the
 * measured values. In case of an error during reading, it outputs an
 * error message.
 */
void read_data(mpu6050_handle_t mpu6050, mpu6050_acce_value_t *acce_value,
               mpu6050_gyro_value_t *gyro_value) {

  esp_err_t err;

  err = mpu6050_get_acce(mpu6050, &acce);
  if (err == ESP_OK) {
    //    printf("Accel: X=%.2f, Y=%.2f, Z=%.2f (m/s^2)\n", acce_value->acce_x,
    //           acce_value->acce_y, acce_value->acce_z);
  } else {
    printf("Error reading accelerometer: %d\n", err);
  }

  err = mpu6050_get_gyro(mpu6050, &gyro);
  if (err == ESP_OK) {
    // printf("Gyro: X=%.2f, Y=%.2f, Z=%.2f (degrees/s)\n",
    // gyro_value->gyro_x,gyro_value->gyro_y, gyro_value->gyro_z);
  } else {
    printf("Error reading gyroscope: %d\n", err);
  }
}

/**
 * @brief Calculates the jerk magnitude, angle and filtered angle from the
 *        accelerometer and gyroscope data.
 *
 * @param[in] acce_value Pointer to the current accelerometer measurements.
 * @param[in] gyro_value Pointer to the current gyroscope measurements.
 *
 * This function calculates the jerk magnitude, angle and filtered angle from
 * the accelerometer and gyroscope data. The jerk magnitude is calculated from
 * the difference of consecutive accelerometer measurements. The angle is
 * calculated by combining the accelerometer and gyroscope measurements using a
 * complementary filter. The filtered angle is then updated with the new
 * measurements.
 */
void process_data(mpu6050_acce_value_t *acce_value,
                  mpu6050_gyro_value_t *gyro_value) {

  float acceleration_mg_x = acce_value->acce_x / 9.81 * 1000;
  float acceleration_mg_y = acce_value->acce_y / 9.81 * 1000;
  float acceleration_mg_z = acce_value->acce_z / 9.81 * 1000;

  float jerk_x = (acceleration_mg_x - prev_acc_x) / (sample_interval / 1000.0);
  float jerk_y = (acceleration_mg_y - prev_acc_y) / (sample_interval / 1000.0);
  float jerk_z = (acceleration_mg_z - prev_acc_z) / (sample_interval / 1000.0);

  jerk_magnitude = sqrt(jerk_x * jerk_x + jerk_y * jerk_y + jerk_z * jerk_z);

  prev_acc_x = acceleration_mg_x;
  prev_acc_y = acceleration_mg_y;
  prev_acc_z = acceleration_mg_z;

  angle_acc =
      atan2(acceleration_mg_y, sqrt(acceleration_mg_x * acceleration_mg_x +
                                    acceleration_mg_z * acceleration_mg_z)) *
      RAD_TO_DEG;

  float delta_time = sample_interval / 1000.0;
  angle_gyro += gyro_value->gyro_x * 0.01526 * delta_time;

  float alpha = 0.98;
  angle = alpha * angle_gyro + (1.0 - alpha) * angle_acc;
}

/**
 * @brief Turn on or off the buzzer.
 *
 * @param activate If true, turn on the buzzer. If false, turn off the buzzer.
 */
void buzzer(bool activate) { gpio_set_level(BUZZER_GPIO, activate ? 1 : 0); }

/**
 * @brief Turn on or off the LED.
 *
 * @param activate If true, turn on the LED. If false, turn off the LED.
 */
void blink_led(bool activate) { gpio_set_level(LED_GPIO, activate ? 1 : 0); }

/**
 * @brief Detects if a person has fallen.
 *
 * @param jerk The calculated jerk magnitude.
 * @param angle The calculated angle.
 *
 * This function detects if a person has fallen based on the jerk magnitude and
 * angle. If the jerk magnitude is greater or equal to the fall threshold, the
 * function sets the fall_detected and buzzer_led_active flags to true.
 * Otherwise, it sets the flags to false.
 */
void is_fallen(float jerk, float angle) {
  if (jerk_magnitude >= fall_threshold) {
    printf("Fallen detected!\n");
    fall_detected = true;
    buzzer_led_active = true;
  } else {
    fall_detected = false;
    //    buzzer_led_active = false;
  }
}
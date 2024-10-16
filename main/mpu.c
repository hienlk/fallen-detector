#include "stuff.h"

const float fall_threshold = 650000;
const float angle_threshold = 30.0;
const int sample_interval = 10;
float prev_acc_x = 0.0, prev_acc_y = 0.0, prev_acc_z = 0.0;
float prev_angle_gyro = 0.0;
// const int DOUBLE_PRESS_THRESHOLD = 1500;
float jerk_magnitude = 0.0;
float angle_acc = 0.0;
float angle_gyro = 0.0;
float angle = 0.0;

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

  ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
  TEST_ASSERT_EQUAL(ESP_OK, ret);

  ret = mpu6050_wake_up(mpu6050);
  TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void read_temperature(mpu6050_handle_t mpu6050) {
  esp_err_t err;

  err = mpu6050_get_temp(mpu6050, &temp);

  if (err == ESP_OK) {
    printf("Temperature: %.2f\n", temp.temp);
  } else {
    printf("Error reading temperature: %d\n", err);
  }
}

void read_data(mpu6050_handle_t mpu6050, mpu6050_acce_value_t *acce_value,
               mpu6050_gyro_value_t *gyro_value) {

  esp_err_t err;

  err = mpu6050_get_acce(mpu6050, &acce);
  if (err == ESP_OK) {
    // printf("Accel: X=%.2f, Y=%.2f, Z=%.2f (m/s^2)\n",
    // acce_value->acce_x,acce_value->acce_y, acce_value->acce_z);
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

void process_data(mpu6050_acce_value_t *acce_value,
                  mpu6050_gyro_value_t *gyro_value) {
  float acceleration_mg_x = acce_value->acce_x * 0.488;
  float acceleration_mg_y = acce_value->acce_y * 0.488;
  float acceleration_mg_z = acce_value->acce_z * 0.488;

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

void buzzer(bool activate) { gpio_set_level(BUZZER_GPIO, activate ? 1 : 0); }

void blink_led(bool activate) { gpio_set_level(LED_GPIO, activate ? 1 : 0); }

void is_fallen(float jerk, float angle) {
  if (jerk_magnitude > fall_threshold && fabs(angle) > angle_threshold) {
    printf("Fallen detected!\n");
    fall_detected = true;
    buzzer_led_active = true;
  } else {
    fall_detected = false;
    buzzer_led_active = false;
  }
}
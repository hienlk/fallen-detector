
#ifndef MAIN_STUFF_H_
#define MAIN_STUFF_H_

#include "console/console.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "esp_rom_gpio.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include "hal/mpu_types.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_sm.h"

#include "host/util/util.h"
#include "math.h"
#include "mpu6050.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "services/ans/ble_svc_ans.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "soc/gpio_num.h"
#include "unity.h"
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
/*


*/
extern mpu6050_handle_t mpu6050;

#define BUZZER_GPIO 4
#define LED_GPIO 5
#define CHECK_BUTTON 10
#define STOP_BUTTON 18
#define RAD_TO_DEG 57.2958

#define I2C_MASTER_SCL_IO 9       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 8       /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

extern uint16_t temp_char_handle;
extern uint16_t fall_char_handle;
extern float angle;
extern float jerk_magnitude;

extern bool fall_detected;
extern bool buzzer_led_active;
extern uint16_t conn_handle;

extern mpu6050_temp_value_t temp;
extern mpu6050_acce_value_t acce;
extern mpu6050_gyro_value_t gyro;

void ble_app_advertise(void);

int device_read_temp(uint16_t con_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg);
int device_read_fallen(uint16_t con_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg);

void blink_led(bool activate);
void buzzer(bool activate);
void i2c_sensor_mpu6050_init(void);
void is_fallen(float jerk, float angle);
void process_data(mpu6050_acce_value_t *acce_value,
                  mpu6050_gyro_value_t *gyro_value);
void read_data(mpu6050_handle_t mpu6050, mpu6050_acce_value_t *acce_value,
               mpu6050_gyro_value_t *gyro_value);
void read_temperature(mpu6050_handle_t mpu6050);
void gatt_server(void);
void host_task(void *param);
void ble_app_on_sync(void);
void send_notification(uint16_t attr_handle, void *data, size_t data_len);

#endif

#include "server.h"

bool fall_detected = false;
bool buzzerLedActive = false;
mpu6050_temp_value_t temp;
mpu6050_acce_value_t acce;
mpu6050_gyro_value_t gyro;
uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

char *TAG = "BLE-Test-Sever";
uint8_t ble_addr_type;
mpu6050_handle_t mpu6050 = NULL;
uint16_t fall_char_handle;
uint16_t temp_char_handle;

const float fallThreshold = 650000;
const float angleThreshold = 30.0;
const int sampleInterval = 10;
float prevAccX = 0.0, prevAccY = 0.0, prevAccZ = 0.0;
float prevAngleGyro = 0.0;
// const int DOUBLE_PRESS_THRESHOLD = 1500;
float jerkMagnitude = 0.0;
float angleAcc = 0.0;
float angleGyro = 0.0;
float angle = 0.0;

void ble_app_advertise(void);

int device_read_temp(uint16_t con_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg) {
  char sensor_data[20];
  sprintf(sensor_data, "Temp: %.2f", temp.temp);

  os_mbuf_append(ctxt->om, sensor_data, strlen(sensor_data));
  return 0;
}

int device_read_fallen(uint16_t con_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg) {
  char sensor_data[20];

  if (fall_detected) {
    sprintf(sensor_data, "Fallen");
  } else {
    sprintf(sensor_data, "0");
  }

  os_mbuf_append(ctxt->om, sensor_data, strlen(sensor_data));
  return 0;
}

int device_write(uint16_t conn_handle, uint16_t attr_handle,
                 struct ble_gatt_access_ctxt *ctxt, void *arg) {
  printf("Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
  return 0;
}

const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),
     .characteristics =
         (struct ble_gatt_chr_def[]){{
                                         .uuid = BLE_UUID16_DECLARE(0xFEF4),
                                         .flags = BLE_GATT_CHR_F_NOTIFY,
                                         .val_handle = &temp_char_handle,
                                         .access_cb = device_read_temp,
                                     },
                                     {
                                         .uuid = BLE_UUID16_DECLARE(0xFEF5),
                                         .flags = BLE_GATT_CHR_F_NOTIFY,
                                         .val_handle = &fall_char_handle,
                                         .access_cb = device_read_fallen,
                                     },
                                     {
                                         .uuid = BLE_UUID16_DECLARE(0xDEAD),
                                         .flags = BLE_GATT_CHR_F_WRITE,
                                         .access_cb = device_write,
                                     },
                                     {0}}},
    {0}};

int ble_gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    ESP_LOGI(TAG, "BLE GAP EVENT CONNECT %s",
             event->connect.status == 0 ? "OK" : "FAILED");
    if (event->connect.status == 0) {
      conn_handle = event->connect.conn_handle;
    }
    return 0;
  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "BLE GAP EVENT DISCONNECT");
    conn_handle = BLE_HS_CONN_HANDLE_NONE;
    ble_app_advertise();
    return 0;
  case BLE_GAP_EVENT_ADV_COMPLETE:
    ESP_LOGI(TAG, "BLE GAP EVENT ADV COMPLETE");
    ble_app_advertise();
    return 0;
  default:
    return 0;
  }
}

void ble_app_advertise(void) {
  struct ble_hs_adv_fields fields;
  const char *device_name = ble_svc_gap_device_name();
  memset(&fields, 0, sizeof(fields));
  fields.name = (uint8_t *)device_name;
  fields.name_len = strlen(device_name);
  fields.name_is_complete = 1;
  ble_gap_adv_set_fields(&fields);

  struct ble_gap_adv_params adv_params;
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                    ble_gap_event, NULL);
}

void ble_app_on_sync(void) {
  ble_hs_id_infer_auto(0, &ble_addr_type);
  ble_app_advertise();
}

void host_task(void *param) { nimble_port_run(); }

void send_notification(uint16_t attr_handle, void *data, size_t data_len) {
  if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, data_len);
    if (om) {
      ble_gatts_notify_custom(conn_handle, attr_handle, om);
    }
  }
}

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

  float jerkX = (acceleration_mg_x - prevAccX) / (sampleInterval / 1000.0);
  float jerkY = (acceleration_mg_y - prevAccY) / (sampleInterval / 1000.0);
  float jerkZ = (acceleration_mg_z - prevAccZ) / (sampleInterval / 1000.0);

  jerkMagnitude = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);

  prevAccX = acceleration_mg_x;
  prevAccY = acceleration_mg_y;
  prevAccZ = acceleration_mg_z;

  angleAcc =
      atan2(acceleration_mg_y, sqrt(acceleration_mg_x * acceleration_mg_x +
                                    acceleration_mg_z * acceleration_mg_z)) *
      RAD_TO_DEG;

  float deltaTime = sampleInterval / 1000.0;
  angleGyro += gyro_value->gyro_x * 0.01526 * deltaTime;

  float alpha = 0.98;
  angle = alpha * angleGyro + (1.0 - alpha) * angleAcc;
}

void buzzer(bool activate) { gpio_set_level(BUZZER_GPIO, activate ? 1 : 0); }

void blink_led(bool activate) { gpio_set_level(LED_GPIO, activate ? 1 : 0); }

void isFallen(float jerk, float angle) {
  if (jerkMagnitude > fallThreshold && fabs(angle) > angleThreshold) {
    printf("Fallen detected!\n");
    fall_detected = true;
    buzzerLedActive = true;
  } else {
    fall_detected = false;
    buzzerLedActive = false;
  }
}

void gatt_server(void) {
  nvs_flash_init();      // 1 - Initialize NVS flash using
  esp_nimble_hci_init(); // 2 - Initialize ESP controller
  nimble_port_init();    // 3 - Initialize the host stack
  ble_svc_gap_device_name_set(
      "BLE-Server");   // 4 - Initialize NimBLE conf iguration - server name
  ble_svc_gap_init();  // 4 - Initialize NimBLE configuration - gap service
  ble_svc_gatt_init(); // 4 - Initialize NimBLE configuration - gatt service
  ble_gatts_count_cfg(
      gatt_svcs); // 4 - Initialize NimBLE configuration - config gatt services
  ble_gatts_add_svcs(
      gatt_svcs); // 4 - Initialize NimBLE configuration - queues gatt services.
  ble_hs_cfg.sync_cb = ble_app_on_sync; // 5 - Initialize application
  nimble_port_freertos_init(host_task); // 6 - Run the thread
}

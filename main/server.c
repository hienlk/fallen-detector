#include "stuff.h"

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

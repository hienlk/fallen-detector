
#include "server.h"

#include "host/ble_gatt.h"
#include "mpu.h"

#define CHECK(rc, msg)                                                         \
  do {                                                                         \
    if ((rc) != 0) {                                                           \
      MODLOG_DFLT(ERROR, "%s; rc=%d\n", (msg), (rc));                          \
      return;                                                                  \
    }                                                                          \
  } while (0)

bool fall_detected = false;
bool buzzer_led_active = false;
mpu6050_temp_value_t temp;
mpu6050_acce_value_t acce;
mpu6050_gyro_value_t gyro;
uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

char *TAG = "BLE-Test-Sever";
static uint8_t own_addr_type;
uint8_t ble_addr_type;
mpu6050_handle_t mpu6050 = NULL;
uint16_t fall_char_handle;
uint16_t temp_char_handle;

static bool is_encrypted = false;
bool nimble_enabled = true;

void ble_app_advertise(void);
void ble_store_config_init(void);

void stop_nimble() {
  if (nimble_enabled) {
    ble_gap_adv_stop();
    nimble_port_stop();
    nimble_port_deinit();
    nimble_enabled = false;
    printf("NimBLE stack stopped\n");
  }
}

static void ble_print_conn_desc(struct ble_gap_conn_desc *desc) {
  MODLOG_DFLT(INFO,
              "handle=%d our_ota_addr_type=%d our_ota_addr=", desc->conn_handle,
              desc->our_ota_addr.type);
  print_addr(desc->our_ota_addr.val);
  MODLOG_DFLT(INFO,
              " our_id_addr_type=%d our_id_addr=", desc->our_id_addr.type);
  print_addr(desc->our_id_addr.val);
  MODLOG_DFLT(
      INFO, " peer_ota_addr_type=%d peer_ota_addr=", desc->peer_ota_addr.type);
  print_addr(desc->peer_ota_addr.val);
  MODLOG_DFLT(INFO,
              " peer_id_addr_type=%d peer_id_addr=", desc->peer_id_addr.type);
  print_addr(desc->peer_id_addr.val);
  MODLOG_DFLT(INFO,
              " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
              "encrypted=%d authenticated=%d bonded=%d\n",
              desc->conn_itvl, desc->conn_latency, desc->supervision_timeout,
              desc->sec_state.encrypted, desc->sec_state.authenticated,
              desc->sec_state.bonded);
}

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
         (struct ble_gatt_chr_def[]){
             {
                 .uuid = BLE_UUID16_DECLARE(0xFEF4),
                 .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                          BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                          BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
                 .val_handle = &temp_char_handle,
                 .access_cb = device_read_temp,
             },
             {
                 .uuid = BLE_UUID16_DECLARE(0xFEF5),
                 .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                          BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                          BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
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
  struct ble_gap_conn_desc desc;
  int rc;

  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    ESP_LOGI(TAG, "BLE GAP EVENT CONNECT %s",
             event->connect.status == 0 ? "OK" : "FAILED");
    if (event->connect.status == 0) {
      conn_handle = event->connect.conn_handle;
      rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
      if (rc == 0) {
        ble_print_conn_desc(&desc);
        if (!desc.sec_state.encrypted) {
          rc = ble_gap_security_initiate(conn_handle);
          ESP_LOGI(TAG, "Security initiate result: %d", rc);
        }
      }
    }
    return 0;

  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGI(TAG, "BLE GAP EVENT DISCONNECT reason=%d",
             event->disconnect.reason);
    ble_print_conn_desc(&event->disconnect.conn);

    conn_handle = BLE_HS_CONN_HANDLE_NONE;
    is_encrypted = false;
    ble_app_advertise();
    return 0;

  case BLE_GAP_EVENT_ENC_CHANGE:
    ESP_LOGI(TAG, "BLE GAP EVENT ENC CHANGE: status=%d",
             event->enc_change.status);
    rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
    if (rc == 0) {
      is_encrypted = desc.sec_state.encrypted;
      ESP_LOGI(TAG,
               "Encryption change: encrypted=%d, authenticated=%d, bonded=%d, "
               "key_size=%d",
               desc.sec_state.encrypted, desc.sec_state.authenticated,
               desc.sec_state.bonded, desc.sec_state.key_size);
    }
    return 0;

  case BLE_GAP_EVENT_REPEAT_PAIRING:
    ESP_LOGI(TAG, "BLE GAP EVENT REPEAT PAIRING");
    rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
    if (rc == 0) {

      ble_store_util_delete_peer(&desc.peer_id_addr);
    }
    return BLE_GAP_REPEAT_PAIRING_RETRY;

  case BLE_GAP_EVENT_ADV_COMPLETE:
    ESP_LOGI(TAG, "BLE GAP EVENT ADV COMPLETE");
    ble_app_advertise();
    return 0;

  case BLE_GAP_EVENT_MTU:
    MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                event->mtu.conn_handle, event->mtu.channel_id,
                event->mtu.value);
    return 0;

  case BLE_GAP_EVENT_CONN_UPDATE:
    ESP_LOGI(TAG, "Connection update: status=%d ", event->conn_update.status);
    if (event->conn_update.status == 0) {
      ESP_LOGI(TAG, "Connection parameters updated successfully");
    } else {
      ESP_LOGE(TAG, "Connection parameters update failed");
    }
    return 0;

  case BLE_GAP_EVENT_CONN_UPDATE_REQ:
    ESP_LOGI(TAG, "Connection update request received");
    // Accept the update request
    return 0;

  case BLE_GAP_EVENT_AUTHORIZE:
    MODLOG_DFLT(INFO,
                "authorize event: conn_handle=%d attr_handle=%d is_read=%d",
                event->authorize.conn_handle, event->authorize.attr_handle,
                event->authorize.is_read);

    /* The default behaviour for the event is to reject authorize request */
    event->authorize.out_response = BLE_GAP_AUTHORIZE_ACCEPT;
    return 0;

  case BLE_GAP_EVENT_NOTIFY_TX:
    MODLOG_DFLT(INFO,
                "notify_tx event; conn_handle=%d attr_handle=%d "
                "status=%d is_indication=%d",
                event->notify_tx.conn_handle, event->notify_tx.attr_handle,
                event->notify_tx.status, event->notify_tx.indication);
    return 0;

  case BLE_GAP_EVENT_SUBSCRIBE:
    MODLOG_DFLT(INFO,
                "subscribe event; conn_handle=%d attr_handle=%d "
                "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                event->subscribe.conn_handle, event->subscribe.attr_handle,
                event->subscribe.reason, event->subscribe.prev_notify,
                event->subscribe.cur_notify, event->subscribe.prev_indicate,
                event->subscribe.cur_indicate);
    return 0;

  case BLE_GAP_EVENT_PASSKEY_ACTION:
    ESP_LOGI(TAG, "PASSKEY_ACTION_EVENT started");
    struct ble_sm_io pkey = {0};
    int key = 0;

    if (event->passkey.params.action == BLE_SM_IOACT_DISP) {
      pkey.action = event->passkey.params.action;
      pkey.passkey = 123456; // This is the passkey to be entered on peer
      ESP_LOGI(TAG, "Enter passkey %" PRIu32 "on the peer side", pkey.passkey);
      rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
      ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
    } else if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
      ESP_LOGI(TAG, "Passkey on device's display: %" PRIu32,
               event->passkey.params.numcmp);
      ESP_LOGI(TAG, "Accept or reject the passkey through console in this "
                    "format -> key Y or key N");
      pkey.action = event->passkey.params.action;
      if (scli_receive_key(&key)) {
        pkey.numcmp_accept = key;
      } else {
        pkey.numcmp_accept = 0;
        ESP_LOGE(TAG, "Timeout! Rejecting the key");
      }
      rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
      ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
    } else if (event->passkey.params.action == BLE_SM_IOACT_OOB) {
      static uint8_t tem_oob[16] = {0};
      pkey.action = event->passkey.params.action;
      for (int i = 0; i < 16; i++) {
        pkey.oob[i] = tem_oob[i];
      }
      rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
      ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
    } else if (event->passkey.params.action == BLE_SM_IOACT_INPUT) {
      ESP_LOGI(TAG,
               "Enter the passkey through console in this format-> key 123456");
      pkey.action = event->passkey.params.action;
      if (scli_receive_key(&key)) {
        pkey.passkey = key;
      } else {
        pkey.passkey = 0;
        ESP_LOGE(TAG, "Timeout! Passing 0 as the key");
      }
      rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
      ESP_LOGI(TAG, "ble_sm_inject_io result: %d", rc);
    }
    return 0;

  default:
    return 0;
  }
}

static void ble_on_reset(int reason) {
  MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void ble_app_advertise(void) {
  int rc;
  struct ble_hs_adv_fields fields;
  struct ble_gap_adv_params adv_params;

  const char *device_name = ble_svc_gap_device_name();
  memset(&fields, 0, sizeof(fields));

  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.name = (uint8_t *)device_name;
  fields.name_len = strlen(device_name);
  fields.name_is_complete = 1;

  // ble_gap_adv_set_fields(&fields);

  rc = ble_gap_adv_set_fields(&fields);
  CHECK(rc, "error setting advertisement data");

  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                         ble_gap_event, NULL);
  CHECK(rc, "error enabling advertisement");
}

void ble_app_on_sync(void) {
  int rc;
  rc = ble_hs_util_ensure_addr(0);
  ble_hs_id_infer_auto(0, &ble_addr_type);

  assert(rc == 0);

  uint8_t addr_val[6] = {0};
  rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);

  MODLOG_DFLT(INFO, "Device Address: ");
  print_addr(addr_val);
  MODLOG_DFLT(INFO, "\n");

  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  CHECK(rc, "error determining address type");

  ble_app_advertise();
}

void host_task(void *param) {
  nimble_port_run();
  nimble_port_freertos_deinit();
}

void send_notification(uint16_t attr_handle, void *data, size_t data_len) {
  if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, data_len);
    if (om) {
      ble_gatts_notify_custom(conn_handle, attr_handle, om);
    }
  }
}

void gatt_server(void) {
  nvs_flash_init();
  esp_nimble_hci_init();
  nimble_port_init();
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
  ble_hs_cfg.sm_bonding = 1;
  ble_hs_cfg.sm_mitm = 1;
  ble_hs_cfg.sm_sc = 0;
  ble_hs_cfg.reset_cb = ble_on_reset;
  ble_hs_cfg.sync_cb = ble_app_on_sync;

  ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC;
  ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC;

  ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ID;
  ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ID;

  ble_svc_gap_device_name_set("BLE-Server");

  ble_svc_gap_init();
  ble_svc_gatt_init();

  ble_gatts_count_cfg(gatt_svcs);
  ble_gatts_add_svcs(gatt_svcs);
  ble_store_config_init();
  nimble_port_freertos_init(host_task);
}
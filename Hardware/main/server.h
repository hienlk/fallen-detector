
#ifndef MAIN_SERVER_H_
#define MAIN_SERVER_H_

#include "esp_peripheral.h"

#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "esp_rom_gpio.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "services/ans/ble_svc_ans.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/*GPIO definitions*/
#define BUZZER_GPIO 4
#define LED_GPIO 5
#define CHECK_BUTTON 10
#define STOP_BUTTON 7
#define BLE_BUTTON 1

extern uint16_t temp_char_handle;
extern uint16_t fall_char_handle;

extern bool fall_detected;
extern bool buzzer_led_active;
extern bool nimble_enabled;

extern uint16_t conn_handle;

void ble_app_advertise(void);

int device_read_temp(uint16_t con_handle, uint16_t attr_handle,
                     struct ble_gatt_access_ctxt *ctxt, void *arg);
int device_read_fallen(uint16_t con_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg);

void blink_led(bool activate);
void buzzer(bool activate);

void gatt_server(void);
void host_task(void *param);
void ble_app_on_sync(void);
void send_notification(uint16_t attr_handle, void *data, size_t data_len);

void stop_nimble();

#endif

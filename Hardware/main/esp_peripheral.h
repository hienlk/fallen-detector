
#ifndef H_ESP_PERIPHERAL_
#define H_ESP_PERIPHERAL_

#include "console/console.h"

#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_sm.h"

#include "unity.h"
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

/* Console */
int scli_init(void);
int scli_receive_key(int *key);

/** Misc. */
void print_bytes(const uint8_t *bytes, int len);
void print_addr(const void *addr);
char *addr_str(const void *addr);
void print_mbuf(const struct os_mbuf *om);

#endif
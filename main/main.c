
#include "mpu.h"
#include "server.h"

SemaphoreHandle_t xReadDataSemaphore;
SemaphoreHandle_t xProcessDataSemaphore;
SemaphoreHandle_t xIsFallenSemaphore;
SemaphoreHandle_t xBuzzerLedMutex;

void init_semaphores() {
  xReadDataSemaphore = xSemaphoreCreateBinary();
  xProcessDataSemaphore = xSemaphoreCreateBinary();
  xIsFallenSemaphore = xSemaphoreCreateBinary();
  xBuzzerLedMutex = xSemaphoreCreateMutex();
}

void vTaskConnectBle(void *pvParameters) {

  for (;;) {

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}

void vTaskCheckButton(void *pvParameters) { // Emergency button
  for (;;) {
    if (gpio_get_level(CHECK_BUTTON) == 0) {
      printf("Button Pressed!\n");

      if (xSemaphoreTake(xBuzzerLedMutex, portMAX_DELAY) == pdTRUE) {
        fall_detected = true;
        buzzer_led_active = true;
        xSemaphoreGive(xBuzzerLedMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vTaskNotifyBle(void *pvParameters) {
  char temp_data[20];
  char fall_data[20];

  while (1) {
    sprintf(temp_data, "Temp: %.2f", temp.temp);
    send_notification(temp_char_handle, temp_data, strlen(temp_data));

    sprintf(fall_data, fall_detected ? "Fallen" : "No");
    send_notification(fall_char_handle, fall_data, strlen(fall_data));

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vTaskStopButton(void *pvParameters) {
  for (;;) {
    if (gpio_get_level(STOP_BUTTON) == 0) {
      printf("Stop Button Pressed!\n");

      if (xSemaphoreTake(xBuzzerLedMutex, portMAX_DELAY) == pdTRUE) {
        fall_detected = false;
        buzzer_led_active = false;
        xSemaphoreGive(xBuzzerLedMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vTaskBuzzerLed(void *pvParameters) {
  for (;;) {

    if (xSemaphoreTake(xBuzzerLedMutex, portMAX_DELAY) == pdTRUE) {
      if (buzzer_led_active) {
        buzzer(true);
        blink_led(true);
      } else {
        buzzer(false);
        blink_led(false);
      }
      xSemaphoreGive(xBuzzerLedMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vTaskReadData(void *pvParameters) {
  for (;;) {
    read_temperature(mpu6050);
    read_data(mpu6050, &acce, &gyro);
    printf("jerk = %.2f \n", jerk_magnitude);
    printf("angle = %.2f \n", angle);

    xSemaphoreGive(xReadDataSemaphore);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void vTaskProcessData(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xReadDataSemaphore, portMAX_DELAY) == pdTRUE) {
      process_data(&acce, &gyro);

      xSemaphoreGive(xProcessDataSemaphore);
    }
  }
}

void vTaskIsFallen(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xProcessDataSemaphore, portMAX_DELAY) == pdTRUE) {
      is_fallen(jerk_magnitude, angle);
      xSemaphoreGive(xIsFallenSemaphore);
    }
  }
}

void vTaskOnOff(void *pvParameters) {
  for (;;) {
    if (gpio_get_level(BLE_BUTTON) == 0) {
      printf("Enable/Disable Button Pressed!\n");

      if (!nimble_enabled) {
        printf("Starting NimBLE stack...\n");
        gatt_server();
        nimble_enabled = true;
      } else {
        printf("Stopping NimBLE stack...\n");
        stop_nimble();
      }

      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void app_main(void) {
  gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);

  gpio_set_direction(CHECK_BUTTON, GPIO_MODE_INPUT);
  gpio_set_pull_mode(CHECK_BUTTON, GPIO_PULLUP_ONLY);
  gpio_set_direction(STOP_BUTTON, GPIO_MODE_INPUT);
  gpio_set_pull_mode(STOP_BUTTON, GPIO_PULLUP_ONLY);
  gpio_set_direction(BLE_BUTTON, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BLE_BUTTON, GPIO_PULLUP_ONLY);

  i2c_sensor_mpu6050_init();

  init_semaphores();
  gatt_server();

  xTaskCreate(vTaskConnectBle, "connect_ble", 1024 * 2, NULL, 7, NULL);
  xTaskCreate(vTaskNotifyBle, "notify_ble", 1024 * 2, NULL, 6, NULL);
  xTaskCreate(vTaskCheckButton, "check_button", 1024 * 2, NULL, 6, NULL);
  xTaskCreate(vTaskReadData, "read_data", 1024 * 2, NULL, 5, NULL);
  xTaskCreate(vTaskProcessData, "process_data", 1024 * 2, NULL, 4, NULL);
  xTaskCreate(vTaskIsFallen, "is_fallen", 1024 * 2, NULL, 3, NULL);
  xTaskCreate(vTaskBuzzerLed, "control_task", 1024 * 2, NULL, 1, NULL);

  xTaskCreate(vTaskOnOff, "nimble", 1024 * 2, NULL, 7, NULL);
}
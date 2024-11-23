
#include "mpu.h"
#include "server.h"

/*Semaphores*/
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



/**
 * @brief Task to monitor and handle emergency button presses
 *
 * This task continuously checks the state of the emergency button.
 * If the button is pressed, it prints a message and sets the
 * fall_detected and buzzer_led_active flags to true, ensuring
 * thread safety using a mutex. The task operates in an infinite
 * loop with a delay between iterations to manage timing.
 *
 * @param pvParameters Pointer to the parameters passed to the task (not used)
 */
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

/**
 * @brief Task to send BLE notifications for temperature and fall status
 *
 * This task periodically sends BLE notifications containing the current
 * temperature and fall status. The temperature data is sent using the
 * `temp_char_handle` and indicates the current temperature reading formatted
 * to two decimal places. The fall status is sent using the `fall_char_handle`
 * and indicates whether a fall has been detected ("Fallen") or not ("No").
 * The task operates in an infinite loop with a delay between iterations
 * to manage notification timing.
 *
 * @param pvParameters Pointer to the parameters passed to the task (not used)
 */
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

/**
 * @brief Task to monitor the Stop button and turn off the buzzer/LED when
 * pressed
 *
 * This task runs in an infinite loop and periodically checks the state of the
 * Stop button. When the button is pressed, the fall status is cleared and the
 * buzzer/LED are turned off. The task does not block and will rapidly poll the
 * button state when it is pressed.
 *
 * @param pvParameters Pointer to the parameters passed to the task (not used)
 */
void vTaskStopButton(void *pvParameters) {
  for (;;) {
    if (gpio_get_level(STOP_BUTTON) == 0) {
      printf("Stop Button Pressed!\n");

      if (xSemaphoreTake(xBuzzerLedMutex, portMAX_DELAY) == pdTRUE) {
        fall_detected = false;
        buzzer_led_active = false;
        printf("Off led buz");
        xSemaphoreGive(xBuzzerLedMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/**
 * @brief Task to control the buzzer and LED based on the active status
 *
 * This task continuously checks whether the buzzer and LED should be active.
 * If the `buzzer_led_active` flag is true, it turns on the buzzer and blinks
 * the LED, otherwise it turns them off. The task ensures thread safety by
 * using a mutex to access the `buzzer_led_active` flag. It operates in an
 * infinite loop with a delay between iterations to manage timing.
 *
 * @param pvParameters Pointer to the parameters passed to the task (not used)
 */
void vTaskBuzzerLed(void *pvParameters) {
  for (;;) {

    if (xSemaphoreTake(xBuzzerLedMutex, portMAX_DELAY) == pdTRUE) {
      if (buzzer_led_active) {
        buzzer(true);
        blink_led(true);
        printf("Led Buz\n");
      } else {
        buzzer(false);
        blink_led(false);
      }
      xSemaphoreGive(xBuzzerLedMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/**
 * @brief Task to read sensor data from the MPU6050
 *
 * This task continuously reads temperature and motion data from the MPU6050
 * sensor. The temperature data is fetched using the `read_temperature`
 * function, and the accelerometer and gyroscope data are fetched using the
 * `read_data` function. After reading the data, it gives the
 * `xReadDataSemaphore` semaphore to signal other tasks that new data is
 * available. The task operates in an infinite loop with a delay between
 * iterations to manage sensor read timing.
 *
 * @param pvParameters Pointer to the parameters passed to the task (not used)
 */
void vTaskReadData(void *pvParameters) {
  for (;;) {
    read_temperature(mpu6050);
    read_data(mpu6050, &acce, &gyro);

    xSemaphoreGive(xReadDataSemaphore);

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/**
 * @brief Task to process accelerometer and gyroscope data
 *
 * This task waits for new sensor data to be available by taking the
 * `xReadDataSemaphore`. Once data is available, it processes the accelerometer
 * and gyroscope data using the `process_data` function to calculate parameters
 * like jerk magnitude. It then prints the calculated jerk magnitude and gives
 * the `xProcessDataSemaphore` to signal that the processed data is ready for
 * further tasks to use. The task operates in an infinite loop.
 *
 * @param pvParameters Pointer to the parameters passed to the task (not used)
 */
void vTaskProcessData(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xReadDataSemaphore, portMAX_DELAY) == pdTRUE) {
      process_data(&acce, &gyro);
      printf("jerk = %.2f \n", jerk_magnitude);
      // printf("angle = %.2f \n", angle);

      xSemaphoreGive(xProcessDataSemaphore);
    }
  }
}

/**
 * @brief Task to check whether a fall has been detected
 *
 * This task waits for new processed sensor data to be available by taking the
 * `xProcessDataSemaphore`. Once data is available, it checks whether the
 * calculated jerk magnitude and angle indicate that a fall has been detected
 * using the `is_fallen` function. If a fall has been detected, the
 * `fall_detected` flag is set to true. The task then gives the
 * `xIsFallenSemaphore` to signal that the fall detection status is ready for
 * other tasks to use. The task operates in an infinite loop.
 *
 * @param pvParameters Pointer to the parameters passed to the task (not used)
 */
void vTaskIsFallen(void *pvParameters) {
  for (;;) {
    if (xSemaphoreTake(xProcessDataSemaphore, portMAX_DELAY) == pdTRUE) {
      is_fallen(jerk_magnitude, angle);
      xSemaphoreGive(xIsFallenSemaphore);
    }
  }
}

/**
 * @brief Task to monitor the enable/disable button and toggle the NimBLE stack
 *
 * This task continuously checks the state of the enable/disable button.
 * If the button is pressed, it toggles the state of the NimBLE stack. If the
 * stack is not running, it prints a message and starts the stack. If the stack
 * is running, it prints a message and stops the stack. The task then waits a
 * second before checking the button again. The task operates in an infinite
 * loop.
 *
 * @param pvParameters Pointer to the parameters passed to the task (not used)
 */
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
  /* GPIO setup */
  gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT); 
  gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT); 
  gpio_set_direction(CHECK_BUTTON, GPIO_MODE_INPUT);
  gpio_set_pull_mode(CHECK_BUTTON, GPIO_PULLUP_ONLY);
  gpio_set_direction(STOP_BUTTON, GPIO_MODE_INPUT);
  gpio_set_pull_mode(STOP_BUTTON, GPIO_PULLUP_ONLY);
  gpio_set_direction(BLE_BUTTON, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BLE_BUTTON, GPIO_PULLUP_ONLY);


  i2c_sensor_mpu6050_init();

  init_semaphores();

  gatt_server();
  
  /* Create tasks */
  xTaskCreate(vTaskNotifyBle, "notify_ble", 1024 * 2, NULL, 6, NULL);
  xTaskCreate(vTaskCheckButton, "check_button", 1024 * 2, NULL, 6, NULL);
  xTaskCreate(vTaskStopButton, "stop_button", 1025 * 2, NULL, 5, NULL);
  xTaskCreate(vTaskReadData, "read_data", 1024 * 2, NULL, 5, NULL);
  xTaskCreate(vTaskProcessData, "process_data", 1024 * 2, NULL, 5, NULL);
  xTaskCreate(vTaskIsFallen, "is_fallen", 1024 * 2, NULL, 3, NULL);
  xTaskCreate(vTaskBuzzerLed, "control_task", 1024 * 2, NULL, 1, NULL);
  xTaskCreate(vTaskOnOff, "nimble", 1024 * 2, NULL, 7, NULL);
}
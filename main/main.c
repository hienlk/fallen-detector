#include "esp_rom_gpio.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include "hal/mpu_types.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "mpu6050.h"
#include "soc/gpio_num.h"
#include "unity.h"
#include "math.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"

#define BUZZER_GPIO 4
#define LED_GPIO 5
#define CHECK_BUTTON 10
#define STOP_BUTTON 18

#define I2C_MASTER_SCL_IO 9      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 8      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define RAD_TO_DEG 57.2958

static mpu6050_handle_t mpu6050 = NULL;
uint8_t mpu6050_deviceid;
mpu6050_acce_value_t acce;
mpu6050_gyro_value_t gyro;
mpu6050_temp_value_t temp;

SemaphoreHandle_t xReadDataSemaphore;
SemaphoreHandle_t xProcessDataSemaphore;
SemaphoreHandle_t xIsFallenSemaphore;
SemaphoreHandle_t xBuzzerLedMutex;



bool fall_detected = false;
bool buzzerLedActive = false;

const float fallThreshold = 650000; 
const float angleThreshold = 45.0;
const int sampleInterval = 10;  
float prevAccX = 0.0, prevAccY = 0.0, prevAccZ = 0.0;
float prevAngleGyro = 0.0;
//const int DOUBLE_PRESS_THRESHOLD = 1500; 
float jerkMagnitude = 0.0;
float angleAcc = 0.0;
float angleGyro = 0.0;
float angle = 0.0;



static void i2c_bus_init(void)
{
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

static void i2c_sensor_mpu6050_init(void)
{
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
        printf("Temperature: %.2f°C\n", temp.temp);
    } else {
        printf("Error reading temperature: %d\n", err);
    }
}

void read_data(mpu6050_handle_t mpu6050, mpu6050_acce_value_t *acce_value, mpu6050_gyro_value_t *gyro_value){

    esp_err_t err;
    
    err = mpu6050_get_acce(mpu6050, &acce);
    if (err == ESP_OK) {
        printf("Accel: X=%.2f, Y=%.2f, Z=%.2f (m/s^2)\n", 
               acce_value->acce_x, acce_value->acce_y, acce_value->acce_z);
    } else {
        printf("Error reading accelerometer: %d\n", err);
    }

    err = mpu6050_get_gyro(mpu6050, &gyro);
    if (err == ESP_OK) {
        printf("Gyro: X=%.2f, Y=%.2f, Z=%.2f (degrees/s)\n", 
               gyro_value->gyro_x, gyro_value->gyro_y, gyro_value->gyro_z);
    } else {
        printf("Error reading gyroscope: %d\n", err);
    }
}
    

void process_data(mpu6050_acce_value_t *acce_value, mpu6050_gyro_value_t *gyro_value) {
    float acceleration_mg_x = acce_value->acce_x / 9.8;
    float acceleration_mg_y = acce_value->acce_y / 9.8;
    float acceleration_mg_z = acce_value->acce_z / 9.8;
    
    float jerkX = (acceleration_mg_x - prevAccX) / (sampleInterval / 1000.0);
    float jerkY = (acceleration_mg_y - prevAccY) / (sampleInterval / 1000.0);
    float jerkZ = (acceleration_mg_z - prevAccZ) / (sampleInterval / 1000.0);

    jerkMagnitude = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);

    prevAccX = acceleration_mg_x;
    prevAccY = acceleration_mg_y;
    prevAccZ = acceleration_mg_z;
    
    
 	angleAcc = atan2(acceleration_mg_y, sqrt(acceleration_mg_x * acceleration_mg_x + acceleration_mg_z * acceleration_mg_z)) * RAD_TO_DEG;
    float deltaTime = sampleInterval / 1000.0; 
    angleGyro += gyro_value->gyro_x * deltaTime;


    float alpha = 0.98;  
    angle = alpha * angleGyro + (1.0 - alpha) * angleAcc;
    
}


void buzzer(bool activate) {
	
    gpio_set_level(BUZZER_GPIO, activate ? 1 : 0);
}

void blink_led(bool activate) {
	
    gpio_set_level(LED_GPIO, activate ? 1 : 0);
}



void isFallen(float jerk, float angle) {
    if (jerk > fallThreshold && fabs(angle) > 45.0) { 
        printf("Fallen detected!\n");
        fall_detected = true;
		buzzerLedActive = true; 	
    } else {
        fall_detected = false;
        /*
        buzzer(false);
        blink_led(false);
        */
        buzzerLedActive = false; 

    }
}


void connect_ble(){ // output: T/F, blink led 
	
	
}


void tranfer_data_ble(){
	
	
}




void init_semaphores() {
    xReadDataSemaphore = xSemaphoreCreateBinary();
    xProcessDataSemaphore = xSemaphoreCreateBinary();
    xIsFallenSemaphore = xSemaphoreCreateBinary();
    xBuzzerLedMutex = xSemaphoreCreateMutex();
}

void vTaskCheckButton(void *pvParameters) {  // Emergency button *chua xu ly chong rung
    for (;;) {
        if (gpio_get_level(CHECK_BUTTON) == 0) {
            printf("Button Pressed!\n");

            if (xSemaphoreTake(xBuzzerLedMutex, portMAX_DELAY) == pdTRUE) {
                fall_detected = true;
                buzzerLedActive = true;
                xSemaphoreGive(xBuzzerLedMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vTaskStopButton(void *pvParameters) {
    for (;;) {
        if (gpio_get_level(STOP_BUTTON) == 0) {
            printf("Stop Button Pressed!\n");

            if (xSemaphoreTake(xBuzzerLedMutex, portMAX_DELAY) == pdTRUE) {
                fall_detected = false;
                buzzerLedActive = false;
                xSemaphoreGive(xBuzzerLedMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vTaskBuzzerLed(void *pvParameters) {
    for (;;) {

        if (xSemaphoreTake(xBuzzerLedMutex, portMAX_DELAY) == pdTRUE) {
            if (buzzerLedActive) {
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
        printf("%.2f \n",jerkMagnitude);
        printf("%.2f \n",angle);
        
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
            isFallen(jerkMagnitude, angle);  
            xSemaphoreGive(xIsFallenSemaphore);
        }
    }
}




void app_main(void) {
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
   
    gpio_set_direction(CHECK_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(CHECK_BUTTON, GPIO_PULLUP_ONLY);
    gpio_set_direction(STOP_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(STOP_BUTTON, GPIO_PULLUP_ONLY);
    
    i2c_sensor_mpu6050_init();
    
    init_semaphores();
    
    xTaskCreate(vTaskCheckButton, "check_button", 1024*2, NULL, 6, NULL);
    xTaskCreate(vTaskReadData, "read_data", 1024*2, NULL, 5, NULL);
    xTaskCreate(vTaskProcessData, "process_data", 1024*2, NULL, 4, NULL);
    xTaskCreate(vTaskIsFallen, "is_fallen", 1024*2, NULL, 3, NULL);
    xTaskCreate(vTaskBuzzerLed, "control_task", 1024*2, NULL, 1, NULL);
    
}

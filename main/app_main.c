/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include <driver/i2c.h>
#include <esp_log.h>
#include <stdio.h>
#include "bmx280.h"

#define BMP180_ADDRESS 0x76
#define SDA_PIN 22
#define SCL_PIN 23 

static char tag[] = "BMP180";

void task_i2cscanner(void *ignore) { // Detect I2C device addresses
    ESP_LOGD(tag, ">> i2cScanner");
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &conf);

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0); 

    int i;
    esp_err_t espRc;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    for (i=3; i< 0x78; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
        i2c_master_stop(cmd);

        espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
        if (i%16 == 0) {
            printf("\n%.2x:", i); 
        }
        if (espRc == 0) {
            printf(" %.2x", i); 
        } else {
            printf(" --");
        }
        //ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
        i2c_cmd_link_delete(cmd);
    }   
    printf("\n");
    vTaskDelete(NULL);
}

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
 */
#define BLINK_GPIO CONFIG_BLINK_GPIO

void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
     */
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static bmx280_config_t bmx280_config; //bmp280 config structure

#define TAG "bmp280"

static void env_sensor_callback(env_data_t* env_data) {
	if (env_data->sensor_idx <= 1) {
		ESP_LOGI(TAG,"env (%d): temp : %.2f C, pressure: %.2f hPa"
            , env_data->sensor_idx, env_data->temp, env_data->pressure);
	} else {
		ESP_LOGE(TAG, "env (%d) - invalid sensor", env_data->sensor_idx);
	}
}

void task_bmp280(void *ignore) {
   memset(&bmx280_config, 0, sizeof(bmx280_config_t)); //zero the memory address first

	if (bmx280_set_hardware_config(&bmx280_config, 0) == ESP_OK) {
		bmx280_config.interval = 1000;
		bmx280_config.callback = &env_sensor_callback;

		if (bmx280_init(&bmx280_config) != ESP_OK) {
			ESP_LOGE(TAG, "couldn't initialise bmx280 sensor %d", 0);
		}
	}
    vTaskDelete( NULL );
}

void app_main()
{
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    //xTaskCreate(&task_i2cscanner, "i2c_scanner", 4096, NULL, 5, NULL);
    xTaskCreate(&task_bmp280, "bmp280_task", 81920, NULL, 5, NULL);
}

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "sdkconfig.h"


#define DATA_LENGTH 8
#define ACK_CHECK_ENABLED 0x1
#define ACK_CHECK_DISABLED 0x0
#define ACK_VALUE 0x0
#define NACK_VALUE 0x1

static esp_err_t i2c_master_init()
{
    int i2c_master_port = CONFIG_I2C_MASTER_PORT_NUMBER;
    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = CONFIG_I2C_MASTER_SDA;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_io_num = CONFIG_I2C_MASTER_SCL;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = CONFIG_I2C_MASTER_FREQUENCY_IN_HZ;
    i2c_param_config(i2c_master_port, &config);
    return i2c_driver_install(i2c_master_port, config.mode,
                              CONFIG_I2C_MASTER_RX_BUFFER_DISABLE,
                              CONFIG_I2C_MASTER_TX_BUFFER_DISABLE, 0); 
}

static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (CONFIG_AM2320_I2C_SENSOR_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_ENABLED);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VALUE);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VALUE);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void read_am2320_sensor(void * pvParameters)
{
    uint8_t * data_read = (uint8_t *)malloc(DATA_LENGTH);
    while (1)
    {
        if (i2c_master_read_slave(CONFIG_I2C_MASTER_PORT_NUMBER, data_read, DATA_LENGTH) == ESP_FAIL)
        {
            printf("AM2320 sensor read failed");
        }
        else
        {
            printf("Data read %u \n", *data_read);
        }
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}


void app_main()
{
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(read_am2320_sensor, "Read AM2320 sensor", 1024 * 2, (void *)0, 10, NULL);
}

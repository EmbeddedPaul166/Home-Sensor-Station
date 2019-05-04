#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "sdkconfig.h"


#define DATA_LENGTH 16
#define ACK_CHECK_ENABLED 0x1
#define ACK_CHECK_DISABLED 0x0
#define ACK_VALUE 0x0
#define NACK_VALUE 0x1

#define AM2320_I2C_SENSOR_ADDRESS CONFIG_AM2320_I2C_SENSOR_ADDRESS
#define AM2320_FUNCTION_CODE_READ 0x03
#define AM2320_START_REGISTER_ADDRESS 0x0
#define AM2320_NUMBER_OF_REGISTERS_TO_READ 0x04

static esp_err_t i2c_master_init()
{
    int i2c_master_port = CONFIG_I2C_MASTER_PORT_NUMBER;
    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = CONFIG_I2C_MASTER_SDA;
    config.scl_io_num = CONFIG_I2C_MASTER_SCL;
    config.master.clk_speed = CONFIG_I2C_MASTER_FREQUENCY_IN_HZ;
    i2c_param_config(i2c_master_port, &config);
    return i2c_driver_install(i2c_master_port, config.mode,
                              CONFIG_I2C_MASTER_RX_BUFFER_DISABLE,
                              CONFIG_I2C_MASTER_TX_BUFFER_DISABLE, 0); 
}

static esp_err_t AM2320_request_humidity_and_temperature_values(i2c_port_t i2c_port_number)
{
    //request temperature and humidity values from the sensor
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_I2C_SENSOR_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLED);
    i2c_master_write_byte(cmd, AM2320_FUNCTION_CODE_READ, ACK_CHECK_ENABLED);
    i2c_master_write_byte(cmd, AM2320_START_REGISTER_ADDRESS, ACK_CHECK_ENABLED);
    i2c_master_write_byte(cmd, AM2320_NUMBER_OF_REGISTERS_TO_READ, ACK_CHECK_ENABLED);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_number, cmd, 2000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t AM2320_receive_humidity_and_temperature_values(i2c_port_t i2c_port_number, uint16_t *humidity_buffer, int16_t *temperature_buffer, uint8_t dump_buffer, size_t data_size)
{
    //receive temperature and humidity values from the sensor
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((AM2320_I2C_SENSOR_ADDRESS+0x01) << 1) | I2C_MASTER_READ, ACK_CHECK_ENABLED);
    i2c_master_read_byte(cmd, dump_buffer, ACK_VALUE);
    i2c_master_read_byte(cmd, dump_buffer, ACK_VALUE);
    i2c_master_read(cmd, humidity_buffer, data_size - 1, ACK_VALUE);
    i2c_master_read(cmd, temperature_buffer, data_size - 1, ACK_VALUE);
    i2c_master_read_byte(cmd, dump_buffer, ACK_VALUE);
    i2c_master_read_byte(cmd, 1, NACK_VALUE);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port_number, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void read_am2320_sensor(void * pvParameters)
{
    vTaskDelay(3000 / portTICK_RATE_MS);
    uint16_t * humidity_read = (uint16_t *)malloc(DATA_LENGTH);
    int16_t * temperature_read = (int16_t *)malloc(DATA_LENGTH);
    uint8_t * dump_buffer = (uint8_t *)malloc(8);
    while (1)
    {
        if (AM2320_request_humidity_and_temperature_values(CONFIG_I2C_MASTER_PORT_NUMBER) != ESP_FAIL)
        {
            if (AM2320_receive_humidity_and_temperature_values(CONFIG_I2C_MASTER_PORT_NUMBER, humidity_read, temperature_read, dump_buffer, DATA_LENGTH) != ESP_FAIL)
            {
                printf("Humidity %f \n", (float)*humidity_read/10.0f);
                printf("Temperature %f \n", (float)*temperature_read/10.0f);
            }
        }
        else
        {
            printf("AM2320 sensor read failed");
        }
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}


void app_main()
{
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(read_am2320_sensor, "Read AM2320 sensor", 1024 * 2, (void *)0, 10, NULL);
    vTaskSuspend(NULL);
}

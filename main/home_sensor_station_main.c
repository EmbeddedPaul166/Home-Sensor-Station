#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "sdkconfig.h"


#define DATA_LENGTH 512
#define ACK_CHECK_ENABLED 0x1
#define ACK_CHECK_DISABLED 0x0
#define ACK_VALUE 0x0
#define NACK_VALUE 0x1

#define AM2320_I2C_SENSOR_ADDRESS CONFIG_I2C_AM2320_SENSOR_ADDRESS
#define AM2320_FUNCTION_CODE_READ 0x03
#define AM2320_HUMIDITY_START_REGISTER 0x00
#define AM2320_TEMPERATURE_START_REGISTER 0x00
#define AM2320_NUMBER_OF_REGISTERS_TO_READ 0x2

SemaphoreHandle_t mutex;

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

static esp_err_t AM2320_wake_sensor(i2c_port_t i2c_port_number)
{
    //Wake sensor
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_I2C_SENSOR_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_DISABLED);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port_number, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t AM2320_request(i2c_port_t i2c_port_number, uint8_t start_register)
{
    //Request values from sensor
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_I2C_SENSOR_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_ENABLED);
    i2c_master_write_byte(cmd, AM2320_FUNCTION_CODE_READ, ACK_CHECK_ENABLED);
    i2c_master_write_byte(cmd, start_register, ACK_CHECK_ENABLED);
    i2c_master_write_byte(cmd, AM2320_NUMBER_OF_REGISTERS_TO_READ, ACK_CHECK_ENABLED);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port_number, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t AM2320_read(i2c_port_t i2c_port_number, uint8_t *upper_bits_buffer,
                             uint8_t *lower_bits_buffer, uint8_t *dump_buffer, size_t data_size)
{
    //Read values from sensor
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_I2C_SENSOR_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_ENABLED);
    i2c_master_read_byte(cmd, dump_buffer, ACK_VALUE);
    i2c_master_read_byte(cmd, dump_buffer, ACK_VALUE);
    i2c_master_read_byte(cmd, dump_buffer, ACK_VALUE);
    i2c_master_read(cmd, upper_bits_buffer, data_size, ACK_VALUE);
    i2c_master_read(cmd, lower_bits_buffer, data_size, ACK_VALUE);
    i2c_master_read_byte(cmd, dump_buffer, ACK_VALUE);
    i2c_master_read_byte(cmd, dump_buffer, NACK_VALUE);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port_number, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void readSensor(uint8_t value, uint16_t * buffer, uint8_t * lower_bits_buffer,
                uint8_t * upper_bits_buffer, uint8_t * dump_buffer)
{ 
    if (value == 0)
    {    
        AM2320_wake_sensor(CONFIG_I2C_MASTER_PORT_NUMBER);
        vTaskDelay(10 / portTICK_RATE_MS);
        AM2320_request(CONFIG_I2C_MASTER_PORT_NUMBER, AM2320_TEMPERATURE_START_REGISTER);
        vTaskDelay(10 / portTICK_RATE_MS);
        AM2320_read(CONFIG_I2C_MASTER_PORT_NUMBER, upper_bits_buffer,
                    lower_bits_buffer, dump_buffer, DATA_LENGTH);
        
        *buffer = *upper_bits_buffer;
        *buffer <<= 8;
        *buffer |= *lower_bits_buffer;

        printf("Temperature: %f \n", (float)*buffer/10.0f);
    }
    else if (value == 1)
    { 
        AM2320_wake_sensor(CONFIG_I2C_MASTER_PORT_NUMBER);
        vTaskDelay(10 / portTICK_RATE_MS);
        AM2320_request(CONFIG_I2C_MASTER_PORT_NUMBER, AM2320_HUMIDITY_START_REGISTER);
        vTaskDelay(10 / portTICK_RATE_MS);
        AM2320_read(CONFIG_I2C_MASTER_PORT_NUMBER, upper_bits_buffer,
                    lower_bits_buffer, dump_buffer, DATA_LENGTH);
        
        *buffer = *upper_bits_buffer;
        *buffer <<= 8;
        *buffer |= *lower_bits_buffer;

        printf("Humidity: %f \n", (float)*buffer/10.0f);
    }
    
}

void AM2320_handle_sensor(void * pvParameters)
{
    vTaskDelay(1000 / portTICK_RATE_MS);
    
    uint8_t * humidity_read_upper_bits = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t * humidity_read_lower_bits = (uint8_t *)malloc(DATA_LENGTH);
    uint16_t * humidity_read = (uint16_t *)malloc(DATA_LENGTH);
    uint8_t * temperature_read_upper_bits = (uint8_t *)malloc(DATA_LENGTH);
    uint8_t * temperature_read_lower_bits = (uint8_t *)malloc(DATA_LENGTH);
    uint16_t * temperature_read = (uint16_t *)malloc(DATA_LENGTH);    
    uint8_t * dump_buffer = (uint8_t *)malloc(DATA_LENGTH);
    
    *humidity_read = 0;
    *temperature_read = 0;
    *dump_buffer = 0;
    
    while (1)
    {
        xSemaphoreTake(mutex, portMAX_DELAY);
        readSensor(0, temperature_read, temperature_read_upper_bits, temperature_read_lower_bits, dump_buffer);
        readSensor(1, humidity_read, humidity_read_upper_bits, humidity_read_lower_bits, dump_buffer);
        xSemaphoreGive(mutex); 
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}


void app_main()
{
    mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(i2c_master_init());
    xTaskCreate(AM2320_handle_sensor, "Read AM2320 sensor", 1024 * 6, NULL, 10, NULL);
    vTaskSuspend(NULL);
}

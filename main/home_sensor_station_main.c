#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "sys/param.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_eth.h"
#include "esp_http_server.h"
#include "lwip/err.h"
#include "lwip/sys.h"



#define DATA_LENGTH 16
#define ACK_CHECK_ENABLED 0x1
#define ACK_CHECK_DISABLED 0x0
#define ACK_VALUE 0x0
#define NACK_VALUE 0x1

#define AM2320_I2C_SENSOR_ADDRESS CONFIG_I2C_AM2320_SENSOR_ADDRESS
#define AM2320_FUNCTION_CODE_READ 0x03
#define AM2320_HUMIDITY_START_REGISTER 0x00
#define AM2320_TEMPERATURE_START_REGISTER 0x02
#define AM2320_NUMBER_OF_REGISTERS_TO_READ 0x02

#define MQ5_DIGITAL_PIN CONFIG_MQ5_DIGITAL_PIN
#define MQ5_ANALOG_PIN CONFIG_MQ5_ANALOG_PIN

#define PIR_DIGITAL_PIN CONFIG_PIR_DIGITAL_PIN

#define PIR_POWER_PIN 12
#define MQ5_POWER_PIN 14

#define GPIO_PIN_SELECT  ((1ULL << MQ5_DIGITAL_PIN) | (1ULL << PIR_DIGITAL_PIN))

#define GPIO_PIN_SELECT_POWER_SOURCE  ((1ULL << MQ5_POWER_PIN) | (1ULL << PIR_POWER_PIN)) 

SemaphoreHandle_t mutex;

//Queues to pass the sensors data
QueueHandle_t temperature_queue;
QueueHandle_t humidity_queue;
QueueHandle_t gas_detection_queue;
QueueHandle_t gas_percentege_queue;
QueueHandle_t movement_detection_queue;

static char server_response[400] = "No data acquired. Try again.";

static const char * TAG = "Read_Sensors_Data";

static EventGroupHandle_t s_wifi_event_group;

const int WIFI_CONNECTED_BIT = BIT0;

static int s_retry_num = 0;

static esp_err_t get_handler(httpd_req_t *req)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    char * resp_str = (char *) req->user_ctx;
    xSemaphoreGive(mutex);
    
    httpd_resp_send(req, resp_str, strlen(resp_str));
    
    return ESP_OK;
}

static const httpd_uri_t sensors_data =
{
    .uri       = "/sensors",
    .method    = HTTP_GET,
    .handler   = get_handler,
    .user_ctx  = &server_response
};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    
    httpd_config_t config = 
    {
        .task_priority      = 12,       
        .stack_size         = 1024*12,                     
        .core_id            = 1,           
        .server_port        = 80,                       
        .ctrl_port          = 32768,                    
        .max_open_sockets   = 7,                        
        .max_uri_handlers   = 8,                        
        .max_resp_headers   = 8,                        
        .backlog_conn       = 5,                       
        .lru_purge_enable   = false,                    
        .recv_wait_timeout  = 5,                        
        .send_wait_timeout  = 5,                        
        .global_user_ctx = NULL,                        
        .global_user_ctx_free_fn = NULL,                
        .global_transport_ctx = NULL,                   
        .global_transport_ctx_free_fn = NULL,           
        .open_fn = NULL,                                
        .close_fn = NULL,                               
        .uri_match_fn = NULL                            
    };

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &sensors_data);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void stop_webserver(httpd_handle_t server)
{
    httpd_stop(server);
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < CONFIG_ESP_WIFI_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        ESP_LOGI(TAG,"connect to the AP fail");
        
        httpd_handle_t* server = (httpd_handle_t*) arg;
        if (*server)
        {
            ESP_LOGI(TAG, "Stopping webserver");
            stop_webserver(*server);
            *server = NULL;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        
        httpd_handle_t* server = (httpd_handle_t*) arg;
        if (*server == NULL)
        {
            ESP_LOGI(TAG, "Starting webserver");
            *server = start_webserver();
        }
    }
}

void setup_server()
{
    static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(nvs_flash_init());
    
    s_wifi_event_group = xEventGroupCreate();
    
    tcpip_adapter_init();
    
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, &server));
    
    wifi_config_t wifi_config =
    {
        .sta =
        {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);    
    
    server = start_webserver();
}

static void hardware_setup()
{
    esp_err_t error_code;
    
    //Setup sensors GPIO
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_PIN_SELECT;
    error_code = gpio_config(&io_conf);
    if (error_code != ESP_OK)
    {
        printf("Sensor GPIO setup error \n");
    }
    
    //Setup power source GPIO for MQ5 and PIR sensors
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_PIN_SELECT_POWER_SOURCE;
    io_conf.pull_up_en = 1;
    error_code = gpio_config(&io_conf);
    if (error_code != ESP_OK)
    {
        printf("Power source GPIO setup error \n");
    }
    
    gpio_set_level(MQ5_POWER_PIN, 1);
    gpio_set_level(PIR_POWER_PIN, 1);

    //Setup ADC
    error_code = adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_0db); 
    if (error_code != ESP_OK)
    {
        printf("ADC setup error \n");
    }
    
    //Setup I2C
    int i2c_master_port = CONFIG_I2C_MASTER_PORT_NUMBER;
    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = CONFIG_I2C_MASTER_SDA;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_io_num = CONFIG_I2C_MASTER_SCL;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = CONFIG_I2C_MASTER_FREQUENCY_IN_HZ;
    i2c_param_config(i2c_master_port, &config);
    error_code = i2c_driver_install(i2c_master_port, config.mode,
                              CONFIG_I2C_MASTER_RX_BUFFER_DISABLE,
                              CONFIG_I2C_MASTER_TX_BUFFER_DISABLE, 0);
    if (error_code != ESP_OK)
    {
        printf("I2C setup error \n");
    }

    //Server setup
    setup_server();
}

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/sensors", req->uri) == 0)
    {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/sensors URI is not available");
        return ESP_OK;
    }
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

static esp_err_t AM2320_wake(i2c_port_t i2c_port_number)
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
    i2c_master_read_byte(cmd, upper_bits_buffer, ACK_VALUE);
    i2c_master_read_byte(cmd, lower_bits_buffer, ACK_VALUE);
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
    float temperature;
    float humidity;
    if (value == 0)
    {    
        AM2320_wake(CONFIG_I2C_MASTER_PORT_NUMBER);
        vTaskDelay(10 / portTICK_RATE_MS);
        AM2320_request(CONFIG_I2C_MASTER_PORT_NUMBER, AM2320_TEMPERATURE_START_REGISTER);
        vTaskDelay(10 / portTICK_RATE_MS);
        AM2320_read(CONFIG_I2C_MASTER_PORT_NUMBER, upper_bits_buffer,
                    lower_bits_buffer, dump_buffer, DATA_LENGTH);
       
        *buffer = *upper_bits_buffer;
        *buffer <<= 8;
        *buffer |= *lower_bits_buffer;
        
        temperature = (float)((int16_t)*buffer)/10.0f;
        xQueueSend(temperature_queue, &temperature, portMAX_DELAY);
    }
    else if (value == 1)
    {   
        AM2320_wake(CONFIG_I2C_MASTER_PORT_NUMBER);
        vTaskDelay(10 / portTICK_RATE_MS);
        AM2320_request(CONFIG_I2C_MASTER_PORT_NUMBER, AM2320_HUMIDITY_START_REGISTER);
        vTaskDelay(10 / portTICK_RATE_MS);
        AM2320_read(CONFIG_I2C_MASTER_PORT_NUMBER, upper_bits_buffer,
                    lower_bits_buffer, dump_buffer, DATA_LENGTH);
        
        *buffer = *upper_bits_buffer;
        *buffer <<= 8;
        *buffer |= *lower_bits_buffer;
        
        humidity = (float)*buffer/10.0f;
        xQueueSend(humidity_queue, &humidity, portMAX_DELAY);
    }
    
}

void AM2320_handle_sensor(void * pvParameters)
{
    vTaskDelay(3000 / portTICK_RATE_MS);
    
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
        readSensor(0, temperature_read, temperature_read_upper_bits, temperature_read_lower_bits, dump_buffer);
        readSensor(1, humidity_read, humidity_read_upper_bits, humidity_read_lower_bits, dump_buffer);
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void MQ5_handle_sensor(void * pvParameters)
{
    vTaskDelay(2000 / portTICK_RATE_MS);
    
    int32_t voltage_read = 0;
    //int32_t maximum_voltage_read = 4096;
    //float gas_percentege = 0.0f;
    bool gas_detection = false;
    
    while (1)
    {
        if (gpio_get_level(MQ5_DIGITAL_PIN))
        {
            gas_detection = true;
            xQueueSend(gas_detection_queue, &gas_detection, portMAX_DELAY);
        }
        else if (!gpio_get_level(MQ5_DIGITAL_PIN))
        {
            gas_detection = false;
            xQueueSend(gas_detection_queue, &gas_detection, portMAX_DELAY);
        }
        
        adc2_get_raw( ADC2_CHANNEL_7, ADC_WIDTH_BIT_12, &voltage_read);
        //gas_percentege = ((float)voltage_read/(float)maximum_voltage_read)*100.0f;
        xQueueSend(gas_percentege_queue, &voltage_read, portMAX_DELAY);
        vTaskDelay(1000 / portTICK_RATE_MS);    
    }
}

void PIR_handle_sensor(void * pvParameters)
{
    vTaskDelay(2000 / portTICK_RATE_MS);
    
    bool movement_detection = false;
    
    while (1)
    {
        if (!gpio_get_level(PIR_DIGITAL_PIN))
        {
            movement_detection = true;
            xQueueSend(movement_detection_queue, &movement_detection, portMAX_DELAY);
        }
        else if (gpio_get_level(PIR_DIGITAL_PIN))
        {
            movement_detection = false;
            xQueueSend(movement_detection_queue, &movement_detection, portMAX_DELAY);
        }
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void manage_sensor_data(void * pvParameters)
{
    vTaskDelay(4000 / portTICK_RATE_MS);
    
    float temperature = 0;
    float humidity = 0;
    bool gas_detection = false;
    float gas_level = 0;
    bool movement_detection = false;
    
    char data_combined[200] = "";
    char buffer[50] = "";
    
    while (1)
    {
        xQueueReceive(temperature_queue, &temperature, portMAX_DELAY);
        xQueueReceive(humidity_queue, &humidity, portMAX_DELAY);
        xQueueReceive(gas_detection_queue, &gas_detection, portMAX_DELAY);
        xQueueReceive(gas_percentege_queue, &gas_level, portMAX_DELAY);
        xQueueReceive(movement_detection_queue, &movement_detection, portMAX_DELAY);
       
        strcpy(data_combined, "");
        
        if (gas_detection)
        {
            printf("WARNING! GAS LEAKAGE DETECTED!\n");
            strcat(data_combined, "WARNING! GAS LEAKAGE DETECTED, ");
        }
        else if (!gas_detection)
        {
            printf("Gas level fine \n");
            strcat(data_combined, "Gas level fine, ");
        }
        if (movement_detection)
        {
            printf("Movement detected \n");
            strcat(data_combined, "Movement detected, ");
        }
        else if (!movement_detection)
        {
            printf("No movement detected \n");
            strcat(data_combined, "No movement detected, ");
        }
        
        sprintf(buffer, "Temperature: %.1f C, Humidity: %.1f %%, Gas level: %.1f", temperature, humidity, gas_level);
        strcat(data_combined, buffer);
        xSemaphoreTake(mutex, portMAX_DELAY);
        strcpy(server_response, data_combined);
        xSemaphoreGive(mutex);
        
        printf("Temperature: %.1f ° \n", temperature);
        printf("Humidity: %.1f %% \n", humidity);
        printf("Gas level: %.1f  \n", gas_level);
        printf("\n");
        vTaskDelay(3000 / portTICK_RATE_MS);    
    }
}

void app_main()
{
    printf("\n");
    
    hardware_setup();

    mutex = xSemaphoreCreateMutex();

    temperature_queue = xQueueCreate(3, sizeof(float));
    humidity_queue = xQueueCreate(3, sizeof(float));
    gas_detection_queue = xQueueCreate(1, sizeof(bool));
    gas_percentege_queue = xQueueCreate(3, sizeof(float));
    movement_detection_queue = xQueueCreate(1, sizeof(bool));

    xTaskCreatePinnedToCore(AM2320_handle_sensor, "Read AM2320 sensor", 1024 * 12, NULL, 12, NULL, 0);
    xTaskCreatePinnedToCore(MQ5_handle_sensor, "Read MQ5 sensor", 1024 * 12, NULL, 11, NULL, 0);
    xTaskCreatePinnedToCore(PIR_handle_sensor, "Read PIR sensor", 1024 * 12, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(manage_sensor_data, "Manage and print sensor data", 1024 * 32, NULL, 9, NULL, 0);
    vTaskSuspend(NULL);
}

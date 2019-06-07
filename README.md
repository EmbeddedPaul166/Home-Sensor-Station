# Home-Sensor-Station

This is a home sensor station with a simple web server to present data. It runs FreeRTOS. It consists of:
- ESP32-DevKitC-32D WiFi + BT 4.2 
- AM2320 (temperature and humidity measurement)
- PIR (movement detection)
- MQ-5 (natural and coal gas detection and measurement)

ESP-IDF together with xtensa-esp32 toolchain were used.

Application is currently under development. Current state:
- AM2320 I2C handling is completed
- MQ5 handling is completed
- PIR handling is completed

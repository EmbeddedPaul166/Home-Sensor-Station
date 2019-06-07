
This is a home sensor station with a simple http web server to present data. It runs FreeRTOS. It consists of:
- ESP32-DevKitC-32D WiFi + BT 4.2 
- AM2320 (temperature and humidity measurement)
- PIR (movement detection)
- MQ-5 (natural and coal gas detection and measurement)

ESP-IDF together with xtensa-esp32 toolchain were used.

Data is presented through serial port and a website in LAN.

Project can be configured with:

make menuconfig

And under "Configuration option" there is an application specific configuration like wi-fi and sensor settings.

Current state of the application is finished. Although there sometimes occurs IllegalInstruction error.
Currently I don't have the means to debug this error and also I need to move to other projects, so this is the final state for now.



|Sample data presentation:|
|------------------------|

| <img src="https://github.com/EmbeddedPaul166/Home-Sensor-Station/edit/master/sample_output.png"> |


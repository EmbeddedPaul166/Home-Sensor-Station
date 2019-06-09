# Home Sensor Station

### Brief description
This is a home sensor station with a simple http web server to present data. It runs FreeRTOS. It consists of:
- ESP32-DevKitC-32D WiFi + BT 4.2 
- AM2320 (temperature and humidity measurement)
- PIR (movement detection)
- MQ-5 (natural and coal gas detection and measurement)
- Two pullup resistors for I2C (AM2320)

Data is presented through serial port and a website in LAN.

### Electrical schematic
<img src="https://github.com/EmbeddedPaul166/Home-Sensor-Station/blob/master/project_schematic.png" height="400">

### How to run
To run this project on your hardware you will need ESP-IDF together with xtensa-esp32 toolchain.

Project can be configured with:
```
make menuconfig
```
And under "Configuration" option there are application-specific configurations like wi-fi and sensor settings.

To build the project and flash it onto the board use:
```
make flash
```

### Sample output
<img src="https://github.com/EmbeddedPaul166/Home-Sensor-Station/blob/master/sample_output_website.png" height="400"> <img src="https://github.com/EmbeddedPaul166/Home-Sensor-Station/blob/master/sample_output_serial.png" height="400">

### Quick note
Application is finished for now. Although there sometimes occurs IllegalInstruction error.
Currently I don't have the means to debug this error and also I need to move to other projects,
so this is the final state for now. I might get back to it sometime later though.





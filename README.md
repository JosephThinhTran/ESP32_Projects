# ESP32_Projects
Fun bare-metal and FreeRTOS-based projects using ESP32 microntrollers.

# ESP32 boards
The projects were tested on the following ESP32 boards:

## [Freenove ESP32-S3-WROOM Board](https://store.freenove.com/products/fnk0085)
This board is more expensive than the official Espressif's DevKit, but it's equipped with camera and SD card socket.


## [ESP32-C6-DEV-KIT-N8](https://www.waveshare.com/wiki/ESP32-C6-DEV-KIT-N8)
This board is basically a clone from the official Espressif's DevKit.

# Projects
## 1. Heart beat measuring and monitoring
### FreeRTOS-based version
- Thread 1: Read sensor data and calculate heart beat.
- Thread 2: Send data to the OLED screen.

### Bare-metal version
Both the sensor data reading, heart beat calculation, and display data on OLED screen are implemented in the "loop" function.

## 2. Multi-threaded ADC reading and monitoring
Three main tasks:
- Read ADC samples per 100ms period.
- Calculate average ADC value over 10 ADC samples.
- Handle the serial terminal to display text and receiving command from user.

## 3. Heart beat monitoring over BLE
TODO:

## 4. Heart beat monitoring over WiFi
TODO:

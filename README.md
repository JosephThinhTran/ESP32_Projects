# ESP32_Projects
Fun bare-metal and FreeRTOS-based projects using ESP32 microntrollers.

# ESP32 boards
The projects were tested on the following ESP32 boards:

## [Freenove ESP32-S3-WROOM Board](https://store.freenove.com/products/fnk0085)
This board is more expensive than the official Espressif's DevKit, but it's equipped with camera and SD card socket.


## [ESP32-C6-DEV-KIT-N8](https://www.waveshare.com/wiki/ESP32-C6-DEV-KIT-N8)
This board is basically a clone from the official Espressif's DevKit.

# Projects
## 1. Heart beat monitoring
### FreeRTOS-based version
- Thread 1: Read sensor data and calculate heart beat.
- Thread 2: Send data to the OLED screen.

### Bare-metal version
Both the sensor data reading, heart beat calculation, and display data on OLED screen are implemented in the "loop" function.

### Additional Hardware Components
- ESP32 board
- [MAX30105 Particle and Pulse Ox Sensor](https://learn.sparkfun.com/tutorials/max30105-particle-and-pulse-ox-sensor-hookup-guide/all#:~:text=This%20example%20runs%20a%20filter,average%20heart%20rate%20(BPM).)
- [Monochrome 0.96" 128x64 OLED Graphic Display ](https://www.adafruit.com/product/326)

## 2. Multi-threaded ADC processing
Three main tasks:
- Read ADC samples (millivolts) per 100ms period.
- Calculate average ADC voltage (millivolt) over 10 ADC samples.
- Handle the serial terminal to display user input characters
  - If receive 'Avg', print out the current average ADC voltage to serial terminal.

Additional tasks:
- Display average ADC voltage to the OLED display as well. This requires [Monochrome 0.96" 128x64 OLED Graphic Display ](https://www.adafruit.com/product/326) connected to the ESP32 board via I2C wires.

## 3. Heart beat monitoring over BLE
TODO:

## 4. Heart beat monitoring over WiFi
TODO:

# ESP32_Projects
Fun bare-metal and FreeRTOS-based Arduino projects using ESP32 microntrollers.

# ESP32 boards
The projects were tested on the following ESP32 boards:

## [Freenove ESP32-S3-WROOM CAM Board](https://store.freenove.com/products/fnk0085)
This board is more expensive than the official Espressif's DevKit, but it's equipped with camera and SD card socket.


## [ESP32-C6-DEV-KIT-N8](https://www.waveshare.com/wiki/ESP32-C6-DEV-KIT-N8)
This board is basically a clone from the official Espressif's DevKit.

# Projects
## 1. Heart beat monitoring
### 1.1 FreeRTOS-based version
- Task 1: Read sensor data and calculate heart beat.
- Task 2: Handle data transfer to OLED screen.
- Task 3: Send average heart rate to the serrial terminal

### 1.2 Bare-metal version
Both the sensor data reading, heart beat calculation, and display data on OLED screen are implemented in the "loop" function.

### Required Library on Arduino IDE
- Adafruit SSD1306 and Adafruit GFX Library for OLED display
- Adafruit BusIO for I2C and SPI interface
- SparkFun MAX3010x and Proximity Sensory Library for MAX3010x sensor

### Required Hardware Components
- [MAX30105 Particle and Pulse Ox Sensor](https://learn.sparkfun.com/tutorials/max30105-particle-and-pulse-ox-sensor-hookup-guide/all#:~:text=This%20example%20runs%20a%20filter,average%20heart%20rate%20(BPM).)
- [Monochrome 0.96" 128x64 OLED Graphic Display ](https://www.adafruit.com/product/326)

#### Wiring MAX3010x sensor breakout board to Freenove ESP32-S3 board
    5V = 5V (3.3V works for the OLED but not the MAX3010x sensor breakout board),
    GND = GND,
    SDA = 19,
    SCL = 20,
    INT = Not connected

#### Wiring MAX3010x sensor breakout board to ESP32-C6-DEV-KIT-N8
    5V = 5V (3.3V works for the OLED but not the MAX3010x sensor breakout board),
    GND = GND,
    SDA = 10,
    SCL = 11,
    INT = Not connected


## 2. Multi-threaded ADC processing
### Three main tasks
- Read ADC samples (millivolts) per 100ms period.
- Calculate average ADC voltage (millivolt) over 10 ADC samples.
- Handle the serial terminal to display user input characters
  - If user types 'Avg', the firmware will print out the current average ADC voltage to serial terminal.

### TODO: Additional tasks:
- Display average ADC voltage to the OLED display as well. This requires [Monochrome 0.96" 128x64 OLED Graphic Display ](https://www.adafruit.com/product/326) connected to the ESP32 board via I2C wires.

## 3. Heart beat monitoring over BLE
### TODO:

## 4. Heart beat monitoring over WiFi
### TODO:

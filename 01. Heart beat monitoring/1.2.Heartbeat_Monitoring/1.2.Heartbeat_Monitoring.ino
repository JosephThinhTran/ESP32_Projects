/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX3010x sensor
  
    This is a demo to show the reading of heart rate or beats per minute (BPM) using
    a Penpheral Beat Amplitude (PBA) algorithm.
    The heart rate value is displayed on the Adafruit_SSD1306 OLED screen

  It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.

  Author: Joseph Thinh Tran

  Hardware Wiring:
  - MAX3010x sensor breakout board to Freenove ESP32-S3 board
    5V = 5V (3.3V works for the OLED but not the MAX3010x sensor breakout board)
    GND = GND
    SDA = 19
    SCL = 20
    INT = Not connected

    - MAX3010x sensor breakout board to ESP32-C6-DEV-KIT-N8
    5V = 5V (3.3V works for the OLED but not the MAX3010x sensor breakout board)
    GND = GND
    SDA = 10
    SCL = 11
    INT = Not connected

  The MAX3010x Breakout can handle 5V or 3.3V I2C logic, but it 5V supply is better.

  Original author: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

*/

/*** ESP32 Board Selection ***/
// #define Freenove_ESP32_S3

/* Waveshare ESP32-C6-DEV-KIT-N8 */
#define ESP32_C6_DEVKIT


#include "Wire.h"
#include <Adafruit_GFX.h>  //OLED  libraries
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include "logo.h"


// I2C pins for Freenove ESP32-S3 board
#if defined(Freenove_ESP32_S3)
  #define I2C_SDA 19
  #define I2C_SCL 20
#elif defined(ESP32_C6_DEVKIT) 
  #define I2C_SDA 10
  #define I2C_SCL 11
#endif  

#define I2C_400KHZ 400000

MAX30105 particleSensor;

const byte RATE_SIZE = 8;  //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];      //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;  //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
bool prevBeatAvg;

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     //32 // OLED display  height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino  reset pin)
#define SCREEN_ADDRESS 0x3D  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  //Declaring the display name (display)

void RenderHeartAndBeat(bool bSmallHeart, int bpm) {

  display.clearDisplay();  //Clear the display

  if (bSmallHeart) {
    display.drawBitmap(5, 5, logo2_bmp, 24, 21, WHITE);  //Draw the first bmp  picture (little heart)
  } else {
    display.drawBitmap(0, 0, logo3_bmp, 32, 32, WHITE);  //Draw  the second picture (bigger heart)
  }

  display.setTextSize(2);  //Near  it display the average BPM you can display the BPM if you want
  display.setTextColor(WHITE);
  display.setCursor(50, 0);
  display.println("BPM");
  display.setCursor(50, 18);
  display.println(bpm);
  display.display();
}

void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println("Heart Rate Measure using ESP32-S3\r\n");

  // Init I2C pinis
  Serial.printf("I2C bus init...");
  if (Wire.begin(I2C_SDA, I2C_SCL)) {
    Serial.println("Done");
  } else {
    Serial.println("FAILED");
    while (1)
      ;
  }

  // Init MAX301x sensor
  Serial.printf("MAX3010x sensor init...");
  if (particleSensor.begin(Wire, I2C_SPEED_FAST))  //400kHz speed
  {
    Serial.println("Done");
  } else {
    Serial.println("MAX3010x was not found. Please check wiring/power. ");
    while (1)
      ;
  }

  particleSensor.setup();                     //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);  //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);   //Turn off Green LED

  // Init OLED display
  Serial.printf("OLED display init...");
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Done");
  } else {
    Serial.println("FAILED");
    while (1)
      ;
  }

  display.display();
  delay(500);

  Serial.println("Place your index finger on the sensor with steady pressure.");
}

void loop() {

  static bool res = false;
  long irValue = particleSensor.getIR();

  if (irValue > 70000) {

    if (res) {
      RenderHeartAndBeat(true, 0);
      res = false;
    }

    if (checkForBeat(irValue) == true) {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60000 / delta;

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;  //Store this reading in the array
        rateSpot %= RATE_SIZE;                     //Wrap variable

        //Take average of readings
        prevBeatAvg = beatAvg;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
        if (beatAvg != prevBeatAvg) {
          RenderHeartAndBeat(true, beatAvg);
          RenderHeartAndBeat(false, beatAvg);
        }
      }
    }

    Serial.print("IR=");
    Serial.print(irValue);
    // Serial.print("BPM=");
    // Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.println(beatAvg);

  } else {  // irValue < 50000 (no finger on the sensor)

    // Serial.print(" No finger?");
    // Serial.println();
    memset((byte *)&rates, 0, RATE_SIZE);
    
    beatAvg = 0;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(30, 5);
    display.println("Please place ");
    display.setCursor(30, 15);
    display.println("your finger ");
    display.display();

    res = true;

  }
}

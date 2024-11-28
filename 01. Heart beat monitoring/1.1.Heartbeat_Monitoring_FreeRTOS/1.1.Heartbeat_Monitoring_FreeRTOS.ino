/*
  Optical Heart Rate Detection (PBA Algorithm) using the MAX3010x sensor
  
    This is a demo to show the reading of heart rate or beats per minute (BPM) using
    a Penpheral Beat Amplitude (PBA) algorithm. The heart rate value is displayed on the Adafruit_SSD1306 OLED screen

    Using FreeRTOS-based multi-task programming
      - Task 1: Collect sensor data, sense heart beat, calculate average heart rate, and store it to the buffer
      - Task 2: Display heart rate to the OLED
      - Task 3: Send debug data to serial terminal

  Note: It is best to attach the sensor to your finger using a rubber band or other tightening
  device. Humans are generally bad at applying constant pressure to a thing. When you
  press your finger against the sensor it varies enough to cause the blood in your
  finger to flow differently which causes the sensor readings to go wonky.

  Author: Joseph Thinh Tran

  Hardware Connections (Breakoutboard to Freenove ESP32-S3):
  -5V = 5V (3.3V works for the OLED but not the MAX3010x sensor breakout board)
  -GND = GND
  -SDA = 19
  -SCL = 20
  -INT = Not connected

  Hardware Connections (Breakoutboard to ESP32-C6-DEV-KIT-N8):
  -5V = 5V (3.3V works for the OLED but not the MAX3010x sensor breakout board)
  -GND = GND
  -SDA = 10
  -SCL = 11
  -INT = Not connected

  The MAX3010x Breakout can handle 5V or 3.3V I2C logic, but it 5V supply is better.

  Original author: Nathan Seidle @ SparkFun Electronics
  Date: October 2nd, 2016
  https://github.com/sparkfun/MAX30105_Breakout

*/

/*** ESP32 Board Selection ***/
#define Freenove_ESP32_S3

/* Waveshare ESP32-C6-DEV-KIT-N8 */
// #define ESP32_C6_DEVKIT


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

// Number of CPU core setting
#if defined(Freenove_ESP32_S3)  // dual core
static const BaseType_t proCpu = 0;
static const BaseType_t appCpu = 1;
#define LED_PIN 2
#elif defined(ESP32_C6_DEVKIT)  // single core
static const BaseType_t proCpu = 0;
static const BaseType_t appCpu = 0;
#define LED_PIN 8
#endif


// Settings
#define I2C_400KHZ 400000

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     //32 // OLED display  height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino  reset pin)
#define SCREEN_ADDRESS 0x3D  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// Declaring the display name (display)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// IR Sensor instances
MAX30105 particleSensor;

const uint8_t RATE_SIZE = 10;     //Increase this for more averaging. 4 is good.
static uint8_t rates[RATE_SIZE];  //Array of heart rates
static uint8_t rateSpot = 0;
static long lastBeat = 0;  //Time at which the last beat occurred

static float beatsPerMinute;
static int beatAvg;
static bool prevBeatAvg;

typedef enum {
  SMALL_HEART = 0,
  BIG_HEART,
  NO_FINGER = 0xFF,
} DISP_CMD;

typedef struct {
  DISP_CMD cmd;  // Display command: small heart, big heart, or no finger detected
  uint8_t bpm;   // beat per minute
} BeatData_t;

#define IR_THRESHOLD 100000

#define MSG_QUEUE_LEN 20
#define MSG_LEN 64  // each block contains up to 64 characters
static QueueHandle_t msgQue;

#define IR_QUEUE_LEN 16
static QueueHandle_t gIRQue;  // Queue of raw data from IR sensor (long int)

#define BEAT_QUEUE_LEN 16
static QueueHandle_t gBeatQue;  // Queue of beat data

// Task handles
static TaskHandle_t hdlGetIR = NULL;
static TaskHandle_t hdlCalcBeat = NULL;
static TaskHandle_t hdlDisplayBeat = NULL;
static TaskHandle_t hdlDoCli = NULL;

static SemaphoreHandle_t mux;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

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

// Init I2C pins
bool initI2CPins(int sda, int scl) {

  char msg[MSG_LEN];
  int len;

  if (Wire.begin(I2C_SDA, I2C_SCL)) {

    sprintf(msg, "I2C bus init...Done\r\n");
    xQueueSend(msgQue, msg, 10);

  } else {

    sprintf(msg, "I2C bus init...FAILED\r\n");
    xQueueSend(msgQue, msg, 10);
    return false;
  }

  return true;
}

// Init MAX301x sensor
bool initMAX3010xSensor() {

  char msg[MSG_LEN];
  int len;

  len = sprintf(msg, "MAX3010x sensor init...");
  if (particleSensor.begin(Wire, I2C_SPEED_FAST))  //400kHz speed
  {
    sprintf(msg, "MAX3010x sensor init...Done\r\n");
    xQueueSend(msgQue, msg, 10);
  } else {
    sprintf(msg, "MAX3010x was not found. Please check wiring/power\r\n");
    xQueueSend(msgQue, msg, 10);
    return false;
  }

  particleSensor.setup();                     //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);  //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);   //Turn off Green LED

  return true;
}

// Init OLED display
bool initOLED(uint8_t addr) {

  char msg[MSG_LEN];
  int len;

  if (display.begin(SSD1306_SWITCHCAPVCC, addr)) {
    sprintf(msg, "OLED display init...Done\r\n");
    xQueueSend(msgQue, msg, 10);
  } else {
    sprintf(msg, "OLED display init...FAILED\r\n");
    xQueueSend(msgQue, msg, 10);
    return false;
  }

  display.display();

  return true;
}

/************************* Tasks *************************************/
// Task 1: Collect sensor data, sense heart beat, and store it to the buffer
void getIRSamples(void* param) {

  char msg[MSG_LEN];
  BeatData_t beatData;
  static bool rstScreen = false;
  static bool bFingerDetect = false;
  static bool flagOnce = false;

  while (1) {

    // get IR sample
    long irValue = particleSensor.getIR();

    if (irValue > IR_THRESHOLD) {

      bFingerDetect = true;
      flagOnce = false;

      // Reset screen
      if (rstScreen) {

        beatData.cmd = SMALL_HEART;
        beatData.bpm = 0;
        if (xQueueSend(gBeatQue, (void*)&beatData, 10) == pdFALSE) {
          sprintf(msg, "WARNING: Display buffer full\r\n");
          xQueueSend(msgQue, (void*)msg, 10);
        }

        rstScreen = false;
      }

      // Detect heart beat
      if (checkForBeat(irValue)) {

        long delta = xTaskGetTickCount() * portTICK_PERIOD_MS - lastBeat;
        lastBeat = xTaskGetTickCount() * portTICK_PERIOD_MS;

        beatsPerMinute = 60000 / delta;

        if (beatsPerMinute < 255 && beatsPerMinute) {
          rates[rateSpot++] = (uint8_t)beatsPerMinute;
          rateSpot %= RATE_SIZE;

          prevBeatAvg = beatAvg;
          int tot = 0;
          for (int i = 0; i < RATE_SIZE; i++) {
            tot += rates[i];
          }
          xSemaphoreTake(mux, portMAX_DELAY);
          beatAvg = tot / RATE_SIZE;
          xSemaphoreGive(mux);

          if (beatAvg != prevBeatAvg) {
            beatData.cmd = SMALL_HEART;
            beatData.bpm = beatAvg;
            if (xQueueSend(gBeatQue, (void*)&beatData, 10) == pdFALSE) {
              sprintf(msg, "WARNING: Display buffer full\r\n");
              xQueueSend(msgQue, (void*)msg, 10);
            }

            sprintf(msg, "%d bpm\r\n", beatAvg);
            xQueueSend(msgQue, (void*)msg, 10);
          }
        }
      }
    } else {  // Finger not detected

      // Reset array
      memset((uint8_t*)&rates, 0, RATE_SIZE);

      // Reset screen
      rstScreen = true;

      // Reset gloval average beat
      xSemaphoreTake(mux, portMAX_DELAY);
      beatAvg = 0;
      xSemaphoreGive(mux);

      // Display on OLED
      beatData.cmd = NO_FINGER;
      beatData.bpm = 0;
      if (xQueueSend(gBeatQue, (void*)&beatData, 10) == pdFALSE) {
        sprintf(msg, "WARNING: Display buffer full\r\n");
        xQueueSend(msgQue, (void*)msg, 10);
      }

      // Send to terminal only one time
      bFingerDetect = false;
      if (bFingerDetect == false) {
        if (flagOnce == false) {
          sprintf(msg, "No finger\r\n");
          xQueueSend(msgQue, (void*)msg, 10);
          flagOnce = true;
        }
      }
    }
  }
}

// Task 2: Calculate average heart rate, store to a queue
void calcBeat(void* param) {

  // Serial.printf("calcBeat task created\r\n");

  long irValue;
  char msg[MSG_LEN];
  BeatData_t beatData;
  static bool rstScreen = false;
  static bool bFingerDetect = false;

  while (1) {

    // Get data from IR queue and process
    if (xQueueReceive(gIRQue, (void*)&irValue, 0) == pdTRUE) {

      if (irValue < IR_THRESHOLD) {  // No finger detected

        // Reset the average array
        memset((uint8_t*)&rates, 0, RATE_SIZE);

        // Reset display
        rstScreen = true;

        // Reset global average beat
        xSemaphoreTake(mux, portMAX_DELAY);
        beatAvg = 0;
        xSemaphoreGive(mux);

        // Display on OLED
        beatData.cmd = NO_FINGER;
        beatData.bpm = 0;
        if (xQueueSend(gBeatQue, (void*)&beatData, 10) == pdFALSE) {
          sprintf(msg, "WARNING: Display buffer full\r\n");
          xQueueSend(msgQue, (void*)msg, 10);
        }

        // Send to terminal only one time
        if (bFingerDetect == false) {
          sprintf(msg, "No finger detected\r\n");
          xQueueSend(msgQue, (void*)msg, 10);
          bFingerDetect = true;  // display only one time
        }

      } else {  // Display heart beat to OLED

        // Finger is detected
        bFingerDetect = false;

        // Reset screen
        if (rstScreen) {
          beatData.cmd = SMALL_HEART;
          beatData.bpm = 0;
          if (xQueueSend(gBeatQue, (void*)&beatData, 10) == pdFALSE) {
            sprintf(msg, "WARNING: Display buffer full\r\n");
            xQueueSend(msgQue, (void*)msg, 10);
          }
        }

        // A beat is sensed
        long delta = xTaskGetTickCount() * portTICK_PERIOD_MS - lastBeat;
        lastBeat = xTaskGetTickCount() * portTICK_PERIOD_MS;

        beatsPerMinute = 60000 / delta;
        sprintf(msg, "beatsPerMinute %f \r\n", beatsPerMinute);
        xQueueSend(msgQue, (void*)msg, 10);

        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
          rates[rateSpot++] = (uint8_t)beatsPerMinute;  // Store this reading in the array
          rateSpot %= RATE_SIZE;                        // Wrap variable

          // Average of bpm reading
          portENTER_CRITICAL(&spinlock);
          prevBeatAvg = beatAvg;
          portEXIT_CRITICAL(&spinlock);

          int tot = 0;
          for (int i = 0; i < RATE_SIZE; i++) {
            tot += rates[i];
          }

          xSemaphoreTake(mux, portMAX_DELAY);
          beatAvg = tot / RATE_SIZE;
          xSemaphoreGive(mux);
          sprintf(msg, "%d bpm\r\n", beatAvg);
          xQueueSend(msgQue, (void*)msg, 10);

          if (beatAvg != prevBeatAvg) {

            beatData.cmd = SMALL_HEART;
            beatData.bpm = beatAvg;
            if (xQueueSend(gBeatQue, (void*)&beatData, 10) == pdFALSE) {
              sprintf(msg, "WARNING: Display buffer full\r\n");
              xQueueSend(msgQue, (void*)msg, 10);
            }
          }
        }
      }
    }
  }
}

// Task 3: Display heart rate to the OLED
void displayBeat(void* param) {

  BeatData_t beatData;

  while (1) {
    // Get data from beat queue
    if (xQueueReceive(gBeatQue, (void*)&beatData, 0) == pdTRUE) {

      if (beatData.cmd == NO_FINGER) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(30, 5);
        display.println("Please place ");
        display.setCursor(30, 15);
        display.println("your finger ");
        display.display();

      } else {

        RenderHeartAndBeat(false, beatData.bpm);
        if (beatData.bpm > 0) {
          vTaskDelay(10 / portTICK_PERIOD_MS);
          RenderHeartAndBeat(true, beatData.bpm);
        }
      }
    }
  }
}

// Task 4: Send debug data to serial terminal
void doCli(void* param) {

  char msg[MSG_LEN];
  while (1) {

    if (xQueueReceive(msgQue, (void*)msg, 10) == pdTRUE) {
      Serial.print(msg);
    }
  }
}

void setup() {

  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("Heart Rate Measure using ESP32-S3\r\n");

  // Create queues
  msgQue = xQueueCreate(MSG_QUEUE_LEN, MSG_LEN);
  gIRQue = xQueueCreate(IR_QUEUE_LEN, sizeof(long int));
  gBeatQue = xQueueCreate(BEAT_QUEUE_LEN, sizeof(BeatData_t));
  
  // Init I2C, MAX3010x sensor, and OLED
  // Should init after creating message queue
  bool rc1 = initI2CPins(I2C_SDA, I2C_SCL);
  bool rc2 = initMAX3010xSensor();
  bool rc3 = initOLED(SCREEN_ADDRESS);

  char msg[MSG_LEN];
  if (rc1 && rc2 && rc3) {
    sprintf(msg, "Place your index finger on the sensor with steady pressure.\r\n");
  } else {
    sprintf(msg, "Check your hardware again\r\n");
  }
  xQueueSend(msgQue, msg, 10);

  // Mutex and semaphore
  mux = xSemaphoreCreateMutex();

  // Tasks
  xTaskCreatePinnedToCore(getIRSamples, "Get IR samples", 3072, NULL, 2, &hdlGetIR, appCpu);
  //xTaskCreatePinnedToCore(calcBeat, "Calc beat", 3072, NULL, 1, &hdlCalcBeat, appCpu);
  xTaskCreatePinnedToCore(displayBeat, "Display OLED", 3072, NULL, 1, &hdlDisplayBeat, appCpu);
  xTaskCreatePinnedToCore(doCli, "Do CLI", 2048, NULL, 1, &hdlDoCli, appCpu);

  // Delete self tasks
  vTaskDelete(NULL);
}

void loop() {
  // Do nothing
}

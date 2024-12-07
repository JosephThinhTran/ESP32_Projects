/***
This project will do the following tasks:
- Retrieve ADC sample per 100ms
- Calculate average ADC value per 10 samples
- Handle serial terminal for:
  + Echo back the user's input
  + If receiving "Avg", then print out the current average ADC value to the terminal

The implementation is in multi-thread paradigm.
Hardware timer is used in this implementation.

This project is inspired by: 
https://www.youtube.com/watch?v=qsflCf6ahXU&list=PLEBQazB0HUyQ4hAPU1cJED6t3DU0h34bz&index=9

***/
// Board definition - Only one is defined
// #define ESP32_S3_FREENOVE_DK
// #define ESP32_C6_DK
#define ESP32_DK

#ifdef ESP32_S3_FREENOVE_DK
  // Using only 1 core for all the tasks
  #ifdef CONFIG_FREERTOS_UNICORE
  static const BaseType_t appCpu = 0;
  #else
  static const BaseType_t appCpu = 1;
  #endif // CONFIG_FREERTOS_UNICORE

#elif defined(ESP32_C6_DK)   // Single-core ESP32 chip
static const BaseType_t appCpu = 0;

#elif defined(ESP32_DK)
  // Using only 1 core for all the tasks
  #ifdef CONFIG_FREERTOS_UNICORE
  static const BaseType_t appCpu = 0;
  #else
  static const BaseType_t appCpu = 1;
  #endif
#endif  // Board def

// Pin Settings
#ifdef ESP32_S3_FREENOVE_DK
#define ADC_PIN 11
#define LED_PIN 2
#elif defined(ESP32_C6_DK)
#define ADC_PIN 2
#define LED_PIN 8
#elif defined(ESP32_DK)
#define ADC_PIN 15
#define LED_PIN 2
#endif  // Board

// Settings
#define TIMER_FREQ 1000000    // 1MHz timer
#define TIMER_MAX_CNT 100000  // Number of timer ticks, here equivalent to 100ms period

#define MSG_LEN 100
#define MSG_QUE_LEN 10

const char cmd[] = "Avg";
const uint8_t cmdLen = strlen(cmd);

#define BUF_LEN 10
#define NUM_BANKS 2

typedef struct {
  int data[NUM_BANKS][BUF_LEN];  // data banks
  uint16_t bIdx;                 // current bank index
  uint16_t wIdx;                 // current write index
} DualBuf_t;

// Global vars
static hw_timer_t* hwTimer = NULL;
static QueueHandle_t msgQue;

static TaskHandle_t hdlDoCli, hdlProcAdc;

static float gAvgAdc;

static DualBuf_t gDBuf = {
  .bIdx = 0,
  .wIdx = 0,
};

static bool gProcDone = true;

SemaphoreHandle_t semProcDone;
portMUX_TYPE spinLock = portMUX_INITIALIZER_UNLOCKED;

SemaphoreHandle_t semNoti;// binary semaphore for timerISR to notify procAdc task (If we do not use vTaskNotify)

// Timer ISR using for capturing ADC sample
void IRAM_ATTR timerISR(void) {
  BaseType_t taskWoken = pdFALSE;

  //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  int sample = analogReadMilliVolts(ADC_PIN);

  // Store sample to the buffer
  if (gDBuf.wIdx < BUF_LEN && gProcDone) {
    gDBuf.data[gDBuf.bIdx][gDBuf.wIdx] = sample;
    gDBuf.wIdx++;
  }

  // If the buffer is full
  if (gDBuf.wIdx >= BUF_LEN) {
    // Check if procAdc task is done processing
    if (xSemaphoreTakeFromISR(semProcDone, &taskWoken) == pdFALSE) {
      gProcDone = false;
    }

    // Notify procAdc task if it was done processing data
    if (gProcDone == true) {
      // Switch buffer banks
      gDBuf.bIdx = (gDBuf.bIdx + 1) % NUM_BANKS;
      gDBuf.wIdx = 0;

      // Notify procAdc task
      //vTaskNotifyGiveFromISR(hdlProcAdc, &taskWoken);
      xSemaphoreGiveFromISR(semNoti, &taskWoken);
    }
  }
}

// Task 1: Process ADC samples
void procAdc(void*) {

  // Timer creation
  hwTimer = timerBegin(TIMER_FREQ);
  timerAttachInterrupt(hwTimer, &timerISR);
  timerAlarm(hwTimer, TIMER_MAX_CNT, true, 0);

  float tot, avg;
  char msg[MSG_LEN];

  while (1) {

    // Wait for notification from ISR
    //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    xSemaphoreTake(semNoti, portMAX_DELAY);

    // Calculate average ADC
    uint16_t bIdx = (gDBuf.bIdx + 1) % NUM_BANKS;
    tot = 0;
    for (int i = 0; i < BUF_LEN; i++) {
      tot += gDBuf.data[bIdx][i];
      //vTaskDelay(105 / portTICK_PERIOD_MS);// Uncomment to see the buffer overrun
    }
    avg = tot / BUF_LEN;

    // Update gAvgAdc
    portENTER_CRITICAL(&spinLock);
    gAvgAdc = avg;
    portEXIT_CRITICAL(&spinLock);

    // Check if buffer overrun issue happens
    if (gProcDone == false) {
      sprintf(msg, "Buffer overrun! ADC sample dropped!\n");
      xQueueSend(msgQue, msg, 10);
    }

    // Set the gProcDone to true
    portENTER_CRITICAL(&spinLock);
    gProcDone = true;
    xSemaphoreGive(semProcDone);
    portEXIT_CRITICAL(&spinLock);
  }
}

// Task 2: Handle serial terminal
void doCli(void* param) {
  char c;
  char buf[MSG_LEN];
  uint8_t idx;

  char msg[MSG_LEN];

  // Reset the buf
  memset(buf, 0, MSG_LEN);
  idx = 0;

  while (1) {
    // Print out a message in the message queue
    if (xQueueReceive(msgQue, msg, 0) == pdTRUE) {
      Serial.print(msg);
    }

    // Check user's input from serial terminal
    if (Serial.available() > 0) {
      c = Serial.read();

      // Store the char to the buffer
      if (idx < MSG_LEN - 1) {
        buf[idx] = c;
        idx++;
      }

      // Check if user hits enter
      if (c == '\n' || c == '\r') {
        Serial.print("\n");

        // Check if 'cmd' is received
        if (memcmp(buf, cmd, cmdLen) == 0) {
          Serial.print("Average ADC: ");
          Serial.print(gAvgAdc);
          Serial.print(" mV\n");
        }

        // Reset the buf
        memset(buf, 0, MSG_LEN);
        idx = 0;

        // Echo back the char
      } else {
        Serial.print(c);
      }
    }

    //vTaskDelay(1 / portTICK_PERIOD_MS);// Uncomment this line if doCli task has higher priority than procAdc task
  }
}

void setup() {

  pinMode(LED_PIN, OUTPUT);

  // UART settings
  Serial.begin(115200);
  vTaskDelay(500 / portTICK_PERIOD_MS);  // wait a little bit
  Serial.print("---Muti-threaded ADC monitoring using HW timer---\n");

  // Queues and semaphores
  msgQue = xQueueCreate(MSG_QUE_LEN, MSG_LEN);

  // Semaphores
  semProcDone = xSemaphoreCreateBinary();
  xSemaphoreGive(semProcDone);  // procAdc task is done at the begin

  semNoti = xSemaphoreCreateBinary();

  // Task creation
  xTaskCreatePinnedToCore(doCli, "Handle serial terminal", 2500, NULL, 1, &hdlDoCli, appCpu);
  xTaskCreatePinnedToCore(procAdc, "Process ADC", 2500, NULL, 2, &hdlProcAdc, appCpu);

  // Delete self tasks (setup and loop)
  vTaskDelete(NULL);
}

void loop() {
  // do nothing
}

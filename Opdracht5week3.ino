#include "esp_sntp.h"
#include "WiFi.h"
#include "secret.h"

// contants for SNTP client
const char* ntpServer = "nl.pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

// LED 
#define GPIO_LED 12
#define GPIO_BUTTONL 27
#define GPIO_BUTTONR 26

static QueueHandle_t queue;
static const int reset_press = -998;
static QueueHandle_t audit_queue;

char* printLocalTime()
{
  //https://docs.espressif.com/projects/esp-idf/en/latest/esp32/apireference/system/system_time.html
  time_t now;
  char *time_buf = (char*)malloc(64 * sizeof(char));
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  strftime(time_buf, (64 * sizeof(char)), "%c", &timeinfo);
  //ESP_LOGI(TAG, "\r\nThe current date/time is: %s\r\n", time_buf);
  Serial.println(time_buf);
  return time_buf;
}

// Nieuw geadded - NTP en Wifi handler
static void wifi_task(void *testptr) {
  //WIFI
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");
  Serial.println("SSID: " + WiFi.SSID());

}





// NTP tester
static void ntp_task(void *testptr) {
  // init time protocol sync
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_init();
  setenv("TZ", "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00", 1);
  tzset();
}

// Button Debouncing task
static void debounce_task(void *argp) {
  unsigned button_gpio = *(unsigned*)argp;
  uint32_t level, state = 0;
  uint32_t mask = 0x7FFFFFFF;
  int event, last = -999;

  for (;;) {
    level = !digitalRead(button_gpio);
    state = (state << 1) | level;
    if ( (state & mask) == mask )
    event = button_gpio; // Press

    else
      event = -button_gpio; // Release

    if ( event != last ) {
      if ( xQueueSendToBack(queue,&event,0) == pdPASS ) {
        last = event;
    } else if ( event < 0 ) {
      do {
        xQueueReset(queue); // Empty queue
      } while ( xQueueSendToBack(queue,&reset_press,0) != pdPASS );

      last = event;
      }
    }
  taskYIELD();
  }
}



// Hydraulic Press
static void press_task(void *argp) {
    static const uint32_t enable = (1 << GPIO_BUTTONL)
    | (1 << GPIO_BUTTONR);
    BaseType_t s;
    int event;
    uint32_t state = 0;
    digitalWrite(GPIO_LED,LOW);

    for (;;) {
      s = xQueueReceive(
        queue,
        &event,
        portMAX_DELAY
      );

      assert(s == pdPASS);

      if ( event == reset_press ) {
      digitalWrite(GPIO_LED,LOW);
      state = 0; 
      printf("RESET!!\n");
        continue;
      }

      if ( event >= 0 ) {
        // Button press
        state |= 1 << event;
      } else {
        // Button release
        state &= ~(1 << -event);
      }
      
      if ( state == enable ) {
        digitalWrite(GPIO_LED,HIGH);
        char *audit = (char*)malloc(64 * sizeof(char));
        audit = printLocalTime();
        if ( xQueueSendToBack(audit_queue,&audit,0) == pdPASS ) {
        xQueueSendToBack(audit_queue,&audit,0);
        } else {
          char *lose = (char*)malloc(64 * sizeof(char));
          xQueueReceive(audit_queue, &lose, 0);
          xQueueSendToBack(audit_queue,&audit,0);
        }
      } else {
      digitalWrite(GPIO_LED,LOW);
    }
  }
}



// Initialization
void setup() {
    //Serial.begin(115200);
    //WiFi.begin(ssid, password);
    //while (WiFi.status() != WL_CONNECTED) {
    //delay(500);
    //Serial.println("Connecting to WiFi..");
    //}

    // Serial.println("Connected to the WiFi network");
    int app_cpu = xPortGetCoreID();
    static int left = GPIO_BUTTONL;
    static int right = GPIO_BUTTONR;

    
    // NTP en Wifi taskhandler
    TaskHandle_t WIFI_TASK;
    TaskHandle_t NTP_TASK;

    TaskHandle_t h;
    BaseType_t rc;


    delay(2000); 
    queue = xQueueCreate(200,sizeof(int));
    assert(queue);

    char *TimeBuf = (char*)malloc(64 * sizeof(char));
    audit_queue = xQueueCreate(10, sizeof(TimeBuf));

    assert(audit_queue);
    pinMode(GPIO_LED,OUTPUT);
    pinMode(GPIO_BUTTONL,INPUT_PULLUP);
    pinMode(GPIO_BUTTONR,INPUT_PULLUP);

    // TASK wifi
    rc = xTaskCreatePinnedToCore(
      wifi_task,
      "wifitask",
      2048, // Stack size
      nullptr, // Geen gpio
      3, // Priority
      &WIFI_TASK, // Task handle
      app_cpu  // CPU
      );

    // TASK ntp
    rc = xTaskCreatePinnedToCore(
      ntp_task,
      "ntptask",
      2048, // Stack size
      nullptr, // geen gpio
      2, // Priority
      &NTP_TASK, // Task handle
      app_cpu // CPU
      );

    rc = xTaskCreatePinnedToCore(
      debounce_task,
      "debounceL",
      2048, // Stack size
      &left, // geen
      1, // Priority
      &h, // Task handle
      app_cpu // CPU
      );

      assert(rc == pdPASS);
      assert(h);

    rc = xTaskCreatePinnedToCore(
      debounce_task,
      "debounceR",
      2048, 
      &right, 
      1, 
      &h, 
      app_cpu 
      );
      assert(rc == pdPASS);
      assert(h);

    rc = xTaskCreatePinnedToCore(
      press_task,
      "led",
      2048, 
      nullptr, 
      1,
      &h, 
      app_cpu 
      );
      assert(rc == pdPASS);
      assert(h);

  vTaskStartScheduler();
}

void loop() {
  vTaskDelete(nullptr);
}

#define BLYNK_TEMPLATE_ID "TMPL4feyxKpfB"
#define BLYNK_TEMPLATE_NAME "SmartIncubator"
#define BLYNK_AUTH_TOKEN "C6HTasYHrdwDOEVBeiKyQX5Q3y4HrKIs"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// =====================================================
// WIFI
// =====================================================

char ssid[] = "Laboospace";
char pass[] = "laboo2026";

// =====================================================
// PINS
// =====================================================

#define DHT_PIN      16
#define DHT_TYPE     DHT22
#define HEATER_PIN   15   // lamp = heat source
// true  = lamp/LED driven directly or through a transistor (HIGH = ON)
// false = lamp on a 5V relay module like the fan (LOW = ON, floating = OFF)
#define HEATER_DIRECT true
#define FAN_RELAY    23
#define BUZZER_PIN   19
#define SERVO_PIN    18   // egg-turning servo

// Relays are active LOW
#define RELAY_ON  LOW
#define RELAY_OFF HIGH

DHT dht(DHT_PIN, DHT_TYPE);
Servo eggServo;

// =====================================================
// OLED (I2C, shared bus with RTC)
// =====================================================

#define SDA_PIN      21
#define SCL_PIN      22
#define OLED_ADDR    0x3C
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOk = false;

// =====================================================
// INCUBATION SETTINGS
// =====================================================

#define TOTAL_DAYS        21
#define LOCKDOWN_DAY      18
#define HEATER_HYST       0.2     // heater ON below (setpoint - 0.2), OFF at setpoint
#define HUM_SETTING_MAX   55.0    // days 1-17
#define HUM_LOCKDOWN_MAX  70.0    // days 18-21
#define HUM_HYST          3.0     // fan OFF once humidity drops 3% below the limit

// Egg turning: continuous-rotation servo (same values as your test code)
#define SERVO_SPIN_US     1200     // rotate
#define SERVO_STOP_US     1500     // stop
#define TURN_SPIN_MS      10000UL  // rotate for 10 s
#define TURN_INTERVAL_MS  20000UL  // start a turn every 20 s (10 s spin + 10 s stop)

// DEMO: set DAY_LENGTH_MS to 120000UL (2 min) to reach lockdown quickly
#define DAY_LENGTH_MS     (24UL * 60UL * 60UL * 1000UL)   // 24 hours

// pdMS_TO_TICKS overflows for values this large, so convert directly
#define LONG_MS_TO_TICKS(ms) ((TickType_t)((ms) / portTICK_PERIOD_MS))

// =====================================================
// SAFETY SETTINGS
// =====================================================

#define TEMP_SAFE_MIN    36.5
#define TEMP_SAFE_MAX    38.5
#define TEMP_CUTOFF      39.5                    // heater forced OFF immediately
#define HUM_SAFE_MIN     40.0
#define HUM_SAFE_MAX     80.0
#define ALARM_GRACE_MS   120000UL                // FR7: 2 minutes
#define SENSOR_GRACE_MS  5000UL                  // ignore a single bad DHT read
#define STALE_MS         3000UL                  // reading older than this = fault
#define WARMUP_MS        (20UL * 60UL * 1000UL)  // no low-temp alarm while warming up

// =====================================================
// FREE RTOS OBJECTS
// =====================================================

struct SensorData_t {
  float temperature;
  float humidity;
  uint32_t timestamp;
  bool valid;
};

QueueHandle_t sensorQueue;        // latest reading (length 1, overwrite)

SemaphoreHandle_t heaterMutex;    // guards relay states + control settings
SemaphoreHandle_t displayMutex;   // guards I2C bus / OLED
SemaphoreHandle_t alarmSemaphore; // watchdog -> Blynk notification path
SemaphoreHandle_t motorMutex;     // guards the servo

TimerHandle_t dayTimer;           // incubation day counter
TimerHandle_t turnTimer;          // periodic egg turn

TaskHandle_t eggTurnTaskHandle;

// =====================================================
// SHARED STATE
// =====================================================

// Protected by heaterMutex
volatile bool manualMode = false;
bool heaterState = false;
bool fanState = false;
float temperatureSetPoint = 37.5;

// Single-writer flags (32-bit reads/writes are atomic on ESP32)
volatile int incubationDay = 1;
volatile bool sensorFault = true;   // true until first good reading
volatile bool overTemp = false;
volatile bool turning = false;     // motor state, changed only while holding motorMutex

enum AlarmCode { ALARM_NONE, ALARM_SENSOR, ALARM_OVERTEMP, ALARM_TEMP, ALARM_HUM };
const char *alarmText[] = {
  "Normal",
  "Sensor fault",
  "OVER-TEMPERATURE - heater cut",
  "Temperature out of range",
  "Humidity out of range"
};
volatile uint8_t alarmCode = ALARM_NONE;

// Network status for the OLED (written only by BlynkSyncTask)
// 0 = no WiFi, 1 = WiFi but no Blynk, 2 = Blynk online
volatile uint8_t netState = 0;

// =====================================================
// HELPERS
// =====================================================

bool isLockdown() {
  return incubationDay >= LOCKDOWN_DAY;
}

void getControlValues(bool &manual, float &setPoint, float &humidityLimit) {
  manual = false;
  setPoint = 37.5;

  if (xSemaphoreTake(heaterMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    manual = manualMode;
    setPoint = temperatureSetPoint;
    xSemaphoreGive(heaterMutex);
  }

  // FR3: humidity target switches at lockdown
  humidityLimit = isLockdown() ? HUM_LOCKDOWN_MAX : HUM_SETTING_MAX;
}

// =====================================================
// HEATER CONTROL
// =====================================================

void heaterOn() {
  if (xSemaphoreTake(heaterMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (HEATER_DIRECT) {
      digitalWrite(HEATER_PIN, HIGH);
    } else {
      pinMode(HEATER_PIN, OUTPUT);
      digitalWrite(HEATER_PIN, LOW);
    }
    heaterState = true;
    xSemaphoreGive(heaterMutex);
  }
}

void heaterOff() {
  if (xSemaphoreTake(heaterMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (HEATER_DIRECT) {
      digitalWrite(HEATER_PIN, LOW);
    } else {
      pinMode(HEATER_PIN, INPUT);
    }
    heaterState = false;
    xSemaphoreGive(heaterMutex);
  }
}

// =====================================================
// FAN CONTROL
// =====================================================

void fanOn() {
  if (xSemaphoreTake(heaterMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Your existing fan relay workaround
    pinMode(FAN_RELAY, OUTPUT);
    digitalWrite(FAN_RELAY, LOW);
    fanState = true;
    xSemaphoreGive(heaterMutex);
  }
}

void fanOff() {
  if (xSemaphoreTake(heaterMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Your existing fan relay workaround
    pinMode(FAN_RELAY, INPUT);
    fanState = false;
    xSemaphoreGive(heaterMutex);
  }
}

// =====================================================
// TASK 1: SENSOR READ (1 s, priority 3)
// =====================================================

void SensorReadTask(void *pvParameters) {
  SensorData_t data;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    data.temperature = dht.readTemperature();
    data.humidity = dht.readHumidity();
    data.timestamp = millis();
    data.valid = !isnan(data.temperature) && !isnan(data.humidity);

    xQueueOverwrite(sensorQueue, &data);

    if (data.valid) {
      Serial.printf("[Sensor] Temperature: %.2f C | Humidity: %.2f %%\n",
                    data.temperature, data.humidity);
    } else {
      Serial.println("[Sensor] DHT22 READ FAILED");
    }

    // NFR1: fixed 1 s period (no drift)
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
  }
}

// =====================================================
// TASK 2: HEATER CONTROL (1 s, priority 3)
// =====================================================

void HeaterCtrlTask(void *pvParameters) {
  SensorData_t data;

  for (;;) {
    bool manual;
    float setPoint, humidityLimit;
    getControlValues(manual, setPoint, humidityLimit);

    if (!manual) {
      bool haveData = xQueuePeek(sensorQueue, &data, 0) == pdTRUE;

      // Fail-safe: no data, bad data, sensor fault or over-temp -> OFF
      if (!haveData || !data.valid || sensorFault || overTemp) {
        heaterOff();
      }
      else if (data.temperature < setPoint - HEATER_HYST) {
        heaterOn();
      }
      else if (data.temperature >= setPoint) {
        heaterOff();
      }
      // Between the two thresholds: keep current state (hysteresis)
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// =====================================================
// TASK 3: HUMIDITY / FAN CONTROL (2 s, priority 2)
// =====================================================

void HumidityCtrlTask(void *pvParameters) {
  SensorData_t data;

  for (;;) {
    bool manual;
    float setPoint, humidityLimit;
    getControlValues(manual, setPoint, humidityLimit);

    if (!manual) {
      bool haveData = xQueuePeek(sensorQueue, &data, 0) == pdTRUE;

      if (!haveData || !data.valid || sensorFault) {
        fanOff();
      }
      else if (data.humidity > humidityLimit) {
        fanOn();
      }
      else if (data.humidity < humidityLimit - HUM_HYST) {
        fanOff();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// =====================================================
// TIMER CALLBACK (must not block)
// =====================================================

void turnTimerCallback(TimerHandle_t xTimer) {
  // FR4: turning stops from lockdown day
  if (!manualMode && !isLockdown()) {
    xTaskNotifyGive(eggTurnTaskHandle);
  }
}

void dayTimerCallback(TimerHandle_t xTimer) {
  if (incubationDay < TOTAL_DAYS) {
    incubationDay = incubationDay + 1;
  }
  Serial.printf("[Timer] Incubation day: %d\n", incubationDay);
}

// =====================================================
// TASK: EGG TURN (event-driven by turnTimer, priority 2)
// =====================================================

void EggTurnTask(void *pvParameters) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // wait for timer

    if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
      turning = true;
      Serial.println("[EggTurn] Rotating...");
      eggServo.writeMicroseconds(SERVO_SPIN_US);

      vTaskDelay(pdMS_TO_TICKS(TURN_SPIN_MS));   // NFR7: <= 10 s

      eggServo.writeMicroseconds(SERVO_STOP_US);
      turning = false;
      Serial.println("[EggTurn] Stopped.");
      xSemaphoreGive(motorMutex);
    }
  }
}

// =====================================================
// TASK 4: SAFETY WATCHDOG (500 ms, priority 4 = HIGHEST)
// =====================================================

void SafetyWatchdogTask(void *pvParameters) {
  SensorData_t data;
  uint32_t sensorBadSince = 0;
  uint32_t tempBadSince = 0;
  uint32_t humBadSince = 0;
  uint8_t lastCode = ALARM_NONE;

  for (;;) {
    uint8_t code = ALARM_NONE;

    bool haveData = xQueuePeek(sensorQueue, &data, 0) == pdTRUE;
    uint32_t now = millis();

    // ---------- Sensor fault: missing, NaN, stale or implausible ----------
    bool bad = !haveData || !data.valid || (now - data.timestamp > STALE_MS);
    if (!bad && (data.temperature < 0 || data.temperature > 60 ||
                 data.humidity < 0 || data.humidity > 100)) {
      bad = true;
    }
    sensorFault = bad;

    if (bad) {
      heaterOff();
      fanOff();
      overTemp = false;
      tempBadSince = 0;
      humBadSince = 0;

      if (sensorBadSince == 0) sensorBadSince = now;
      if (now - sensorBadSince >= SENSOR_GRACE_MS) code = ALARM_SENSOR;
    }
    else {
      sensorBadSince = 0;

      // ---------- Hard over-temperature: act immediately ----------
      overTemp = data.temperature > TEMP_CUTOFF;
      if (overTemp) heaterOff();

      // ---------- Out of safe range: alarm after grace period ----------
      bool tempBad = data.temperature > TEMP_SAFE_MAX ||
                     (data.temperature < TEMP_SAFE_MIN && now > WARMUP_MS);
      bool humBad = data.humidity < HUM_SAFE_MIN || data.humidity > HUM_SAFE_MAX;

      if (tempBad) { if (tempBadSince == 0) tempBadSince = now; }
      else tempBadSince = 0;

      if (humBad) { if (humBadSince == 0) humBadSince = now; }
      else humBadSince = 0;

      if (overTemp) code = ALARM_OVERTEMP;
      else if (tempBadSince && now - tempBadSince >= ALARM_GRACE_MS) code = ALARM_TEMP;
      else if (humBadSince && now - humBadSince >= ALARM_GRACE_MS) code = ALARM_HUM;
    }

    // ---------- Local alarm (works without WiFi) ----------
    digitalWrite(BUZZER_PIN, code != ALARM_NONE ? HIGH : LOW);

    // ---------- Notify Blynk path once per new alarm ----------
    if (code != ALARM_NONE && code != lastCode) {
      Serial.printf("[WATCHDOG] ALARM: %s\n", alarmText[code]);
      xSemaphoreGive(alarmSemaphore);
    }
    if (code == ALARM_NONE && lastCode != ALARM_NONE) {
      Serial.println("[WATCHDOG] Alarm cleared.");
    }

    alarmCode = code;
    lastCode = code;

    vTaskDelay(pdMS_TO_TICKS(500));  // NFR2
  }
}

// =====================================================
// TASK 5: DISPLAY (1 s, priority 1 = LOW)
// Serial + OLED status
// =====================================================

void DisplayTask(void *pvParameters) {
  SensorData_t data;
  const char *netText[] = {"WiFi: connecting", "Blynk: connecting", "Blynk: online"};

  for (;;) {
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      bool ok = xQueuePeek(sensorQueue, &data, 0) == pdTRUE && data.valid;

      // ---------- Serial ----------
      if (ok) {
        Serial.printf("[Display] T=%.1fC H=%.1f%% Day %d/%d (%d left)%s | %s\n",
                      data.temperature, data.humidity,
                      incubationDay, TOTAL_DAYS, TOTAL_DAYS - incubationDay,
                      isLockdown() ? " LOCKDOWN" : "",
                      alarmText[alarmCode]);
      } else {
        Serial.println("[Display] Sensor Error");
      }

      // ---------- OLED ----------
      if (oledOk) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);

        display.setTextSize(2);
        display.setCursor(0, 0);
        if (ok) display.printf("%.1fC", data.temperature);
        else    display.print("SENSOR!");

        display.setTextSize(1);
        display.setCursor(0, 18);
        if (ok) display.printf("Humidity: %.0f%%", data.humidity);

        display.setCursor(0, 28);
        display.printf("Day %d/%d%s", incubationDay, TOTAL_DAYS, isLockdown() ? " LOCKDOWN" : "");

        display.setCursor(0, 38);
        display.print(alarmText[alarmCode]);

        display.setCursor(0, 54);
        display.print(netText[netState]);

        display.display();
      }

      xSemaphoreGive(displayMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// =====================================================
// TASK 6: BLYNK SYNC (priority 2, core 0)
// Only this task talks to Blynk. Control never waits on it.
// =====================================================

void BlynkSyncTask(void *pvParameters) {
  SensorData_t data;
  uint32_t lastPublish = 0;

  // Non-blocking connect (FR8 / NFR6)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Blynk.config(BLYNK_AUTH_TOKEN);
  uint32_t wifiRetry = millis();

  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      Blynk.run();
      wifiRetry = millis();
    }
    // Not connected for 15 s: restart the WiFi attempt
    else if (millis() - wifiRetry > 15000) {
      Serial.println("[WiFi] Retrying...");
      WiFi.disconnect();
      WiFi.begin(ssid, pass);
      wifiRetry = millis();
    }

    netState = (WiFi.status() != WL_CONNECTED) ? 0 : (Blynk.connected() ? 2 : 1);

    if (Blynk.connected()) {

      // ---------- FR7: push notification ----------
      if (xSemaphoreTake(alarmSemaphore, 0) == pdTRUE && alarmCode != ALARM_NONE) {
        Blynk.logEvent("incubator_alarm", alarmText[alarmCode]);
      }

      // ---------- FR6: publish every 2 s ----------
      if (millis() - lastPublish >= 2000) {

        if (xQueuePeek(sensorQueue, &data, 0) == pdTRUE && data.valid) {
          Blynk.virtualWrite(V0, data.temperature);
          Blynk.virtualWrite(V1, data.humidity);
        }

        bool currentHeater = false;
        bool currentFan = false;
        if (xSemaphoreTake(heaterMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          currentHeater = heaterState;
          currentFan = fanState;
          xSemaphoreGive(heaterMutex);
        }

        Blynk.virtualWrite(V2, currentHeater ? 1 : 0);
        Blynk.virtualWrite(V3, currentFan ? 1 : 0);
        Blynk.virtualWrite(V5, incubationDay);
        Blynk.virtualWrite(V4, alarmText[alarmCode]);
        Blynk.virtualWrite(V8, turning ? 1 : 0);

        lastPublish = millis();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Manual motor control (used by V8 / V6), protected by motorMutex
bool motorSet(bool on) {
  if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    eggServo.writeMicroseconds(on ? SERVO_SPIN_US : SERVO_STOP_US);
    turning = on;
    xSemaphoreGive(motorMutex);
    return true;
  }
  return false;   // busy (automatic turn in progress)
}

// =====================================================
// BLYNK HANDLERS (run inside BlynkSyncTask via Blynk.run)
// =====================================================

BLYNK_CONNECTED() {
  static bool firstConnect = true;

  // Always start in AUTO mode (fail-safe)
  Blynk.virtualWrite(V6, 0);

  // Restore day + setpoint from server only after a reboot
  if (firstConnect) {
    Blynk.syncVirtual(V5, V7);
    firstConnect = false;
  }
}

// V6: manual / auto
BLYNK_WRITE(V6) {
  int value = param.asInt();

  if (xSemaphoreTake(heaterMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    manualMode = (value == 1);
    xSemaphoreGive(heaterMutex);
  }

  // Leaving manual mode: stop the motor if it was left running
  if (value == 0 && turning) {
    motorSet(false);
  }

  Serial.println(value == 1 ? "[Blynk] MANUAL MODE ON" : "[Blynk] AUTOMATIC MODE ON");
}

// V2: heater switch (manual mode only)
BLYNK_WRITE(V2) {
  int value = param.asInt();
  bool manual;
  float setPoint, humidityLimit;
  getControlValues(manual, setPoint, humidityLimit);

  if (manual) {
    if (value == 1) heaterOn();
    else heaterOff();
  }
  else {
    // Auto mode: undo the switch press, show real state
    bool currentState = false;
    if (xSemaphoreTake(heaterMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      currentState = heaterState;
      xSemaphoreGive(heaterMutex);
    }
    Blynk.virtualWrite(V2, currentState ? 1 : 0);
  }
}

// V3: fan switch (manual mode only)
BLYNK_WRITE(V3) {
  int value = param.asInt();
  bool manual;
  float setPoint, humidityLimit;
  getControlValues(manual, setPoint, humidityLimit);

  if (manual) {
    if (value == 1) fanOn();
    else fanOff();
  }
  else {
    bool currentState = false;
    if (xSemaphoreTake(heaterMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      currentState = fanState;
      xSemaphoreGive(heaterMutex);
    }
    Blynk.virtualWrite(V3, currentState ? 1 : 0);
  }
}

// V5: set incubation day
BLYNK_WRITE(V5) {
  incubationDay = constrain(param.asInt(), 1, TOTAL_DAYS);
  Serial.printf("[Blynk] Incubation Day: %d\n", incubationDay);
}

// V7: temperature setpoint (clamped for safety)
BLYNK_WRITE(V7) {
  float newSetPoint = constrain(param.asFloat(), 35.0, 39.0);

  if (xSemaphoreTake(heaterMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    temperatureSetPoint = newSetPoint;
    xSemaphoreGive(heaterMutex);
  }

  Serial.printf("[Blynk] New Temperature Set Point: %.1f C\n", newSetPoint);
}

// V8: motor switch - shows state, controls motor in manual mode
BLYNK_WRITE(V8) {
  int value = param.asInt();

  if (manualMode && motorSet(value == 1)) {
    Serial.println(value == 1 ? "[Blynk] Motor ON (manual)" : "[Blynk] Motor OFF (manual)");
  }
  else {
    // Auto mode or motor busy: undo the switch press, show real state
    Blynk.virtualWrite(V8, turning ? 1 : 0);
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n================================");
  Serial.println(" SMART INCUBATOR - FREERTOS");
  Serial.println("================================");

  // ---------- GPIO: OFF level written BEFORE output enable ----------
  // Heater lamp OFF at boot
  if (HEATER_DIRECT) {
    digitalWrite(HEATER_PIN, LOW);
    pinMode(HEATER_PIN, OUTPUT);
  } else {
    pinMode(HEATER_PIN, INPUT);
  }

  pinMode(FAN_RELAY, INPUT);  // fan OFF (your workaround)

  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);

  dht.begin();

  // ---------- Servo ----------
  eggServo.setPeriodHertz(50);
  eggServo.attach(SERVO_PIN, 500, 2400);
  eggServo.writeMicroseconds(SERVO_STOP_US);   // start stopped

  // ---------- OLED ----------
  Wire.begin(SDA_PIN, SCL_PIN);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Smart Incubator");
    display.println("Starting...");
    display.display();
  } else {
    Serial.println("[OLED] Not found - check wiring / address 0x3C");
  }

  // ---------- RTOS objects (before any task or Blynk handler uses them) ----------
  sensorQueue    = xQueueCreate(1, sizeof(SensorData_t));
  heaterMutex    = xSemaphoreCreateMutex();
  displayMutex   = xSemaphoreCreateMutex();
  alarmSemaphore = xSemaphoreCreateBinary();
  motorMutex     = xSemaphoreCreateMutex();

  // ---------- Tasks ----------
  xTaskCreatePinnedToCore(SensorReadTask,     "SensorRead",     4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(HeaterCtrlTask,     "HeaterCtrl",     4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(HumidityCtrlTask,   "HumidityCtrl",   4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(EggTurnTask,        "EggTurn",        4096, NULL, 2, &eggTurnTaskHandle, 1);
  xTaskCreatePinnedToCore(SafetyWatchdogTask, "SafetyWatchdog", 4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(DisplayTask,        "Display",        4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(BlynkSyncTask,      "BlynkSync",      8192, NULL, 2, NULL, 0);

  // ---------- Software timers ----------
  turnTimer = xTimerCreate("TurnTimer", LONG_MS_TO_TICKS(TURN_INTERVAL_MS),
                           pdTRUE, NULL, turnTimerCallback);
  if (turnTimer != NULL) xTimerStart(turnTimer, 0);

  dayTimer = xTimerCreate("DayTimer", LONG_MS_TO_TICKS(DAY_LENGTH_MS),
                           pdTRUE, NULL, dayTimerCallback);

  if (dayTimer != NULL)  xTimerStart(dayTimer, 0);

  Serial.println("All FreeRTOS tasks started.");
}

// =====================================================
// LOOP: intentionally empty, all work is in tasks
// =====================================================

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

#include <Arduino.h>
// Webserver/JSON benoetigen mehr Stack als der Arduino-Standard (8 KiB).
// Arduino 3.3.x: ueberschreibt die Loop-Task-Stackgroesse auf 24 KiB.
SET_LOOP_TASK_STACK_SIZE(24 * 1024);

// Zeitkiste 1.2.12 / ESP32-S3 Zero / Arduino-ESP32 3.x
// Datenmessgeraet, Zeitserver und GNSS-Disziplinierung ohne EFC-Regelung.
// GPS-TX -> GPIO1, GPS-RX <- GPIO2, GPS-PPS -> GPIO4
// LCD 16x4 + DS3231 SDA5 SCL6; SQW7; RTC 32k GPIO8
// OCXO 10 MHz -> Spannungsteiler -> GPIO9, gemeinsame Masse!
// RTC-VCC an feste Versorgung, NICHT an ESP GPIO.
#define ZEITKISTE_WLAN_AN 1
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <HardwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <time.h>
#include <LittleFS.h>
#include <math.h>
#include "driver/pulse_cnt.h"
#include "driver/temperature_sensor.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

// AsyncTCP muss auf Core0 bleiben, damit der Webserver den Messkern auf Core1
// nicht belastet. Dieses Makro MUSS als PlatformIO build_flag gesetzt werden,
// damit es auch beim Kompilieren der AsyncTCP-Bibliothek wirksam ist.
#if CONFIG_ASYNC_TCP_RUNNING_CORE != 0
#error "Zeitkiste 1.2.12: CONFIG_ASYNC_TCP_RUNNING_CORE muss per build_flags auf 0 gesetzt sein."
#endif
#ifndef CONFIG_PCNT_ISR_IRAM_SAFE
#define CONFIG_PCNT_ISR_IRAM_SAFE 0
#endif

// PIO/ESP-IDF-Build muss IRAM-sicheren PCNT wirklich enthalten.
// Ein blosses -D-Flag bei vorkompiliertem ESP-IDF reicht NICHT.
#if !defined(CONFIG_ARDUINO_RUN_CORE0) || !CONFIG_ARDUINO_RUN_CORE0
#error "Zeitkiste: Arduino setup()/loop() muss auf Core0 laufen (sdkconfig.defaults)."
#endif
// Kconfig und wirksame Arduino-Task-Affinity getrennt pruefen.
// Verwende den durch ESP-IDF-Kconfig konfigurierten Arduino-Core (ohne eigenes -D-Flag).
#if ARDUINO_RUNNING_CORE != 0
#error "Zeitkiste: ARDUINO_RUNNING_CORE muss 0 sein; sdkconfig und Arduino-Core-Auswahl pruefen."
#endif
#if ARDUINO_EVENT_RUNNING_CORE != 0
#error "Zeitkiste: Arduino-Eventtask muss auf Core0 laufen."
#endif
#if CONFIG_ARDUINO_UDP_RUNNING_CORE != 0
#error "Zeitkiste: Arduino-UDP-Task muss auf Core0 laufen."
#endif
#if !CONFIG_PCNT_ISR_IRAM_SAFE
#error "Zeitkiste PIO: CONFIG_PCNT_ISR_IRAM_SAFE fehlt. sdkconfig kontrollieren; bei Bedarf neu konfigurieren."
#endif
#if !defined(CONFIG_ARDUINO_ISR_IRAM) || !CONFIG_ARDUINO_ISR_IRAM
#error "Zeitkiste PIO: CONFIG_ARDUINO_ISR_IRAM muss aktiv sein (PPS/SQW waehrend Flash)."
#endif

// Explizite Forward-Deklarationen (Arduino-IDE-Prototyp-Generator).
uint64_t clockNowUs();
uint64_t clockAtMonoUs(int64_t mono);
const char *clockState();
float estimatedUncertaintyMs();
bool gpsLocked();
bool ntpGpsReady(); // Core-uebergreifend publizierter NTP/GPS-Lock
bool clearAllMeasurements();
void processPps();
void rtcLiveRateReset();
void rtcLiveRatePush(int64_t monoUs, double phaseMs);
void ntpServerTick();
String jnum(double v, int d);
uint64_t bestTimestampEpoch();
void markFlashActivity();
void rotateEventsIfNeeded();
extern float chipTempC;
void enqueueRtcEvent(uint8_t kind, uint64_t epoch, int16_t aging, double value);
extern volatile uint32_t rtcEventsDropped;

constexpr char FW_VERSION[] = "1.2.12";
constexpr char HOSTNAME[] = "zeitkiste";
constexpr char TZ_INFO[] = "CET-1CEST,M3.5.0,M10.5.0/3";

constexpr char AP_SSID[] = "Zeitkiste-Setup";
constexpr char AP_PASS[] = "zeitkiste";

constexpr int PIN_SDA = 5;
constexpr int PIN_SCL = 6;
// RTC wird extern versorgt, kein GPIO-VCC!
constexpr int PIN_RTC_SQW = 7;
constexpr int PIN_RTC_32K = 8;
constexpr int PIN_GPS_RX = 1;
constexpr int PIN_GPS_TX = 2;
constexpr int PIN_GPS_PPS = 4;
constexpr int PIN_OCXO = 9;

constexpr uint8_t DS3231_ADDR = 0x68;

constexpr uint32_t LOCAL_NTP_INTERVAL = 10000;
constexpr uint32_t POOL_NTP_INTERVAL = 60000;
constexpr uint32_t NTP_TIMEOUT = 900;
// Bei zu grossen RTTs lieber KEINE Zeitabweichung anzeigen als eine
// unkontrollierbare halbe WLAN/Server-Wartezeit als Zeitfehler ausgeben.
constexpr uint32_t LOCAL_NTP_MAX_RTT_US = 150000;
constexpr uint32_t POOL_NTP_MAX_RTT_US = 250000;
constexpr uint32_t NTP_MAX_SAMPLE_AGE_MS = 240000;
constexpr uint32_t NTP_BEST_WINDOW_MS = 180000;
// ATGM336H: RMC-Sekunde wird dem vorhergehenden PPS zugeordnet.
// Vor Stratum-1-Betrieb gegen eine unabhaengige Zeitquelle verifizieren!
constexpr int8_t RMC_PPS_ADJUST_SECONDS = 0;
constexpr uint8_t GPS_NTP_STRATUM = 2; // nach externer Sekundenzuordnungspruefung ggf. 1

Preferences prefs;
AsyncWebServer web(80);
DNSServer dns;
WiFiUDP udp;
HardwareSerial GPS(1);

LiquidCrystal_I2C *lcd = nullptr;
uint8_t lcdAddr = 0;

String wifiSsid;
String wifiPass;
String localNtpHost = "192.168.178.62";
String poolNtpHost = "pool.ntp.org";
SemaphoreHandle_t ntpHostMutex = nullptr;

// WLAN-/Core0-Diagnose 1.2.8. Nur Zaehler/Timestamps im Event-Callback;
// keine Strings/Flashzugriffe aus dem Netzwerk-Eventtask.
volatile uint32_t wifiDisconnectEvents = 0;
volatile uint32_t wifiGotIpEvents = 0;
volatile uint32_t wifiLostIpEvents = 0;
volatile uint32_t wifiLastDisconnectReason = 0;
volatile uint32_t wifiLastEventMs = 0;
uint32_t loopMaxGapUs = 0;
uint32_t loopMaxWorkUs = 0;
uint32_t webHandleMaxUs = 0;
uint32_t lcdMaxUs = 0;
uint32_t loggerMaxUs = 0;
uint32_t loopGapOver50ms = 0;
uint32_t loopGapOver200ms = 0;
uint32_t webOver50ms = 0;

// ESPAsyncWebServer fuehrt Handler im AsyncTCP-Task aus. Kurze GET-Antworten
// duerfen dort direkt erzeugt werden; Flash-/Format-/Reset-Aktionen werden
// dagegen in den normalen Core0-loop verschoben.
enum WebMaintenanceAction : uint8_t {
  WEB_MAINT_NONE = 0,
  WEB_MAINT_CLEAR_ALL,
  WEB_MAINT_INIT_FS,
  WEB_MAINT_SAVE_RESTART
};
SemaphoreHandle_t webMaintMutex = nullptr;
WebMaintenanceAction webMaintPending = WEB_MAINT_NONE;
bool webMaintBusy = false;
uint32_t webMaintNotBeforeMs = 0;
String webMaintSsid, webMaintPass, webMaintLocalNtp, webMaintPoolNtp;
bool webMaintUpdatePass = false;

String asyncPostArg(AsyncWebServerRequest *request, const char *name) {
  if (!request || !request->hasParam(name, true)) return String();
  const AsyncWebParameter *p = request->getParam(name, true);
  return p ? p->value() : String();
}

void noteWebCallback(int64_t startUs) {
  const int64_t nowUs=esp_timer_get_time();
  const uint32_t dt=(uint32_t)(nowUs>startUs ? nowUs-startUs : 0);
  if (dt>webHandleMaxUs) webHandleMaxUs=dt;
  if (dt>50000U) webOver50ms++;
}

bool queueWebMaintenance(WebMaintenanceAction action, uint32_t delayMs=250) {
  if (!webMaintMutex || xSemaphoreTake(webMaintMutex,0)!=pdTRUE) return false;
  const bool free=(webMaintPending==WEB_MAINT_NONE && !webMaintBusy);
  if (free) {
    webMaintPending=action;
    webMaintNotBeforeMs=millis()+delayMs;
  }
  xSemaphoreGive(webMaintMutex);
  return free;
}

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  wifiLastEventMs = millis();
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    wifiDisconnectEvents++;
    wifiLastDisconnectReason = info.wifi_sta_disconnected.reason;
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    wifiGotIpEvents++;
  } else if (event == ARDUINO_EVENT_WIFI_STA_LOST_IP) {
    wifiLostIpEvents++;
  }
}

bool apMode = false;
bool rtcPresent = false;
bool rtcOsf = true;
uint64_t rtcLastSyncEpoch = 0;
double rtcOffsetBeforeSyncMs = NAN;
uint32_t rtcSyncCount = 0;
uint32_t rtcLastFailedWriteMs = 0;
bool fsReady = false;

// ------------------------------------------------------------
// Hilfsfunktionen Zeit
// ------------------------------------------------------------

int64_t daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  int era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097LL + (int64_t)doe - 719468LL;
}

uint64_t civilToEpoch(int y, int mo, int d, int h, int mi, int s) {
  int64_t days = daysFromCivil(y, mo, d);
  if (days < 0) return 0;
  return (uint64_t)days * 86400ULL +
         (uint64_t)h * 3600ULL +
         (uint64_t)mi * 60ULL +
         (uint64_t)s;
}

String fmtTime(uint64_t epochUs, bool local = true, bool ms = true) {
  if (!epochUs) return ms ? "--:--:--.---" : "--:--:--";
  time_t tt = epochUs / 1000000ULL;
  struct tm t {};
  if (local) localtime_r(&tt, &t); else gmtime_r(&tt, &t);

  char b[24];
  if (ms) {
    snprintf(b, sizeof(b), "%02d:%02d:%02d.%03u",
             t.tm_hour, t.tm_min, t.tm_sec,
             (unsigned)((epochUs / 1000ULL) % 1000ULL));
  } else {
    snprintf(b, sizeof(b), "%02d:%02d:%02d",
             t.tm_hour, t.tm_min, t.tm_sec);
  }
  return String(b);
}

String fmtDate(uint64_t epochUs, bool local = false) {
  if (!epochUs) return "--.--.----";
  time_t tt = epochUs / 1000000ULL;
  struct tm t {};
  if (local) localtime_r(&tt, &t); else gmtime_r(&tt, &t);
  char buf[16];
  snprintf(buf,sizeof(buf),"%02d.%02d.%04d",t.tm_mday,t.tm_mon+1,t.tm_year+1900);
  return String(buf);
}

String fit16(String s) {
  if (s.length() > 16) s.remove(16);
  while (s.length() < 16) s += ' ';
  return s;
}

String fmtDateShort(uint64_t epochUs, bool local = true) {
  if (!epochUs) return "--.--";
  time_t tt = epochUs / 1000000ULL;
  struct tm t {};
  if (local) localtime_r(&tt, &t); else gmtime_r(&tt, &t);
  char b[8];
  snprintf(b,sizeof(b),"%02d.%02d",t.tm_mday,t.tm_mon+1);
  return String(b);
}

// LCD (Core0) und RTC (Core1) teilen einen Bus. Schuetzt ganze
// Mehrtransaktions-Zugriffe, nicht nur Wire.endTransmission().
SemaphoreHandle_t i2cBusMutex = nullptr;
class I2cBusGuard {
 public:
  I2cBusGuard() { if (i2cBusMutex) xSemaphoreTakeRecursive(i2cBusMutex,portMAX_DELAY); }
  ~I2cBusGuard() { if (i2cBusMutex) xSemaphoreGiveRecursive(i2cBusMutex); }
};
void lcdLine(int row, const String &s) {
  if (!lcd) return;
  I2cBusGuard guard;
  lcd->setCursor(0, row);
  lcd->print(fit16(s));
}

// Laufende LCD-Ausgabe ab 1.2.12: Anzeige erst im RAM zusammensetzen und
// danach nur geaenderte Zeichen in kleinen Portionen uebertragen.
// Das LCD ist unkritisch: ist der gemeinsame RTC/LCD-I2C-Bus belegt,
// wartet Core0 NICHT auf den Mutex, sondern versucht es im naechsten Loop.
constexpr uint8_t LCD_ROWS = 4;
constexpr uint8_t LCD_COLS = 16;
constexpr uint8_t LCD_WRITE_CHUNK = 4;
char lcdDesired[LCD_ROWS][LCD_COLS + 1] = {};
char lcdShadow[LCD_ROWS][LCD_COLS + 1] = {};
bool lcdBufferReady = false;
uint8_t lcdScanRow = 0;
uint8_t lcdScanCol = 0;
uint32_t lcdWriteChunks = 0;
uint32_t lcdWriteChars = 0;
uint32_t lcdBusBusySkips = 0;
uint32_t lcdChunkMaxUs = 0;

void lcdBufferInit() {
  for (uint8_t r=0; r<LCD_ROWS; ++r) {
    for (uint8_t c=0; c<LCD_COLS; ++c) {
      lcdDesired[r][c] = ' ';
      lcdShadow[r][c] = '\0'; // ungueltig -> erste Laufzeitanzeige wird geschrieben
    }
    lcdDesired[r][LCD_COLS] = '\0';
    lcdShadow[r][LCD_COLS] = '\0';
  }
  lcdScanRow = lcdScanCol = 0;
  lcdBufferReady = true;
}

void lcdDesiredLine(uint8_t row, const String &s) {
  if (row >= LCD_ROWS) return;
  if (!lcdBufferReady) lcdBufferInit();
  const String fixed = fit16(s);
  for (uint8_t c=0; c<LCD_COLS; ++c)
    lcdDesired[row][c] = c < fixed.length() ? fixed[c] : ' ';
  lcdDesired[row][LCD_COLS] = '\0';
}

void lcdSetScan(uint8_t row, uint8_t col) {
  if (col >= LCD_COLS) {
    row = (uint8_t)((row + col / LCD_COLS) % LCD_ROWS);
    col = (uint8_t)(col % LCD_COLS);
  }
  lcdScanRow = row;
  lcdScanCol = col;
}

void lcdAdvanceScan() {
  lcdScanCol++;
  if (lcdScanCol >= LCD_COLS) {
    lcdScanCol = 0;
    lcdScanRow = (uint8_t)((lcdScanRow + 1) % LCD_ROWS);
  }
}

void lcdService() {
  if (!lcd) return;
  if (!lcdBufferReady) lcdBufferInit();

  // Hoechstens einen kleinen zusammenhaengenden Schreibblock pro Loop.
  // Dadurch kann kein kompletter 4x16-Refresh Core0 ~170 ms am Stueck belegen.
  for (uint8_t scanned=0; scanned<LCD_ROWS*LCD_COLS; ++scanned) {
    const uint8_t row=lcdScanRow, col=lcdScanCol;
    lcdAdvanceScan();
    if (lcdShadow[row][col] == lcdDesired[row][col]) continue;

    uint8_t len=1;
    while (len<LCD_WRITE_CHUNK && col+len<LCD_COLS &&
           lcdShadow[row][col+len] != lcdDesired[row][col+len]) len++;

    if (i2cBusMutex &&
        xSemaphoreTakeRecursive(i2cBusMutex,0) != pdTRUE) {
      lcdBusBusySkips++;
      lcdSetScan(row,col); // denselben Dirty-Block spaeter erneut versuchen
      return;
    }

    const int64_t t0=esp_timer_get_time();
    lcd->setCursor(col,row);
    for (uint8_t i=0; i<len; ++i) {
      const uint8_t ch=(uint8_t)lcdDesired[row][col+i];
      lcd->write(ch);
      lcdShadow[row][col+i]=lcdDesired[row][col+i];
    }
    const int64_t t1=esp_timer_get_time();
    if (i2cBusMutex) xSemaphoreGiveRecursive(i2cBusMutex);

    const uint32_t dt=(uint32_t)(t1>t0 ? t1-t0 : 0);
    if (dt>lcdChunkMaxUs) lcdChunkMaxUs=dt;
    lcdWriteChunks++;
    lcdWriteChars+=len;
    lcdSetScan(row,(uint8_t)(col+len));
    return;
  }
}

String signedVal(double v, int dec = 3) {
  if (isnan(v)) return "---";
  String s = v >= 0 ? "+" : "";
  s += String(v, dec);
  return s;
}

// ------------------------------------------------------------
// NTP
// ------------------------------------------------------------

constexpr uint8_t NTP_LOCAL = 0;
constexpr uint8_t NTP_POOL  = 1;

bool ntpValid[2] = { false, false };
uint64_t ntpEpochUs[2] = { 0, 0 };
int64_t ntpMonoUs[2] = { 0, 0 };
uint32_t ntpRttUs[2] = { 0, 0 };       // Netz-RTT ohne Server-Verarbeitungszeit
uint32_t ntpRawRttUs[2] = { 0, 0 };    // komplette Request->Reply Zeit
uint32_t ntpServerProcUs[2] = { 0, 0 }; // Server Receive->Transmit
uint32_t ntpLastOkMs[2] = { 0, 0 };
uint32_t ntpWindowStartMs[2] = { 0, 0 };
uint32_t ntpAccepted[2] = { 0, 0 };
uint32_t ntpRejectedDelay[2] = { 0, 0 };
uint32_t ntpRejectedProtocol[2] = { 0, 0 };
uint32_t ntpLatestRawRttUs[2] = { 0, 0 };
uint32_t ntpLatestProcUs[2] = { 0, 0 };
uint32_t ntpLatestNetRttUs[2] = { 0, 0 };
uint32_t ntpLastAttemptMs[2] = { 0, 0 };
volatile uint32_t ntpWorkerLocalQueries = 0, ntpWorkerPoolQueries = 0;
volatile uint32_t ntpWorkerFailedQueries[2] = {0,0};
volatile uint32_t ntpWorkerSkippedNoGps[2] = {0,0};
// Feingranulare NTP-Diagnose: DNS, UDP-Senden, Timeout und Gesamtlaufzeit.
volatile uint32_t ntpDnsFailures[2] = {0,0};
volatile uint32_t ntpSendFailures[2] = {0,0};
volatile uint32_t ntpTimeouts[2] = {0,0};
volatile uint32_t ntpClockFailures[2] = {0,0};
volatile uint32_t ntpDnsLastUs[2] = {0,0};
volatile uint32_t ntpDnsMaxUs[2] = {0,0};
volatile uint32_t ntpQueryMaxUs[2] = {0,0};
int ntpWorkerAssignedCore = -1;
portMUX_TYPE ntpDataMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t ntpResetGeneration = 0;

bool ntpFresh(uint8_t idx) {
  if (idx >= 2) return false;
  bool valid;
  uint32_t last;
  portENTER_CRITICAL(&ntpDataMux);
  valid = ntpValid[idx]; last = ntpLastOkMs[idx];
  portEXIT_CRITICAL(&ntpDataMux);
  return valid && (uint32_t)(millis() - last) < NTP_MAX_SAMPLE_AGE_MS;
}

uint64_t ntpAt(uint8_t idx, int64_t mono) {
  if (idx >= 2) return 0;
  bool valid;
  uint32_t last;
  uint64_t epoch;
  int64_t anchor;
  portENTER_CRITICAL(&ntpDataMux);
  valid=ntpValid[idx]; last=ntpLastOkMs[idx];
  epoch=ntpEpochUs[idx]; anchor=ntpMonoUs[idx];
  portEXIT_CRITICAL(&ntpDataMux);
  if (!valid || (uint32_t)(millis()-last) >= NTP_MAX_SAMPLE_AGE_MS) return 0;
  int64_t d=mono-anchor;
  if (d>=0) return epoch+(uint64_t)d;
  uint64_t back=(uint64_t)(-d);
  return back>epoch ? 0 : epoch-back;
}

uint64_t ntpNow(uint8_t idx) {
  return ntpAt(idx, esp_timer_get_time());
}

// ------------------------------------------------------------
// RTC
// ------------------------------------------------------------

uint8_t bcd2dec(uint8_t v) { return (v >> 4) * 10 + (v & 15); }
uint8_t dec2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

bool i2cPresent(uint8_t a) {
  Wire.beginTransmission(a);
  return Wire.endTransmission() == 0;
}

bool rtcReadReg(uint8_t reg, uint8_t &v) {
  I2cBusGuard busGuard;
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false)) return false;
  if (Wire.requestFrom((int)DS3231_ADDR, 1) != 1) return false;
  v = Wire.read();
  return true;
}

bool rtcWriteReg(uint8_t reg, uint8_t v) {
  I2cBusGuard busGuard;
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(reg);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

void rtcConfigure() {
  if (!rtcPresent) return;
  uint8_t status = 0x80;
  if (rtcReadReg(0x0F, status)) rtcOsf = (status & 0x80) != 0;

  uint8_t c;
  if (rtcReadReg(0x0E, c)) {
    c &= ~(0x1C); // SQW = 1 Hz
    rtcWriteReg(0x0E, c);
  }

  uint8_t s;
  if (rtcReadReg(0x0F, s)) {
    s |= 0x08; // 32 kHz on
    rtcWriteReg(0x0F, s);
  }
}

bool rtcReadEpoch(uint64_t &epoch) {
  I2cBusGuard busGuard;
  if (!rtcPresent) return false;

  Wire.beginTransmission(DS3231_ADDR);
  Wire.write((uint8_t)0);
  if (Wire.endTransmission(false)) return false;
  if (Wire.requestFrom((int)DS3231_ADDR, 7) != 7) return false;

  int sec = bcd2dec(Wire.read() & 0x7F);
  int min = bcd2dec(Wire.read() & 0x7F);
  uint8_t hrRaw = Wire.read();
  Wire.read();
  int day = bcd2dec(Wire.read() & 0x3F);
  uint8_t monRaw = Wire.read();
  int year = 2000 + bcd2dec(Wire.read());

  int hour;
  if (hrRaw & 0x40) {
    hour = bcd2dec(hrRaw & 0x1F);
    if (hour == 12) hour = 0;
    if (hrRaw & 0x20) hour += 12;
  } else {
    hour = bcd2dec(hrRaw & 0x3F);
  }

  int mon = bcd2dec(monRaw & 0x1F);
  if (monRaw & 0x80) year += 100;

  if (year < 2020 || year > 2099 || mon < 1 || mon > 12 ||
      day < 1 || day > 31 || hour > 23 || min > 59 || sec > 59) return false;
  epoch = civilToEpoch(year, mon, day, hour, min, sec);
  return epoch != 0;
}

bool rtcWriteEpoch(uint64_t epoch) {
  I2cBusGuard busGuard;
  if (!rtcPresent) return false;

  time_t tt = epoch;
  struct tm t {};
  gmtime_r(&tt, &t);

  Wire.beginTransmission(DS3231_ADDR);
  Wire.write((uint8_t)0);
  Wire.write(dec2bcd(t.tm_sec));
  Wire.write(dec2bcd(t.tm_min));
  Wire.write(dec2bcd(t.tm_hour));
  Wire.write(dec2bcd(t.tm_wday ? t.tm_wday : 7));
  Wire.write(dec2bcd(t.tm_mday));
  Wire.write(dec2bcd(t.tm_mon + 1));
  Wire.write(dec2bcd((t.tm_year + 1900) % 100));
  if (Wire.endTransmission() != 0) return false;
  uint8_t status;
  if (rtcReadReg(0x0F, status)) rtcWriteReg(0x0F, status & ~0x80);
  rtcOsf = false;
  return true;
}

// ------------------------------------------------------------
// GPS
// ------------------------------------------------------------

String nmea;
bool gpsFix = false;
uint8_t gpsSats = 0;
uint8_t gpsVisible = 0;
uint8_t gpsBestCno = 0;   // C/N0 ueber alle frischen GNSS-GSV-Gruppen
float gpsHdop = NAN;
uint8_t gpsFixType = 1;  // GSA 1=kein Fix, 2=2D, 3=3D

// GSV verschiedener Konstellationen separat auswerten. GN wird nur als
// Ersatz fuer Einzeltalker genutzt, sonst waeren die Sats doppelt gezaehlt.
// Keine Positionsdaten werden gespeichert oder angezeigt.
constexpr uint8_t GNSS_GROUP_COUNT = 6;
constexpr uint8_t GNSS_GP = 0;
constexpr uint8_t GNSS_BD = 1;
constexpr uint8_t GNSS_GB = 2;
constexpr uint8_t GNSS_GL = 3;
constexpr uint8_t GNSS_GA = 4;
constexpr uint8_t GNSS_GN = 5;
const char * const gnssTalkers[GNSS_GROUP_COUNT] = {"GP", "BD", "GB", "GL", "GA", "GN"};
uint16_t gnssVisible[GNSS_GROUP_COUNT] = {};
uint16_t gnssSignal[GNSS_GROUP_COUNT] = {};
uint8_t gnssBestCno[GNSS_GROUP_COUNT] = {};
uint32_t gnssUpdateMs[GNSS_GROUP_COUNT] = {};
uint16_t gnssWorkSignal[GNSS_GROUP_COUNT] = {};
uint8_t gnssWorkBest[GNSS_GROUP_COUNT] = {};
uint8_t gnssNextMsg[GNSS_GROUP_COUNT] = {};
constexpr uint32_t GNSS_GSV_VALID_MS = 7000;

// $GPTXT: ANTENNA OK / OPEN / SHORT, sofern vom Breakout unterstuetzt.
// Elektrische Antennenerkennung ist KEIN Test der HF-Anpassung.
enum { ANT_UNKNOWN = 0, ANT_OK = 1, ANT_OPEN = 2, ANT_SHORT = 3 };
uint8_t antennaStatus = ANT_UNKNOWN;
uint32_t antennaLastMs = 0;

uint16_t gnssVisibleNow(uint8_t index) {
  if (index >= GNSS_GROUP_COUNT || !gnssUpdateMs[index] ||
      millis() - gnssUpdateMs[index] > GNSS_GSV_VALID_MS) return 0;
  return gnssVisible[index];
}

uint16_t gnssSignalNow(uint8_t index) {
  if (index >= GNSS_GROUP_COUNT || !gnssUpdateMs[index] ||
      millis() - gnssUpdateMs[index] > GNSS_GSV_VALID_MS) return 0;
  return gnssSignal[index];
}

uint8_t gnssCnoNow(uint8_t index) {
  if (index >= GNSS_GROUP_COUNT || !gnssUpdateMs[index] ||
      millis() - gnssUpdateMs[index] > GNSS_GSV_VALID_MS) return 0;
  return gnssBestCno[index];
}

const char* antennaTextFor(uint8_t status) {
  switch(status) {
    case ANT_OK: return "OK";
    case ANT_OPEN: return "OPEN";
    case ANT_SHORT: return "SHORT";
    default: return "UNBEKANNT";
  }
}
const char* antennaText() {
  switch (antennaStatus) {
    case ANT_OK: return "OK";
    case ANT_OPEN: return "OPEN";
    case ANT_SHORT: return "SHORT";
    default: return "UNBEKANNT";
  }
}

// Summen frisch halten. Bei BD/GB handelt es sich um alternative Talker
// fuer BeiDou, deshalb diese nicht ungeprueft addieren.
void updateGnssSummary() {
  bool distinct = false;
  uint16_t total = 0;
  uint8_t best = 0;
  for (uint8_t i = 0; i < GNSS_GN; i++) {
    if (i == GNSS_GB && gnssVisibleNow(GNSS_BD)) continue;
    if (gnssVisibleNow(i)) distinct = true;
    total += gnssVisibleNow(i);
    if (gnssCnoNow(i) > best) best = gnssCnoNow(i);
  }
  if (!distinct) {
    total = gnssVisibleNow(GNSS_GN);
    best = gnssCnoNow(GNSS_GN);
  }
  gpsVisible = (uint8_t)min((uint16_t)255, total);
  gpsBestCno = best;
}

uint32_t nmeaOk = 0, nmeaBad = 0;
uint32_t lastGpsByteMs = 0;
uint32_t lastRmcMs = 0;
uint64_t previousRmcEpoch = 0;
uint16_t stableUtcSeconds = 0;
uint32_t gnssTimeAnomalies = 0;
uint32_t stableLastMappedSeq = 0;
uint32_t ppsBadPeriods = 0;
uint32_t counterReadFailures = 0;
uint32_t lastFixMs = 0;

uint64_t lastRmcEpoch = 0;

// Zuordnung: RMC-Zeit gehört zur zuletzt vorausgegangenen PPS-Flanke.
bool gpsEpochMapped = false;
uint32_t mappedPpsSeq = 0;
int64_t mappedPpsMono = 0;
uint64_t mappedPpsEpoch = 0;

String csv(const String &s, int field) {
  int n = 0, start = 0;
  for (int i = 0; i <= (int)s.length(); ++i) {
    if (i == (int)s.length() || s[i] == ',') {
      if (n == field) return s.substring(start, i);
      n++;
      start = i + 1;
    }
  }
  return "";
}

bool parseRmcEpoch(const String &line, uint64_t &epoch) {
  String tim = csv(line, 1);
  String stat = csv(line, 2);
  String dat = csv(line, 9);

  if (stat != "A" || tim.length() < 6 || dat.length() < 6) return false;

  int hh = tim.substring(0,2).toInt();
  int mm = tim.substring(2,4).toInt();
  int ss = tim.substring(4,6).toInt();

  int dd = dat.substring(0,2).toInt();
  int mo = dat.substring(2,4).toInt();
  int yy = dat.substring(4,6).toInt();
  int year = yy >= 80 ? 1900 + yy : 2000 + yy;

  if (hh > 23 || mm > 59 || ss > 59 || dd < 1 || dd > 31 ||
      mo < 1 || mo > 12 || year < 2020 || year > 2099) return false;
  epoch = civilToEpoch(year, mo, dd, hh, mm, ss);
  return epoch != 0;
}

// ------------------------------------------------------------
// PCNT + PPS
// ------------------------------------------------------------

pcnt_unit_handle_t ocxoUnit = nullptr;
pcnt_channel_handle_t ocxoChan = nullptr;
pcnt_unit_handle_t rtc32Unit = nullptr;
pcnt_channel_handle_t rtc32Chan = nullptr;

// Interrupts merken ausschliesslich die Flankenzeit.
// PCNT-Treiber nicht im GPIO-ISR aufrufen: Flash/I2C/WLAN koennen
// sonst eine nicht-IRAM-sichere Treiberfunktion unterbrechen.
volatile uint32_t ppsSeqISR = 0;
volatile int64_t ppsMonoISR = 0;
volatile uint32_t ppsPeriodISR = 0;
volatile int64_t prevPpsMonoISR = 0;
volatile uint32_t sqwSeqISR = 0;
volatile int64_t sqwMonoISR = 0;
volatile uint32_t sqwPeriodISR = 0;
volatile int64_t prevSqwMonoISR = 0;
portMUX_TYPE pulseDataMux = portMUX_INITIALIZER_UNLOCKED;

// Separate snapshot helpers protect 64-bit values AND the matching period/sequence.
// Interrupt disable on the calling CPU alone is NOT a cross-core lock.
void snapshotPps(uint32_t &seq, uint32_t &period, int64_t &mono) {
  portENTER_CRITICAL(&pulseDataMux);
  seq=ppsSeqISR; period=ppsPeriodISR; mono=ppsMonoISR;
  portEXIT_CRITICAL(&pulseDataMux);
}
void snapshotSqw(uint32_t &seq, uint32_t &period, int64_t &mono) {
  portENTER_CRITICAL(&pulseDataMux);
  seq=sqwSeqISR; period=sqwPeriodISR; mono=sqwMonoISR;
  portEXIT_CRITICAL(&pulseDataMux);
}
void snapshotBothPulses(uint32_t &ppsSeq, uint32_t &ppsPeriod, int64_t &ppsMono,
                        uint32_t &sqwSeq, uint32_t &sqwPeriod, int64_t &sqwMono) {
  portENTER_CRITICAL(&pulseDataMux);
  ppsSeq=ppsSeqISR; ppsPeriod=ppsPeriodISR; ppsMono=ppsMonoISR;
  sqwSeq=sqwSeqISR; sqwPeriod=sqwPeriodISR; sqwMono=sqwMonoISR;
  portEXIT_CRITICAL(&pulseDataMux);
}

// Die Aufwaermzeit ist eine KONFIGURIERBARE Heuristik ab Start, keine
// Messung der Innentemperatur des OCXO. Kein Sensor am Quarzofen vorhanden.
constexpr uint32_t OCXO_STARTUP_OBSERVATION_MS = 30UL * 60UL * 1000UL;
enum RunPhase : uint8_t { PHASE_WARMUP=0, PHASE_GPS_WAIT=1, PHASE_RUNNING=2 };
uint32_t measurementStartedMs = 0;
RunPhase runPhase = PHASE_WARMUP;
// Diese drei Diagnose-Spalten sind bewusst NICHT exklusiv:
// [0] waehrend Anlauf, [1] gleichzeitig ohne GPS-Lock, [2] stabiler Betrieb.
uint32_t badWindowsByPhase[3] = {};
uint32_t rejectedOcxoByPhase[3] = {};
uint32_t overflowSuspectsByPhase[3] = {};
uint32_t rejectedGpsReference = 0;
bool warmupActive() {
  return !measurementStartedMs ||
         (uint32_t)(millis()-measurementStartedMs) < OCXO_STARTUP_OBSERVATION_MS;
}
void updateRunPhase() {
  if (warmupActive()) runPhase = PHASE_WARMUP;
  else if (!gpsLocked()) runPhase = PHASE_GPS_WAIT;
  else runPhase = PHASE_RUNNING;
}
const char *runPhaseText(RunPhase p) {
  return p==PHASE_WARMUP ? "ANLAUF (HEURISTIK)" :
         p==PHASE_GPS_WAIT ? "GPS WARTEN" : "MESSBETRIEB";
}
void countDiagEvent(uint32_t dst[3]) {
  const bool warm = warmupActive();
  const bool nogps = !gpsLocked();
  if (warm) dst[0]++;
  if (nogps) dst[1]++;
  if (!warm && !nogps) dst[2]++;
}

bool counterOCOK = false, counter32OK = false;
uint32_t ocxoSignalChanges = 0;
uint32_t counterInvalidWindows = 0;       // unrettbare PPS-nahe PCNT-Snapshots
uint32_t ocxoOverflowSuspects = 0;        // nur Diagnose, keine bestaetigten Verluste
uint32_t ocxoPpsRejects = 0;
uint32_t counterDiscontinuities = 0;
uint32_t pcntDriverReadErrors = 0;
uint32_t pcntReadAttempts = 0;
uint32_t pcntRetryReads = 0;
uint32_t pcntRecoveredReads = 0;
uint32_t pcntRecoveredSpans = 0;
uint32_t badOcxoPps = 0, badRtc32Pps = 0, badGpsPps = 0;
uint32_t lastBadCounterMs = 0;
uint32_t lastBadOcxoDelta = 0;
double lastBadOcxoPpm = NAN;
uint32_t ppsExtrapolationWarnings = 0;

bool ocxoSignal = false, rtc32Signal = false;
uint32_t lastGoodOcxoCounterMs = 0, lastGoodRtc32CounterMs = 0;
bool ocxoCalibrated = false;
double lastCalibratedOcxoHz = NAN;
float ocxoEstimatedPpm = 0.0f;

constexpr int PCNT_OVERFLOW_LIMIT = 32767; // max. positiver 16-Bit-Grenzwert
// 1.2.3: PCNT und Arduino-GPIO-ISR sind IRAM-sicher. Flash-Aktivitaet wird
// nur noch protokolliert, nicht pauschal als Messfehler gewertet. PCNT zaehlt
// weiter, waehrend der Mess-Task ggf. kurz nicht laeuft.
volatile uint32_t lastFlashActivityMs = 0;
uint32_t flashGuardRejectedPps = 0; // Kompatibilitaet: jetzt = PPS waehrend Flashaktivitaet
void markFlashActivity() { lastFlashActivityMs = millis(); }
bool flashGuardActive() {
  return lastFlashActivityMs && (uint32_t)(millis()-lastFlashActivityMs) < 3500U;
}

bool setupCounter(pcnt_unit_handle_t &unit,
                  pcnt_channel_handle_t &chan, int gpio) {
  pcnt_unit_config_t uc = {};
  uc.high_limit = PCNT_OVERFLOW_LIMIT;
  uc.low_limit = -32768;
  uc.intr_priority = 2; // mehr Prioritaet als Default, KEIN Ersatz fuer IRAM-SAFE
  uc.flags.accum_count = 1;
  if (pcnt_new_unit(&uc, &unit) != ESP_OK) return false;
  pcnt_chan_config_t cc = {};
  cc.edge_gpio_num = gpio;
  cc.level_gpio_num = -1;
  if (pcnt_new_channel(unit, &cc, &chan) != ESP_OK) return false;
  if (pcnt_channel_set_edge_action(chan,
      PCNT_CHANNEL_EDGE_ACTION_INCREASE,
      PCNT_CHANNEL_EDGE_ACTION_HOLD) != ESP_OK) return false;
  if (pcnt_unit_add_watch_point(unit, PCNT_OVERFLOW_LIMIT) != ESP_OK) return false;
  if (pcnt_unit_add_watch_point(unit, -32768) != ESP_OK) return false;
  if (pcnt_unit_enable(unit) != ESP_OK) return false;
  if (pcnt_unit_clear_count(unit) != ESP_OK) return false;
  return pcnt_unit_start(unit) == ESP_OK;
}

struct PcntReadSample {
  uint32_t count = 0;
  int64_t mono = 0;
  uint32_t callUs = 0; // Dauer von pcnt_unit_get_count(), reine Diagnose
  bool ok = false;
};

// 1.2.11: Rohspur der unmittelbar aufeinanderfolgenden PCNT-Leseversuche inkl. Aufrufdauer.
// Nur Diagnose; die Messentscheidung selbst bleibt unveraendert.
struct PcntReadTrace {
  PcntReadSample a, b, c;
  bool thirdRead = false;
};

enum PcntFailureKind : uint8_t {
  PCNT_FAIL_NONE = 0,
  PCNT_FAIL_SNAPSHOT,
  PCNT_FAIL_OCXO_INTERVAL,
  PCNT_FAIL_RTC32_INTERVAL
};

struct PcntRunFailureSnapshot {
  bool valid = false;
  PcntFailureKind kind = PCNT_FAIL_NONE;
  uint32_t uptimeMs = 0;
  uint32_t ppsSeq = 0;
  bool ocOk = false, r32Ok = false;
  PcntReadTrace ocTrace, r32Trace;
  uint32_t intervalPrevRaw = 0, intervalCurrRaw = 0, intervalSpan = 0;
  double intervalPerSecond = NAN;
  uint32_t flashAgeMs = UINT32_MAX;
  uint32_t logAgeMs = UINT32_MAX;
  int8_t rtcAging = 0;
  uint32_t rtcCalElapsedMs = 0;
  uint32_t rtcCalTargetMs = 0;
  uint8_t rtcCalLastReason = 0;
};

PcntRunFailureSnapshot firstRunPcntFailure;
uint32_t pcntOcxoUnrecoverable = 0;
uint32_t pcntRtc32Unrecoverable = 0;
uint32_t pcntOcxoUnrecoverableRun = 0;
uint32_t pcntRtc32UnrecoverableRun = 0;


PcntReadSample readPcntOnce(pcnt_unit_handle_t unit, bool ready) {
  PcntReadSample r;
  if (!ready || !unit) return r;
  int value = 0;
  const int64_t a = esp_timer_get_time();
  const esp_err_t e = pcnt_unit_get_count(unit, &value);
  const int64_t b = esp_timer_get_time();
  const int64_t callDt = b-a;
  r.callUs = callDt <= 0 ? 0U :
             (callDt > (int64_t)UINT32_MAX ? UINT32_MAX : (uint32_t)callDt);
  pcntReadAttempts++;
  if (e != ESP_OK) {
    pcntDriverReadErrors++;
    return r;
  }
  r.count = (uint32_t)value; // modulo 2^32; Delta wird ebenfalls modulo gebildet
  r.mono = (a+b)/2;
  r.ok = true;
  return r;
}

bool pcntPairPlausible(const PcntReadSample &a, const PcntReadSample &b,
                       double nominalHz) {
  if (!a.ok || !b.ok) return false;
  const int64_t dtUs = b.mono-a.mono;
  if (dtUs < 0 || dtUs > 5000) return false;
  const int32_t d = (int32_t)(b.count-a.count); // kurzer Abstand: echte Differenz << 2^31
  const double expected = nominalHz * (double)dtUs * 1e-6;
  const double tol = nominalHz > 1000000.0 ? fmax(160.0, expected*0.08)
                                           : fmax(4.0, expected*0.25);
  return d >= -2 && fabs((double)d-expected) <= tol;
}

void notePcntPairFailure(const PcntReadSample &a, const PcntReadSample &b,
                         double nominalHz, bool ocxo) {
  if (!a.ok || !b.ok) return;
  const int64_t dtUs=b.mono-a.mono;
  if (dtUs<0 || dtUs>5000) return;
  const int32_t d=(int32_t)(b.count-a.count);
  const double expected=nominalHz*(double)dtUs*1e-6;
  const double err=fabs((double)d-expected);
  const double units=round(err/(double)PCNT_OVERFLOW_LIMIT);
  if (ocxo && units>=1.0 && units<=8.0 &&
      fabs(err-units*(double)PCNT_OVERFLOW_LIMIT)<2200.0) {
    ocxoOverflowSuspects+=(uint32_t)units;
    // Unabhaengige Diagnoseachsen: Anlauf und GPS-Verlust koennen ueberlappen.
    for (uint32_t i=0;i<(uint32_t)units;i++) countDiagEvent(overflowSuspectsByPhase);
  }
}

// Liest denselben PCNT unmittelbar mehrfach. Ein einzelner Read nahe einem
// Overflow darf die PPS-Basis nicht verschieben. Erst ein konsistentes Paar
// wird akzeptiert. Das reduziert die bekannte HW/SW-Overflow-Race-Flaeche.
bool readPcntStable(pcnt_unit_handle_t unit, bool ready, double nominalHz,
                    bool ocxo, uint32_t &count, int64_t &mono, bool &recovered,
                    PcntReadTrace &trace) {
  recovered=false;
  trace=PcntReadTrace{};
  PcntReadSample a=readPcntOnce(unit,ready);
  PcntReadSample b=readPcntOnce(unit,ready);
  trace.a=a; trace.b=b;
  if (pcntPairPlausible(a,b,nominalHz)) {
    count=b.count; mono=b.mono; return true;
  }
  pcntRetryReads++;
  PcntReadSample c=readPcntOnce(unit,ready);
  trace.c=c; trace.thirdRead=true;
  if (pcntPairPlausible(b,c,nominalHz) || pcntPairPlausible(a,c,nominalHz)) {
    count=c.count; mono=c.mono; recovered=true; pcntRecoveredReads++; return true;
  }
  notePcntPairFailure(a,b,nominalHz,ocxo);
  notePcntPairFailure(b,c,nominalHz,ocxo);
  counterReadFailures++;
  const bool inRun=!warmupActive() && gpsLocked();
  if (ocxo) {
    pcntOcxoUnrecoverable++;
    if (inRun) pcntOcxoUnrecoverableRun++;
  } else {
    pcntRtc32Unrecoverable++;
    if (inRun) pcntRtc32UnrecoverableRun++;
  }
  return false;
}

struct PpsCounterRead {
  bool ocOk=false, r32Ok=false;
  bool ocRecovered=false, r32Recovered=false;
  uint32_t ocRaw=0, r32Raw=0;
  double ocDelayS=NAN, r32DelayS=NAN;
  PcntReadTrace ocTrace, r32Trace;
};

void captureFirstRunPcntFailure(PcntFailureKind kind, uint32_t ppsSeq,
                                const PpsCounterRead &r,
                                uint32_t intervalPrevRaw=0,
                                uint32_t intervalCurrRaw=0,
                                uint32_t intervalSpan=0,
                                double intervalPerSecond=NAN);

PpsCounterRead readCountersForPps(int64_t ppsMono, uint32_t ppsSeq) {
  PpsCounterRead r;
  int64_t ocMono=0, r32Mono=0;
  r.ocOk=readPcntStable(ocxoUnit,counterOCOK,10000000.0,true,
                        r.ocRaw,ocMono,r.ocRecovered,r.ocTrace);
  r.r32Ok=readPcntStable(rtc32Unit,counter32OK,32768.0,false,
                         r.r32Raw,r32Mono,r.r32Recovered,r.r32Trace);
  if (r.ocOk) {
    r.ocDelayS=(ocMono-ppsMono)*1e-6;
    if (r.ocDelayS < -0.002 || r.ocDelayS > 0.750) r.ocOk=false;
  }
  if (r.r32Ok) {
    r.r32DelayS=(r32Mono-ppsMono)*1e-6;
    if (r.r32DelayS < -0.002 || r.r32DelayS > 0.750) r.r32Ok=false;
  }
  if (!r.ocOk || !r.r32Ok) {
    counterInvalidWindows++;
    counterDiscontinuities++;
    countDiagEvent(badWindowsByPhase);
    captureFirstRunPcntFailure(PCNT_FAIL_SNAPSHOT,ppsSeq,r);
  }
  return r;
}

void IRAM_ATTR onGpsPps() {
  int64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&pulseDataMux);
  if (prevPpsMonoISR) ppsPeriodISR = (uint32_t)(now - prevPpsMonoISR);
  prevPpsMonoISR = now;
  ppsMonoISR = now;
  ppsSeqISR++;
  portEXIT_CRITICAL_ISR(&pulseDataMux);
}

void IRAM_ATTR onRtcSqw() {
  int64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&pulseDataMux);
  if (prevSqwMonoISR) sqwPeriodISR = (uint32_t)(now - prevSqwMonoISR);
  prevSqwMonoISR = now;
  sqwMonoISR = now;
  sqwSeqISR++;
  portEXIT_CRITICAL_ISR(&pulseDataMux);
}

bool gpsRefValid() {
  // Nur Messkern/Core1. Der NTP-Worker verwendet ntpGpsReady().
  uint32_t seq, period; int64_t mono;
  snapshotPps(seq, period, mono);
  const int64_t ppsAge = esp_timer_get_time() - mono;
  return gpsFix && gpsSats >= 4 &&
         seq && period >= 990000 && period <= 1010000 &&
         (uint32_t)(millis() - lastFixMs) < 3000 &&
         (uint32_t)(millis() - lastRmcMs) < 3000 &&
         mono > 0 && ppsAge >= 0 && ppsAge < 2000000;
}

bool gpsLocked() {
  return gpsRefValid() && stableUtcSeconds >= 30 && gpsEpochMapped &&
         gpsBestCno >= 15 && (antennaStatus != ANT_SHORT);
}

// Warum ging gpsLocked() verloren? Die Bits bilden exakt die Kriterien der
// bestehenden Lock-Pruefung ab, ohne deren Verhalten zu aendern.
enum GpsLockFailIndex : uint8_t {
  GPS_FAIL_FIX=0, GPS_FAIL_SATS, GPS_FAIL_PPS, GPS_FAIL_RMC,
  GPS_FAIL_STABLE, GPS_FAIL_MAP, GPS_FAIL_CNO, GPS_FAIL_ANT,
  GPS_FAIL_COUNT
};
constexpr uint16_t GPS_FAIL_BIT(GpsLockFailIndex i) { return (uint16_t)(1U << (uint8_t)i); }

uint16_t gpsLockFailureMask() {
  uint32_t seq, period; int64_t mono;
  snapshotPps(seq, period, mono);
  const uint32_t now=millis();
  const int64_t ppsAge=mono>0 ? esp_timer_get_time()-mono : INT64_MAX;
  uint16_t m=0;
  if (!gpsFix || (uint32_t)(now-lastFixMs)>=3000U) m|=GPS_FAIL_BIT(GPS_FAIL_FIX);
  if (gpsSats<4) m|=GPS_FAIL_BIT(GPS_FAIL_SATS);
  if (!seq || period<990000 || period>1010000 || mono<=0 || ppsAge<0 || ppsAge>=2000000LL)
    m|=GPS_FAIL_BIT(GPS_FAIL_PPS);
  if ((uint32_t)(now-lastRmcMs)>=3000U) m|=GPS_FAIL_BIT(GPS_FAIL_RMC);
  if (stableUtcSeconds<30) m|=GPS_FAIL_BIT(GPS_FAIL_STABLE);
  if (!gpsEpochMapped) m|=GPS_FAIL_BIT(GPS_FAIL_MAP);
  if (gpsBestCno<15) m|=GPS_FAIL_BIT(GPS_FAIL_CNO);
  if (antennaStatus==ANT_SHORT) m|=GPS_FAIL_BIT(GPS_FAIL_ANT);
  return m;
}

uint32_t gpsUnlockEvents=0;
uint32_t gpsUnlockSinceMs=0;
uint32_t gpsUnlockLastDurationMs=0;
uint32_t gpsUnlockMaxDurationMs=0;
uint16_t gpsUnlockCurrentReasonMask=0;
uint16_t gpsUnlockLastReasonMask=0;
bool gpsLockDiagSeenLock=false;
bool gpsLockDiagPrevLocked=false;

void gpsLockDiagTick() {
  const bool locked=gpsLocked();
  const uint32_t now=millis();
  if (!gpsLockDiagSeenLock) {
    if (locked) { gpsLockDiagSeenLock=true; gpsLockDiagPrevLocked=true; }
    return; // Startphase nicht als Aussetzer zaehlen.  }
  if (locked) {
    if (!gpsLockDiagPrevLocked && gpsUnlockSinceMs) {
      const uint32_t d=now-gpsUnlockSinceMs;
      gpsUnlockLastDurationMs=d;
      if (d>gpsUnlockMaxDurationMs) gpsUnlockMaxDurationMs=d;
      gpsUnlockLastReasonMask=gpsUnlockCurrentReasonMask;
      gpsUnlockSinceMs=0;
      gpsUnlockCurrentReasonMask=0;
    }
  } else {
    const uint16_t reason=gpsLockFailureMask();
    if (gpsLockDiagPrevLocked) {
      gpsUnlockEvents++;
      gpsUnlockSinceMs=now;
      gpsUnlockCurrentReasonMask=reason;
    } else {
      gpsUnlockCurrentReasonMask|=reason;
    }
  }
  gpsLockDiagPrevLocked=locked;
}

bool validNmeaChecksum(const String &s) {
  int star = s.lastIndexOf('*');
  if (s.length() < 7 || s[0] != '$' || star < 0 || star + 2 >= (int)s.length()) return false;
  uint8_t calc = 0;
  for (int i = 1; i < star; ++i) calc ^= (uint8_t)s[i];
  auto val = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  };
  int hi = val(s[star+1]), lo = val(s[star+2]);
  return hi >= 0 && lo >= 0 && calc == (uint8_t)((hi<<4) | lo);
}

bool gpsEpochForPps(uint32_t seq, uint64_t &epoch) {
  if (!gpsEpochMapped || seq < mappedPpsSeq) return false;
  uint32_t d = seq - mappedPpsSeq;
  if (d > 10) return false;
  epoch = mappedPpsEpoch + d;
  return true;
}

void parseNmea(const String &line) {
  if (line.startsWith("$GPRMC") || line.startsWith("$GNRMC")) {
    gpsFix = csv(line, 2) == "A";
    if (gpsFix) lastFixMs = millis();
    else { stableUtcSeconds = 0; gpsEpochMapped = false; }

    uint64_t e;
    if (parseRmcEpoch(line, e)) {
      lastRmcEpoch = e;
      lastRmcMs = millis();

      uint32_t seq, period;
      int64_t ppsMono;
      snapshotPps(seq, period, ppsMono);

      int64_t age = esp_timer_get_time() - ppsMono;
      if (seq && age >= 0 && age < 1500000) {
        gpsEpochMapped = true;
        mappedPpsSeq = seq;
        mappedPpsMono = ppsMono;
        // Sekundenzuordnung ist modulspezifisch: hier per Konstante dokumentiert.
        mappedPpsEpoch = (uint64_t)((int64_t)e + RMC_PPS_ADJUST_SECONDS);
        if (previousRmcEpoch != 0 && e == previousRmcEpoch + 1 &&
            stableLastMappedSeq + 1 == seq && gpsSats >= 4) {
          stableUtcSeconds = min((int)stableUtcSeconds + 1, 1000);
        } else {
          if (previousRmcEpoch && (e != previousRmcEpoch+1 || stableLastMappedSeq+1 != seq))
            gnssTimeAnomalies++;
          stableUtcSeconds = 1;
        }
        stableLastMappedSeq = seq;
        previousRmcEpoch = e;
      }
    }
  }

  if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) {
    String fix = csv(line, 6);
    String sats = csv(line, 7);
    String hdop = csv(line, 8);

    if (sats.length()) gpsSats = sats.toInt();
    gpsHdop = hdop.length() ? hdop.toFloat() : NAN;
    if (fix.toInt() > 0) {
      gpsFix = true;
      lastFixMs = millis();
    }
  }

  if ((line.startsWith("$GP") || line.startsWith("$GN") ||
       line.startsWith("$BD") || line.startsWith("$GB") ||
       line.startsWith("$GA")) &&
      line.substring(3, 6) == "GSA") {
    // GSA zeigt den Fix-Typ; bei GNSS-Multisystemen koennen mehrere GSA
    // kommen. Kein Navigationsfix darf nicht durch alte Werte ersetzt werden.
    int ft = csv(line, 2).toInt();
    if (ft >= 1 && ft <= 3) gpsFixType = ft;
  }

  // GPS, BeiDou, GLONASS, Galileo und zusammengefasste GN-GSV.
  // Die Signale pro Talker erst nach einer kompletten GSV-Serie publizieren.
  if (line.length() >= 6 && line[0] == '$' &&
      line.substring(3, 6) == "GSV") {
    String talker = line.substring(1, 3);
    int idx = -1;
    for (uint8_t i = 0; i < GNSS_GROUP_COUNT; ++i) {
      if (talker == gnssTalkers[i]) { idx = i; break; }
    }
    if (idx < 0) return;

    int totalMsgs = csv(line, 1).toInt();
    int msgNo = csv(line, 2).toInt();
    int visible = csv(line, 3).toInt();
    if (totalMsgs < 1 || totalMsgs > 32 ||
        msgNo < 1 || msgNo > totalMsgs || visible < 0) return;

    // Einzelne GSV-Saetze einer Serie duerfen nicht zu frueheren Serien
    // aufsummiert werden. Msg 1 startet immer neu.
    if (msgNo == 1) {
      gnssWorkSignal[idx] = 0;
      gnssWorkBest[idx] = 0;
      gnssNextMsg[idx] = 1;
    }
    if (gnssNextMsg[idx] != msgNo) return;

    for (int f = 4; f <= 16; f += 4) {
      String prn = csv(line, f);
      if (!prn.length() || prn.toInt() <= 0) continue;
      String cnoString = csv(line, f + 3);
      if (!cnoString.length()) continue;
      int cno = cnoString.toInt();
      if (cno <= 0 || cno > 99) continue;
      gnssWorkSignal[idx]++;
      if (cno > gnssWorkBest[idx]) gnssWorkBest[idx] = (uint8_t)cno;
    }

    gnssNextMsg[idx] = msgNo + 1;
    if (msgNo == totalMsgs) {
      gnssVisible[idx] = (uint16_t)min(visible, 255);
      gnssSignal[idx] = gnssWorkSignal[idx];
      gnssBestCno[idx] = gnssWorkBest[idx];
      gnssUpdateMs[idx] = millis();
      gnssNextMsg[idx] = 0;
      updateGnssSummary();
    }
    return;
  }

  if (line.length() >= 6 && line[0] == '$' &&
      line.substring(3, 6) == "TXT") {
    if (line.indexOf("ANTENNA OK") >= 0) antennaStatus = ANT_OK;
    else if (line.indexOf("ANTENNA OPEN") >= 0) antennaStatus = ANT_OPEN;
    else if (line.indexOf("ANTENNA SHORT") >= 0) antennaStatus = ANT_SHORT;
    else return;
    antennaLastMs = millis();
    Serial.printf("GNSS Antenne: %s\n", antennaText());
  }
}

void pollGps() {
  while (GPS.available()) {
    char c = GPS.read();
    lastGpsByteMs = millis();

    if (c == '\n') {
      nmea.trim();
      if (nmea.length()) {
        if (validNmeaChecksum(nmea)) { nmeaOk++; parseNmea(nmea); }
        else nmeaBad++;
      }
      nmea = "";
    } else if (c != '\r' && nmea.length() < 180) {
      nmea += c;
    }
  }

  if (millis() - lastFixMs > 3000) { gpsFix = false; stableUtcSeconds = 0; }
  // Auch das Auslaufen alter GSV-Serien kontrollieren, aber nicht
  // bei jedem einzelnen Arduino-Loop die Gruppen erneut durchlaufen.
  static uint32_t lastSummaryMs=0;
  if ((uint32_t)(millis()-lastSummaryMs)>=250) {
    lastSummaryMs=millis();
    updateGnssSummary();
  }
}

// ------------------------------------------------------------
// Frequenzmessung
// ------------------------------------------------------------

double ocxo1 = NAN, ocxo10 = NAN, ocxo60 = NAN;
double ocxo100 = NAN, ocxo300 = NAN, ocxo1000 = NAN;
double rtc321 = NAN, rtc3210 = NAN, rtc3260 = NAN;
double rtc32100 = NAN, rtc32300 = NAN, rtc321000 = NAN;

double ocxoPpm10 = NAN, ocxoPpm60 = NAN;
double ocxoPpm100 = NAN, ocxoPpm300 = NAN, ocxoPpm1000 = NAN;
double rtc32Ppm60 = NAN, rtc32Ppm100 = NAN, rtc32Ppm300 = NAN, rtc32Ppm1000 = NAN;

// Letzte 1000 gueltige, zusammenhaengende GPS-PPS-Intervalle.
// Die PPS-nahe Software-Rueckextrapolation liefert Bruchteile eines Counts;
// diese nicht pro Sekunde runden, erst ueber das gesamte Fenster mitteln.
constexpr int FREQ_HISTORY = 1000;
double ocHist[FREQ_HISTORY] = {};
double r32Hist[FREQ_HISTORY] = {};
int histPos = 0;
int histCount = 0;

uint32_t processedPps = 0;
int64_t prevPpsMono = 0;
// Kanalweise PPS-Basen. Bei einem schlechten Read bleibt die letzte gute Basis
// stehen; der naechste gute Read kann den gesamten Mehrsekunden-Abstand retten.
bool havePrevOcSnap=false, havePrevR32Snap=false;
uint32_t prevOcRaw=0, prevR32Raw=0;
double prevOcDelayS=0.0, prevR32DelayS=0.0;
uint32_t prevOcSeq=0, prevR32Seq=0;
// Nur Diagnose der jeweils letzten PPS-nahen Rohstaende.
double ocxoPpsTicks = 0, rtc32PpsTicks = 0;
uint32_t counterPpsSeq = 0;
uint32_t gnssLostEvents = 0;
uint32_t lastGoodPpsMs = 0;
uint32_t lastHandledDiscontinuity = 0; // Diagnosekompatibilitaet
uint32_t histValidOcxo100 = 0, histValidRtc100 = 0;

void resetFreqWindows() {
  histPos = 0;
  histCount = 0;

  ocxo1 = ocxo10 = ocxo60 = NAN;
  ocxo100 = ocxo300 = ocxo1000 = NAN;

  rtc321 = rtc3210 = rtc3260 = NAN;
  rtc32100 = rtc32300 = rtc321000 = NAN;

  ocxoPpm10 = ocxoPpm60 = NAN;
  ocxoPpm100 = ocxoPpm300 = ocxoPpm1000 = NAN;

  rtc32Ppm60 = rtc32Ppm100 = rtc32Ppm300 = rtc32Ppm1000 = NAN;
}

int validCount(const double *a, int wanted) {
  if (histCount < wanted) return 0;
  int valid = 0;
  for (int back=1; back<=wanted; ++back) {
    int i=histPos-back;
    if (i<0) i+=FREQ_HISTORY;
    if (isfinite(a[i])) ++valid;
  }
  return valid;
}

double meanCounts(const double *a, int wanted) {
  if (histCount < wanted) return NAN;
  double sum = 0.0;
  int good = 0;
  for (int back=1; back<=wanted; ++back) {
    int i=histPos-back;
    if (i<0) i+=FREQ_HISTORY;
    if (isfinite(a[i])) { sum+=a[i]; ++good; }
  }
  // Anders als die Vorversion: Wenige verworfene PPS-Sekunden duerfen das
  // gesamte 1000-s-Fenster nicht fuer 1000 Sekunden zerstoeren.
  // Die Mittelung verwendet ausschliesslich echte gueltige Sekunden.
  if (good < (wanted*98+99)/100) return NAN;
  return sum / good;
}

void pushSample(double oc, double r32) {
  ocHist[histPos] = oc;
  r32Hist[histPos] = r32;

  histPos = (histPos + 1) % FREQ_HISTORY;
  if (histCount < FREQ_HISTORY) histCount++;

  ocxo1 = oc;
  rtc321 = r32;

  ocxo10 = meanCounts(ocHist, 10);
  ocxo60 = meanCounts(ocHist, 60);
  ocxo100 = meanCounts(ocHist, 100);
  ocxo300 = meanCounts(ocHist, 300);
  ocxo1000 = meanCounts(ocHist, 1000);

  rtc3210 = meanCounts(r32Hist, 10);
  rtc3260 = meanCounts(r32Hist, 60);
  rtc32100 = meanCounts(r32Hist, 100);
  rtc32300 = meanCounts(r32Hist, 300);
  rtc321000 = meanCounts(r32Hist, 1000);

  ocxoPpm10 = ocxoPpm60 = ocxoPpm100 = ocxoPpm300 = ocxoPpm1000 = NAN;
  rtc32Ppm60 = rtc32Ppm100 = rtc32Ppm300 = rtc32Ppm1000 = NAN;

  if (!isnan(ocxo10))   ocxoPpm10   = (ocxo10   - 10000000.0) / 10.0;
  if (!isnan(ocxo60))   ocxoPpm60   = (ocxo60   - 10000000.0) / 10.0;
  if (!isnan(ocxo100))  ocxoPpm100  = (ocxo100  - 10000000.0) / 10.0;
  if (!isnan(ocxo300))  ocxoPpm300  = (ocxo300  - 10000000.0) / 10.0;
  if (!isnan(ocxo1000)) ocxoPpm1000 = (ocxo1000 - 10000000.0) / 10.0;
  if (isfinite(ocxo100) && validCount(ocHist,100)==100 && fabs(ocxoPpm100) <= 20.0) {
    lastCalibratedOcxoHz = isfinite(ocxo1000) ? ocxo1000 :
                           (isfinite(ocxo300) ? ocxo300 : ocxo100);
    ocxoCalibrated = true;
    ocxoEstimatedPpm = (float)((lastCalibratedOcxoHz - 10000000.0) / 10.0);
  } else {
    ocxoCalibrated = false;
    lastCalibratedOcxoHz = NAN;
  }

  if (!isnan(rtc3260))   rtc32Ppm60   = (rtc3260   - 32768.0) / 32768.0 * 1e6;
  if (!isnan(rtc32100))  rtc32Ppm100  = (rtc32100  - 32768.0) / 32768.0 * 1e6;
  if (!isnan(rtc32300))  rtc32Ppm300  = (rtc32300  - 32768.0) / 32768.0 * 1e6;
  if (!isnan(rtc321000)) rtc32Ppm1000 = (rtc321000 - 32768.0) / 32768.0 * 1e6;
}


// ------------------------------------------------------------
// Allan Deviation (ADEV) des OCXO
// ------------------------------------------------------------
// Klassische Allan-Deviation aus benachbarten, NICHT ueberlappenden
// Mittelwertbloecken. Eingabe sind ausschliesslich echte, einzelne 1-s-
// OCXO-Intervalle relativ zu GPS-PPS. Ein verworfenes PPS-Intervall oder ein
// per Mehrsekunden-Span geretteter PCNT-Wert unterbricht die zusammenhaengende
// ADEV-Serie; ueber eine Luecke werden niemals zwei Bloecke verglichen.
constexpr int ADEV_TAU_COUNT=8;
constexpr uint16_t ADEV_TAU_S[ADEV_TAU_COUNT]={1,2,5,10,30,100,300,1000};
struct AdevAccumulator {
  uint16_t blockCount=0;
  double blockSum=0.0;
  double prevBlockMean=0.0;
  bool havePrevBlock=false;
  double sumSqDiff=0.0;
  uint32_t pairs=0;
};
AdevAccumulator adevAcc[ADEV_TAU_COUNT];
portMUX_TYPE adevDataMux=portMUX_INITIALIZER_UNLOCKED;
uint32_t adevPrevSeq=0;
uint32_t adevValidSeconds=0;
uint32_t adevRejectedSeconds=0;
uint32_t adevCurrentRun=0;

void resetAdevSegmentLocked() {
  adevCurrentRun=0;
  for (int i=0;i<ADEV_TAU_COUNT;i++) {
    adevAcc[i].blockCount=0;
    adevAcc[i].blockSum=0.0;
    adevAcc[i].prevBlockMean=0.0;
    adevAcc[i].havePrevBlock=false;
  }
}

void resetAdev() {
  portENTER_CRITICAL(&adevDataMux);
  memset(adevAcc,0,sizeof(adevAcc));
  adevPrevSeq=0;
  adevValidSeconds=0;
  adevRejectedSeconds=0;
  adevCurrentRun=0;
  portEXIT_CRITICAL(&adevDataMux);
}

void adevPush(uint32_t seq, bool validOneSecond, double hz) {
  const bool numeric=validOneSecond && isfinite(hz);
  const double y=numeric ? (hz-10000000.0)/10000000.0 : 0.0; // fractional frequency
  portENTER_CRITICAL(&adevDataMux);
  const bool seqBreak=adevPrevSeq && seq!=adevPrevSeq+1;
  if (seqBreak) {
    // Auch PPS-Sequenzen, die der Task komplett verpasst hat, als Luecke zaehlen.
    if (seq>adevPrevSeq+1) adevRejectedSeconds += seq-adevPrevSeq-1;
    resetAdevSegmentLocked();
  }
  adevPrevSeq=seq;
  if (!numeric) {
    adevRejectedSeconds++;
    resetAdevSegmentLocked();
    portEXIT_CRITICAL(&adevDataMux);
    return;
  }
  adevValidSeconds++;
  adevCurrentRun++;
  for (int i=0;i<ADEV_TAU_COUNT;i++) {
    AdevAccumulator &a=adevAcc[i];
    a.blockSum+=y;
    a.blockCount++;
    if (a.blockCount>=ADEV_TAU_S[i]) {
      const double mean=a.blockSum/(double)ADEV_TAU_S[i];
      if (a.havePrevBlock) {
        const double d=mean-a.prevBlockMean;
        a.sumSqDiff+=d*d;
        a.pairs++;
      }
      a.prevBlockMean=mean;
      a.havePrevBlock=true;
      a.blockCount=0;
      a.blockSum=0.0;
    }
  }
  portEXIT_CRITICAL(&adevDataMux);
}

void processPps() {
  uint32_t seq, period;
  int64_t mono;
  snapshotPps(seq, period, mono);
  if (!seq || seq == processedPps) return;

  const bool skipped = processedPps && seq != processedPps + 1;
  const int64_t dmono = prevPpsMono ? mono-prevPpsMono : 0;
  processedPps = seq;
  prevPpsMono = mono;

  // Genau EIN PPS-bezogener PCNT-Snapshot statt 100 Polls/s. Die Hardware
  // zaehlt autonom; Mehrfachlesen dient nur zur Race-Erkennung.
  PpsCounterRead cr=readCountersForPps(mono,seq);
  counterPpsSeq=seq;
  if (cr.ocOk) ocxoPpsTicks=(double)cr.ocRaw;
  if (cr.r32Ok) rtc32PpsTicks=(double)cr.r32Raw;

  if ((!cr.ocOk || !cr.r32Ok) ||
      (cr.ocOk && (cr.ocDelayS < -0.002 || cr.ocDelayS > 0.750)) ||
      (cr.r32Ok && (cr.r32DelayS < -0.002 || cr.r32DelayS > 0.750)))
    ppsExtrapolationWarnings++;

  const bool refGood = gpsRefValid() && !skipped &&
      (!dmono || (dmono > 990000 && dmono < 1010000)) &&
      period > 990000 && period < 1010000;
  if (!refGood) {
    badGpsPps++; gnssLostEvents++; rejectedGpsReference++;
  }
  if (flashGuardActive()) flashGuardRejectedPps++; // Diagnose, KEIN Reject mehr

  const double fOC=isfinite(ocxo300) ? ocxo300 :
                   (isfinite(ocxo100) ? ocxo100 : 10000000.0);
  const double f32=isfinite(rtc32100) ? rtc32100 : 32768.0;
  double doc=NAN, dr32=NAN;
  bool ocCounterGood=false, r32CounterGood=false;
  uint32_t ocCounterSpan=0; // ADEV akzeptiert nur echte einzelne 1-s-Intervalle

  if (cr.ocOk) {
    if (!havePrevOcSnap) {
      havePrevOcSnap=true; prevOcRaw=cr.ocRaw; prevOcDelayS=cr.ocDelayS; prevOcSeq=seq;
    } else {
      const uint32_t span=seq-prevOcSeq;
      if (span>=1 && span<=8) {
        const uint32_t rawDelta=(uint32_t)(cr.ocRaw-prevOcRaw);
        const double total=(double)rawDelta - cr.ocDelayS*fOC + prevOcDelayS*fOC;
        const double perSecond=total/(double)span;
        if (isfinite(perSecond) && fabs(perSecond-10000000.0)<=1000.0) {
          doc=perSecond;
          ocCounterGood=true;
          ocCounterSpan=span;
          if (span>1) pcntRecoveredSpans += span-1;
          const bool old=ocxoSignal;
          ocxoSignal=perSecond>5000000.0;
          if (old!=ocxoSignal) ocxoSignalChanges++;
          lastGoodOcxoCounterMs=millis();
          // Nur eine plausible neue Basis uebernimmt die Referenz.
          prevOcRaw=cr.ocRaw; prevOcDelayS=cr.ocDelayS; prevOcSeq=seq;
        } else {
          counterDiscontinuities++;
          counterInvalidWindows++;
          counterReadFailures++;
          countDiagEvent(badWindowsByPhase);
          lastBadCounterMs=millis();
          lastBadOcxoDelta=rawDelta;
          lastBadOcxoPpm=isfinite(perSecond) ? (perSecond-10000000.0)/10.0 : NAN;
          captureFirstRunPcntFailure(PCNT_FAIL_OCXO_INTERVAL,seq,cr,
                                     prevOcRaw,cr.ocRaw,span,perSecond);
          const double err=fabs(total-10000000.0*(double)span);
          const double units=round(err/(double)PCNT_OVERFLOW_LIMIT);
          if (units>=1.0 && units<=200.0 &&
              fabs(err-units*(double)PCNT_OVERFLOW_LIMIT)<2500.0) {
            ocxoOverflowSuspects+=(uint32_t)units;
            for (uint32_t i=0;i<(uint32_t)units;i++) countDiagEvent(overflowSuspectsByPhase);
          }
          // WICHTIG: Baseline bleibt stehen. Kein Kaskadenfehler durch einen Ausreisser.
        }
      } else {
        // Zu grosse Luecke: neue saubere Basis beginnen, aber diese Sekunde nicht bewerten.
        prevOcRaw=cr.ocRaw; prevOcDelayS=cr.ocDelayS; prevOcSeq=seq;
      }
    }
  }

  if (cr.r32Ok) {
    if (!havePrevR32Snap) {
      havePrevR32Snap=true; prevR32Raw=cr.r32Raw; prevR32DelayS=cr.r32DelayS; prevR32Seq=seq;
    } else {
      const uint32_t span=seq-prevR32Seq;
      if (span>=1 && span<=8) {
        const uint32_t rawDelta=(uint32_t)(cr.r32Raw-prevR32Raw);
        const double total=(double)rawDelta - cr.r32DelayS*f32 + prevR32DelayS*f32;
        const double perSecond=total/(double)span;
        if (isfinite(perSecond) && perSecond>31500.0 && perSecond<34000.0) {
          dr32=perSecond;
          r32CounterGood=true;
          rtc32Signal=perSecond>25000.0;
          lastGoodRtc32CounterMs=millis();
          prevR32Raw=cr.r32Raw; prevR32DelayS=cr.r32DelayS; prevR32Seq=seq;
        } else {
          counterDiscontinuities++;
          counterInvalidWindows++;
          counterReadFailures++;
          countDiagEvent(badWindowsByPhase);
          captureFirstRunPcntFailure(PCNT_FAIL_RTC32_INTERVAL,seq,cr,
                                     prevR32Raw,cr.r32Raw,span,perSecond);
          // Auch hier die gute Baseline NICHT durch einen Ausreisser ersetzen.
        }
      } else {
        prevR32Raw=cr.r32Raw; prevR32DelayS=cr.r32DelayS; prevR32Seq=seq;
      }
    }
  }

  if (lastGoodOcxoCounterMs && (uint32_t)(millis()-lastGoodOcxoCounterMs)>3500U)
    ocxoSignal=false;
  if (lastGoodRtc32CounterMs && (uint32_t)(millis()-lastGoodRtc32CounterMs)>3500U)
    rtc32Signal=false;

  const bool ocGood=refGood && ocCounterGood && counterOCOK && ocxoSignal;
  const bool rtGood=refGood && r32CounterGood && counter32OK && rtc32Signal;
  if (!ocGood) {
    badOcxoPps++; ocxoPpsRejects++; countDiagEvent(rejectedOcxoByPhase);
  }
  if (!rtGood) badRtc32Pps++;

  // Pro GPS-PPS genau ein History-Slot. Ein schlechter Read ergibt ein Loch;
  // der naechste gute Read kann seinen Mehrsekunden-Zaehlerabstand mitteln.
  pushSample(ocGood ? doc : NAN, rtGood ? dr32 : NAN);
  // ADEV darf einen ueber mehrere Sekunden gemittelten Rettungswert nicht als
  // scheinbares 1-s-Sample behandeln. Solche Stellen bilden harte Segmentgrenzen.
  adevPush(seq, ocGood && ocCounterSpan==1, doc);
  if (ocGood) lastGoodPpsMs=millis();
}

// ------------------------------------------------------------
// RTC SQW Phase / Drift
// ------------------------------------------------------------

uint32_t processedSqw = 0;
double rtcGpsPhaseMs = NAN;
double rtcGpsDriftMs = NAN;
double rtcBaselineMs = NAN;
double rtcLocalMs = NAN;
uint64_t lastRtcReadEpoch = 0;
bool lastRtcReadValid = false;
uint32_t lastRtcReadMs = 0;

int64_t foldSecond(int64_t us) {
  while (us > 500000) us -= 1000000;
  while (us < -500000) us += 1000000;
  return us;
}

void processSqw() {
  uint32_t seq, pseq, sqwPeriod, ppsPeriod;
  int64_t smono, pmono;
  snapshotBothPulses(pseq, ppsPeriod, pmono, seq, sqwPeriod, smono);

  if (!seq || seq == processedSqw) return;
  processedSqw = seq;

  uint64_t rtcEpoch;
  // Bei I2C- oder NTP-Ausfall keinen alten Offset als aktuellen Messwert zeigen.
  rtcLocalMs = NAN;
  lastRtcReadValid = rtcReadEpoch(rtcEpoch);
  lastRtcReadMs = millis();
  if (lastRtcReadValid) lastRtcReadEpoch = rtcEpoch;
  if (lastRtcReadValid && ntpFresh(NTP_LOCAL)) {
    uint64_t n = ntpAt(NTP_LOCAL, smono);
    if (n) rtcLocalMs =
        ((int64_t)(rtcEpoch * 1000000ULL) - (int64_t)n) / 1000.0;
  }

  if (!pseq || !gpsRefValid()) {
    rtcGpsPhaseMs = NAN;
    rtcLiveRateReset();
    return;
  }
  if (sqwPeriod < 990000 || sqwPeriod > 1010000 ||
      ppsPeriod < 990000 || ppsPeriod > 1010000) {
    rtcGpsPhaseMs = NAN;
    rtcLiveRateReset();
    return;
  }

  rtcGpsPhaseMs = foldSecond(smono - pmono) / 1000.0;
  rtcLiveRatePush(smono, rtcGpsPhaseMs);

  if (isnan(rtcBaselineMs)) {
    rtcBaselineMs = rtcGpsPhaseMs;
    rtcGpsDriftMs = 0;
  } else {
    double d = rtcGpsPhaseMs - rtcBaselineMs;
    while (d > 500) d -= 1000;
    while (d < -500) d += 1000;
    rtcGpsDriftMs = d;
  }
}

// ------------------------------------------------------------
// DS3231M Aging-Abgleich per 1-Hz-SQW relativ zu GPS-PPS.
// Keine 32-kHz-Rohfrequenz, kein OCXO-PCNT beteiligt.
// Langsame, begrenzte Korrektur, keine sprunghafte Sekundenverstellung.
// DS3231M: ein Aging-LSB veraendert den Gang typisch um -0.12 ppm.
// Die ESP-interne Chiptemperatur ist kein belastbares RTC-Temperaturmass und
// darf deshalb ein ansonsten gueltiges Kalibrierfenster nicht mehr verwerfen.
// ------------------------------------------------------------
int8_t rtcAgingOffset = 0;
bool rtcAgingReadable = false;
double rtcMeasuredRatePpm = NAN;
uint32_t rtcAgingAdjustments = 0;
uint32_t rtcCalStartMs = 0;
double rtcCalStartPhaseMs = NAN;
float rtcCalStartTempC = NAN;
uint32_t rtcCalLastSqwSeq = 0;
uint32_t rtcCalLastPpsSeq = 0;
uint32_t rtcCalRejected = 0;
uint32_t rtcCalLastAdjustmentMs = 0;
uint32_t rtcCalTargetMs = 10UL*60UL*1000UL;
bool rtcCalTargetChosen = false;
double rtcCalProvisionalPpm = NAN;
uint32_t rtcCalGpsFailCount[GPS_FAIL_COUNT] = {};
uint16_t rtcCalLastGpsFailMask = 0;
enum RtcCalResetReason : uint8_t {
  RTC_CAL_NONE, RTC_CAL_GPS, RTC_CAL_SQW, RTC_CAL_TEMPERATURE,
  RTC_CAL_PHASE, RTC_CAL_RATE, RTC_CAL_SYNC, RTC_CAL_RTC_STATE, RTC_CAL_FINISHED,
  RTC_CAL_CLEAR, RTC_CAL_REASON_COUNT
};
// Explizite Deklarationen vermeiden Arduino-Autoprototypen vor dem Enum.
const char *rtcCalReasonName(RtcCalResetReason r);
void rtcCalReset(RtcCalResetReason reason);
uint32_t rtcCalResetCount[RTC_CAL_REASON_COUNT] = {};
RtcCalResetReason rtcCalLastReason = RTC_CAL_NONE;
const char *rtcCalReasonName(RtcCalResetReason r) {
  switch (r) {
    case RTC_CAL_GPS: return "GPS/PPS";
    case RTC_CAL_SQW: return "SQW";
    case RTC_CAL_TEMPERATURE: return "TEMPERATUR";
    case RTC_CAL_PHASE: return "PHASE";
    case RTC_CAL_RATE: return "GANG/PLAUSI";
    case RTC_CAL_SYNC: return "RTC_SYNC";
    case RTC_CAL_RTC_STATE: return "RTC_STATUS";
    case RTC_CAL_FINISHED: return "ZYKLUS_ENDE";
    case RTC_CAL_CLEAR: return "MESSWERTE_GELOESCHT";
    default: return "KEIN";
  }
}
double foldedPhaseDeltaMs(double current, double start) {
  double delta = current - start;
  while (delta > 500.0) delta -= 1000.0;
  while (delta < -500.0) delta += 1000.0;
  return delta;
}

// Schneller, rein beobachtender RTC-Gang fuer Anzeige/Nullinstrument.
// Die adaptive Aging-Kalibrierung bleibt davon getrennt.
// Nach 30 s gibt es einen vorlaeufigen Wert; ab 120 s ist das Fenster voll.
constexpr uint16_t RTC_LIVE_RATE_MIN_S = 30;
constexpr uint16_t RTC_LIVE_RATE_TARGET_S = 120;
struct RtcLiveRatePoint {
  int64_t monoUs = 0;
  double phaseMs = NAN;
};
RtcLiveRatePoint rtcLiveRateHist[RTC_LIVE_RATE_TARGET_S + 1];
uint16_t rtcLiveRateCount = 0;
uint16_t rtcLiveRatePos = 0;
double rtcLiveRatePpm = NAN;
uint16_t rtcLiveRateSpanSec = 0;

void rtcLiveRateReset() {
  rtcLiveRateCount = 0;
  rtcLiveRatePos = 0;
  rtcLiveRatePpm = NAN;
  rtcLiveRateSpanSec = 0;
}

void rtcLiveRatePush(int64_t monoUs, double phaseMs) {
  if (!isfinite(phaseMs) || monoUs <= 0) {
    rtcLiveRateReset();
    return;
  }
  rtcLiveRateHist[rtcLiveRatePos] = {monoUs, phaseMs};
  rtcLiveRatePos = (rtcLiveRatePos + 1) % (RTC_LIVE_RATE_TARGET_S + 1);
  if (rtcLiveRateCount < RTC_LIVE_RATE_TARGET_S + 1) rtcLiveRateCount++;

  if (rtcLiveRateCount < RTC_LIVE_RATE_MIN_S + 1) {
    rtcLiveRatePpm = NAN;
    rtcLiveRateSpanSec = rtcLiveRateCount ? rtcLiveRateCount - 1 : 0;
    return;
  }

  const uint16_t oldest = (rtcLiveRatePos + (RTC_LIVE_RATE_TARGET_S + 1) - rtcLiveRateCount)
                        % (RTC_LIVE_RATE_TARGET_S + 1);
  const RtcLiveRatePoint &a = rtcLiveRateHist[oldest];
  const double elapsedS = (monoUs - a.monoUs) / 1000000.0;
  if (elapsedS < RTC_LIVE_RATE_MIN_S || elapsedS > RTC_LIVE_RATE_TARGET_S + 2.0) {
    rtcLiveRatePpm = NAN;
    rtcLiveRateSpanSec = 0;
    return;
  }
  const double dphaseMs = foldedPhaseDeltaMs(phaseMs, a.phaseMs);
  const double ppm = -(dphaseMs * 1000.0) / elapsedS;
  if (!isfinite(ppm) || fabs(ppm) > 20.0) {
    rtcLiveRatePpm = NAN;
    rtcLiveRateSpanSec = (uint16_t)lround(elapsedS);
    return;
  }
  rtcLiveRatePpm = ppm;
  rtcLiveRateSpanSec = (uint16_t)lround(elapsedS);
}
constexpr uint32_t RTC_CAL_MIN_WINDOW_MS = 10UL*60UL*1000UL;
constexpr uint32_t RTC_CAL_MAX_WINDOW_MS = 24UL*60UL*60UL*1000UL;
// Bei zweifelhafter RTC-Bestueckung auf false setzen:
constexpr bool RTC_AGING_AUTOCAL = true;

uint32_t rtcCalWindowForPpm(double ppm) {
  const double a=fabs(ppm);
  // Solange die RTC noch deutlich danebenliegt, schnell in kleinen, auf
  // maximal zwei Aging-LSB begrenzten Schritten einregeln. Erst nahe Null
  // werden lange Fenster benutzt, damit Messrauschen nicht nachgeregelt wird.
  if (a>=0.5)  return 10UL*60UL*1000UL;
  if (a>=0.1)  return 60UL*60UL*1000UL;
  if (a>=0.03) return 4UL*60UL*60UL*1000UL;
  return RTC_CAL_MAX_WINDOW_MS;
}

void rtcCalCountGpsFailure() {
  const uint16_t mask=gpsLockFailureMask();
  rtcCalLastGpsFailMask=mask;
  for (uint8_t i=0;i<GPS_FAIL_COUNT;i++)
    if (mask & (uint16_t)(1U<<i)) rtcCalGpsFailCount[i]++;
}

void rtcCalResetGps() {
  rtcCalCountGpsFailure();
  rtcCalReset(RTC_CAL_GPS);
}

void rtcCalReset(RtcCalResetReason reason) {
  if (rtcCalStartMs && reason != RTC_CAL_NONE) {
    rtcCalResetCount[reason]++;
    rtcCalLastReason = reason;
    Serial.printf("RTC Cal Fenster verworfen/beendet: %s nach %.2fh\n",
                  rtcCalReasonName(reason), (millis()-rtcCalStartMs)/3600000.0);
  }
  rtcCalStartMs = 0;
  rtcCalStartPhaseMs = NAN;
  rtcCalStartTempC = NAN;
  rtcCalTargetMs = RTC_CAL_MIN_WINDOW_MS;
  rtcCalTargetChosen = false;
  rtcCalProvisionalPpm = NAN;
}

void rtcCalBegin() {
  rtcCalStartMs = millis();
  rtcCalStartPhaseMs = rtcGpsPhaseMs;
  rtcCalStartTempC = NAN; // ESP-Chiptemperatur ist kein RTC-Temperaturmass.
  rtcCalTargetMs = RTC_CAL_MIN_WINDOW_MS;
  rtcCalTargetChosen = false;
  rtcCalProvisionalPpm = NAN;
}

void rtcCalTick() {
  uint32_t pseq, pperiod, sseq, speriod;
  int64_t pmono, smono;
  snapshotBothPulses(pseq, pperiod, pmono, sseq, speriod, smono);
  const bool lock = gpsLocked();
  if (!RTC_AGING_AUTOCAL || !rtcPresent || rtcOsf || !rtcAgingReadable) {
    if (rtcCalStartMs) rtcCalReset(RTC_CAL_RTC_STATE);
    return;
  }
  if (!lock) {
    if (rtcCalStartMs) rtcCalResetGps();
    return;
  }
  if (!isfinite(rtcGpsPhaseMs)) {
    if (rtcCalStartMs) rtcCalReset(RTC_CAL_PHASE);
    return;
  }
  if (pseq == rtcCalLastPpsSeq || sseq == rtcCalLastSqwSeq) return;
  rtcCalLastPpsSeq=pseq;
  rtcCalLastSqwSeq=sseq;
  if (speriod < 990000 || speriod > 1010000) {
    if (rtcCalStartMs) rtcCalReset(RTC_CAL_SQW);
    return;
  }
  if (!rtcCalStartMs) { rtcCalBegin(); return; }
  const double phaseDeltaMs = foldedPhaseDeltaMs(rtcGpsPhaseMs, rtcCalStartPhaseMs);
  if (fabs(phaseDeltaMs) > 250.0) {
    rtcCalRejected++;
    rtcCalReset(RTC_CAL_PHASE);
    return;
  }
  const uint32_t elapsed=millis()-rtcCalStartMs;
  if (elapsed>=RTC_CAL_MIN_WINDOW_MS)
    rtcCalProvisionalPpm=-(phaseDeltaMs*1000.0)/((double)elapsed/1000.0);

  // Nach den ersten 10 Minuten wird anhand des bereits messbaren Gangs genau
  // einmal die Laenge dieses Zyklus festgelegt: grosse Fehler schnell, kleine
  // Fehler lange beobachten. Dadurch springt das Ziel nicht durch Messrauschen.
  if (!rtcCalTargetChosen && elapsed>=RTC_CAL_MIN_WINDOW_MS) {
    if (!isfinite(rtcCalProvisionalPpm) || fabs(rtcCalProvisionalPpm)>15.0) {
      rtcCalRejected++;
      rtcCalReset(RTC_CAL_RATE);
      return;
    }
    rtcCalTargetMs=rtcCalWindowForPpm(rtcCalProvisionalPpm);
    rtcCalTargetChosen=true;
    Serial.printf("RTC Cal Ziel %.2fh bei vorlaeufig %+.4f ppm\n",
                  rtcCalTargetMs/3600000.0, rtcCalProvisionalPpm);
  }
  if (!rtcCalTargetChosen || elapsed<rtcCalTargetMs) return;

  // Spaetere SQW-Flanke => RTC laeuft zu langsam => negativer Aging-Offset.
  const double ppm=-(phaseDeltaMs*1000.0)/((double)elapsed/1000.0);
  rtcMeasuredRatePpm=ppm;
  if (!gpsLocked()) {
    rtcCalRejected++;
    rtcCalResetGps();
    return;
  }
  if (!isfinite(ppm) || fabs(ppm)>15.0) {
    rtcCalRejected++;
    rtcCalReset(RTC_CAL_RATE);
    return;
  }
  // Unter 0.1 ppm nur noch beobachten; darueber auf den naechsten typischen
  // 0.12-ppm-Aging-Schritt runden. Pro Zyklus weiterhin maximal zwei Schritte.
  if (fabs(ppm) >= 0.10) {
    int change=(int)lround(ppm/0.12); // DS3231M: positiv verlangsamt
    if (change>2) change=2;
    if (change< -2) change=-2;
    int next=(int)rtcAgingOffset+change;
    if (next>100) next=100;
    if (next< -100) next=-100;
    if (next != (int)rtcAgingOffset && rtcWriteReg(0x10,(uint8_t)(int8_t)next)) {
      uint8_t confirm;
      if (rtcReadReg(0x10,confirm) && (int8_t)confirm==(int8_t)next) {
        rtcAgingOffset=(int8_t)next;
        rtcAgingAdjustments++;
        rtcCalLastAdjustmentMs=millis();
        Serial.printf("RTC AGING %d, Gang %+.4f ppm, Fenster %.2fh\n",
                      next,ppm,elapsed/3600000.0);
        enqueueRtcEvent(1, bestTimestampEpoch(), next, ppm);
      } else rtcCalRejected++;
    }
  }
  rtcCalReset(RTC_CAL_FINISHED);
}

// ------------------------------------------------------------
// NTP Query
// ------------------------------------------------------------

// Der Client setzt vorhandene vertrauenswuerdige GPS-Zeit voraus.
// Offset: ((T2-T1)+(T3-T4))/2 ; Delay: (T4-T1)-(T3-T2).
// Delays mit 100en Millisekunden werden verworfen statt dem Nutzer
// einen scheinbar exakten Zeitfehler vorzutäuschen.
void ntpStamp(uint8_t *buf, int offset, uint64_t unixUs);

bool decodeNtpStamp(const uint8_t *p, uint64_t approximateUnixUs, uint64_t &unixUs) {
  constexpr uint64_t NTP_DELTA = 2208988800ULL;
  constexpr uint64_t NTP_ERA = 4294967296ULL;
  uint32_t low=0,frac=0;
  for (int k=0; k<4; ++k) {
    low=(low<<8)|p[k]; frac=(frac<<8)|p[k+4];
  }
  uint64_t reference=approximateUnixUs/1000000ULL+NTP_DELTA;
  uint64_t epoch=(reference/NTP_ERA)*NTP_ERA;
  uint64_t candidate=epoch+(uint64_t)low;
  if (candidate + NTP_ERA/2 < reference) candidate+=NTP_ERA;
  else if (candidate > reference + NTP_ERA/2 && candidate>=NTP_ERA)
    candidate-=NTP_ERA;
  if (candidate<NTP_DELTA+1577836800ULL) return false; // vor 2020
  uint64_t seconds=candidate-NTP_DELTA;
  // Gegen falsche Epochen / voellig unplausible Antworten schuetzen.
  const int64_t difference=(int64_t)seconds-(int64_t)(approximateUnixUs/1000000ULL);
  if (llabs(difference)>86400LL) return false;
  unixUs=seconds*1000000ULL+(((uint64_t)frac*1000000ULL)>>32);
  return true;
}

bool ntpQuery(const String &host, uint8_t idx) {
  if (idx > 1 || WiFi.status() != WL_CONNECTED || !ntpGpsReady()) return false;
  const int64_t queryStartUs = esp_timer_get_time();
  uint32_t sampleGeneration;
  portENTER_CRITICAL(&ntpDataMux);
  sampleGeneration=ntpResetGeneration;
  portEXIT_CRITICAL(&ntpDataMux);
  ntpLastAttemptMs[idx] = millis();
  IPAddress ip;
  const int64_t dnsStartUs = esp_timer_get_time();
  const bool resolved = WiFi.hostByName(host.c_str(), ip);
  const uint32_t dnsUs = (uint32_t)((esp_timer_get_time()>dnsStartUs) ? esp_timer_get_time()-dnsStartUs : 0);
  ntpDnsLastUs[idx] = dnsUs;
  if (dnsUs > ntpDnsMaxUs[idx]) ntpDnsMaxUs[idx] = dnsUs;
  if (!resolved) {
    ntpDnsFailures[idx]++;
    const int64_t endUs=esp_timer_get_time();
    const uint32_t total=(uint32_t)(endUs>queryStartUs ? endUs-queryStartUs : 0);
    if (total>ntpQueryMaxUs[idx]) ntpQueryMaxUs[idx]=total;
    return false;
  }

  while (udp.parsePacket() > 0) {
    uint8_t trash[48];
    while (udp.available()) udp.read(trash, min(48, udp.available()));
  }

  uint8_t q[48] = {};
  q[0] = 0x23; // LI=0, VN=4, client mode=3
  int64_t sendUs = esp_timer_get_time();
  uint64_t t1 = clockAtMonoUs(sendUs);
  if (!t1) { ntpClockFailures[idx]++; return false; }
  ntpStamp(q,40,t1);

  if (!udp.beginPacket(ip,123)) { ntpSendFailures[idx]++; return false; }
  udp.write(q,48);
  if (!udp.endPacket()) { ntpSendFailures[idx]++; return false; }
  uint32_t startMs = millis();
  while (millis() - startMs < NTP_TIMEOUT) {
    // NTP-Worker prueft nur die publizierte GPS-Freigabe.
    if (udp.parsePacket() > 0) {
      int64_t recvUs = esp_timer_get_time();
      int pLen = udp.available();
      if (pLen < 48 || udp.remoteIP() != ip) {
        ntpRejectedProtocol[idx]++;
        uint8_t trash[48];
        while (udp.available()) udp.read(trash,min(48,udp.available()));
        continue;
      }
      uint8_t r[48] = {};
      if (udp.read(r,48) != 48) { ntpRejectedProtocol[idx]++; continue; }
      uint8_t junk[48];
      while (udp.available()) udp.read(junk,min(48,udp.available()));
      if ((r[0] & 7) != 4 || (r[0] >> 6) == 3 ||
          r[1] == 0 || r[1] > 15 ||
          memcmp(r+24,q+40,8) != 0) {
        ntpRejectedProtocol[idx]++;
        continue;
      }
      uint64_t t2 = 0, t3 = 0;
      if (!decodeNtpStamp(r+32, t1, t2) ||
          !decodeNtpStamp(r+40, t1, t3)) {
        ntpRejectedProtocol[idx]++;
        return false;
      }
      if (t3<t2 || t3-t2>5000000ULL) { ntpRejectedProtocol[idx]++; return false; }
      int64_t elapsed = recvUs-sendUs;
      if (elapsed<=0 || elapsed>2000000LL) { ntpRejectedProtocol[idx]++; return false; }
      uint64_t t4 = clockAtMonoUs(recvUs);
      if (!t4 || !ntpGpsReady()) { ntpClockFailures[idx]++; return false; }
      int64_t netDelay=(int64_t)(t4-t1)-(int64_t)(t3-t2);
      uint32_t rawUs=(uint32_t)elapsed;
      uint32_t procUs=(uint32_t)min((uint64_t)5000000,t3-t2);
      uint32_t netUs=netDelay>=0 && netDelay<2000000 ?
                     (uint32_t)netDelay : rawUs;
      ntpLatestRawRttUs[idx]=rawUs;
      ntpLatestProcUs[idx]=procUs;
      ntpLatestNetRttUs[idx]=netUs;
      if (netDelay<0 || netUs>(idx==NTP_LOCAL ?
          LOCAL_NTP_MAX_RTT_US : POOL_NTP_MAX_RTT_US)) {
        ntpRejectedDelay[idx]++;
        return false;
      }
      // OFFSET = (T2-T1 + T3-T4)/2.  Nur gute Samples verwenden.
      int64_t offset=((int64_t)t2-(int64_t)t1 +
                      (int64_t)t3-(int64_t)t4)/2;
      if (llabs(offset)>2000000LL) { ntpRejectedProtocol[idx]++; return false; }
      uint32_t nowMs=millis();
      // Der Datensatz muss zum noch aktuellen Loesch-/Lern-Zyklus gehoeren.
      portENTER_CRITICAL(&ntpDataMux);
      if (sampleGeneration != ntpResetGeneration) {
        portEXIT_CRITICAL(&ntpDataMux);
        return false;
      }
      bool bestExpired = !ntpValid[idx] ||
          (uint32_t)(nowMs - ntpLastOkMs[idx]) >= NTP_MAX_SAMPLE_AGE_MS;
      bool replaceBest=bestExpired ||
          (uint32_t)(nowMs-ntpWindowStartMs[idx])>=NTP_BEST_WINDOW_MS ||
          netUs<=ntpRttUs[idx];
      ntpAccepted[idx]++;
      if (replaceBest) {
        ntpWindowStartMs[idx]=nowMs;
        ntpEpochUs[idx]=(uint64_t)((int64_t)t4+offset);
        ntpMonoUs[idx]=recvUs;
        ntpRawRttUs[idx]=rawUs;
        ntpServerProcUs[idx]=procUs;
        ntpRttUs[idx]=netUs;
        ntpLastOkMs[idx]=nowMs;
        ntpValid[idx]=true;
      }
      portEXIT_CRITICAL(&ntpDataMux);
      const int64_t endUs=esp_timer_get_time();
      const uint32_t total=(uint32_t)(endUs>queryStartUs ? endUs-queryStartUs : 0);      if (total>ntpQueryMaxUs[idx]) ntpQueryMaxUs[idx]=total;
      return true;
    }
    delay(1);
  }
  ntpTimeouts[idx]++;
  const int64_t endUs=esp_timer_get_time();
  const uint32_t total=(uint32_t)(endUs>queryStartUs ? endUs-queryStartUs : 0);
  if (total>ntpQueryMaxUs[idx]) ntpQueryMaxUs[idx]=total;
  return false;
}

// ------------------------------------------------------------
// Vergleiche
// ------------------------------------------------------------

double localGpsMs = NAN;
double poolGpsMs = NAN;
double localPoolMs = NAN;

void updateOffsets() {
  localPoolMs = NAN;
  if (ntpFresh(NTP_LOCAL) && ntpFresh(NTP_POOL)) {
    uint64_t l=ntpNow(NTP_LOCAL), p=ntpNow(NTP_POOL);
    if (l && p) localPoolMs = ((int64_t)l - (int64_t)p) / 1000.0;
  }

  uint32_t seq;
  int64_t pmono;

  uint32_t pperiod;
  snapshotPps(seq, pperiod, pmono);

  uint64_t gpsEpoch;
  if (!gpsLocked() || !gpsEpochForPps(seq, gpsEpoch)) {
    localGpsMs = poolGpsMs = NAN;
    return;
  }

  uint64_t gpsUs = gpsEpoch * 1000000ULL;

  if (ntpFresh(NTP_LOCAL)) {
    uint64_t n = ntpAt(NTP_LOCAL, pmono);
    if (n) localGpsMs = ((int64_t)n - (int64_t)gpsUs) / 1000.0;
  }

  if (ntpFresh(NTP_POOL)) {
    uint64_t n = ntpAt(NTP_POOL, pmono);
    if (n) poolGpsMs = ((int64_t)n - (int64_t)gpsUs) / 1000.0;
  } else poolGpsMs = NAN;
  if (!ntpFresh(NTP_LOCAL)) localGpsMs = NAN;
}


// ------------------------------------------------------------
// Temperaturmodell, PPS-Disziplinierung und Holdover-Uhr
// ------------------------------------------------------------
// Achtung: Chiptemperatur = Naeherung, keine OCXO-Innentemperatur.
// OCXO und ESP-Zeitbasis werden getrennt modelliert. Die OCXO-Impulse dienen bis zur validierten PCNT-Messung NUR zur
// Frequenzdiagnose. ESP-Timer und GNSS/PPS fuehren die Uhrzeit.
constexpr int BIN_COUNT = 32;
constexpr float BIN_START_C = 20.0f;
constexpr float BIN_STEP_C = 2.5f;
struct TempBin {
  uint32_t espN;
  float espAvgPpm;
  uint32_t ocxoN;
  float ocxoAvgPpm;
};
TempBin tempBins[BIN_COUNT] = {};
temperature_sensor_handle_t tempHandle = nullptr;
float chipTempC = NAN;
float ocxoTempProxyC = NAN;
bool tempReady = false;
float espMeasuredPpm = NAN;
float espEstimatedPpm = 0.0f;

uint32_t tempLearningCount = 0;
uint32_t lastTempSaveMs = 0;
uint32_t lastTempReadMs = 0;
uint32_t lastTempLearnMs = 0;
int64_t espModelRefMono = 0;
uint32_t espModelRefSeq = 0;
portMUX_TYPE tempIoMux = portMUX_INITIALIZER_UNLOCKED;
float tempIoRawC = NAN;
uint32_t tempIoSeq = 0;
uint32_t tempIoProcessedSeq = 0;

int tempIndex(float t) {
  if (!isfinite(t)) return -1;
  int idx = (int)floorf((t - BIN_START_C) / BIN_STEP_C);
  if (idx < 0 || idx >= BIN_COUNT) return -1;
  return idx;
}

float tempModelInterpolate(float t, bool forOcxo) {
  if (!isfinite(t)) return NAN;
  float pos = (t - BIN_START_C) / BIN_STEP_C - 0.5f;
  int a = -1, b = -1;
  for (int i=0; i<BIN_COUNT; ++i) {
    uint32_t n = forOcxo ? tempBins[i].ocxoN : tempBins[i].espN;
    if (n < 3) continue;
    if (i <= pos) a = i;
    if (i >= pos && b == -1) b = i;
  }
  if (a < 0 && b < 0) return NAN;
  if (a < 0) a = b;
  if (b < 0) b = a;
  float va = forOcxo ? tempBins[a].ocxoAvgPpm : tempBins[a].espAvgPpm;
  float vb = forOcxo ? tempBins[b].ocxoAvgPpm : tempBins[b].espAvgPpm;
  if (a == b) return va;
  float frac = (pos - a) / (b - a);
  return va + (vb - va) * frac;
}

void tempInit() {
  temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
  if (temperature_sensor_install(&cfg, &tempHandle) == ESP_OK &&
      temperature_sensor_enable(tempHandle) == ESP_OK) tempReady = true;
  Preferences p;
  if (p.begin("zktemp1", true)) {
    if (p.getBytesLength("bins") == sizeof(tempBins))
      p.getBytes("bins", tempBins, sizeof(tempBins));
    p.end();
  }
}

void tempSave(); // auf IO-Core, Implementierung nach MeasurementView

// Hardware-Zugriff bewusst auf Core0. Core1 verarbeitet nur den publizierten
// Skalar und beruehrt den Temperatursensor-Treiber nicht mehr.
void temperatureIoTick() {
  if ((uint32_t)(millis()-lastTempReadMs) < 10000U) return;
  lastTempReadMs=millis();
  float t;
  if (!tempReady || temperature_sensor_get_celsius(tempHandle,&t)!=ESP_OK) return;
  if (!isfinite(t) || t < -20.0f || t > 110.0f) return;
  portENTER_CRITICAL(&tempIoMux);
  tempIoRawC=t;
  tempIoSeq++;
  portEXIT_CRITICAL(&tempIoMux);
}

void tempTick() {
  float t; uint32_t tempSeq;
  portENTER_CRITICAL(&tempIoMux);
  t=tempIoRawC; tempSeq=tempIoSeq;
  portEXIT_CRITICAL(&tempIoMux);
  if (!tempSeq || tempSeq==tempIoProcessedSeq || !isfinite(t)) return;
  tempIoProcessedSeq=tempSeq;
  if (!isfinite(chipTempC)) chipTempC = t;
  else chipTempC = chipTempC * 0.8f + t * 0.2f;
  // Langsamer thermischer Proxy des Ofens (~250 s Zeitkonstante).
  if (!isfinite(ocxoTempProxyC)) ocxoTempProxyC = chipTempC;
  else ocxoTempProxyC = ocxoTempProxyC * 0.96f + chipTempC * 0.04f;
  int idx = tempIndex(chipTempC);
  int idxOcxo = tempIndex(ocxoTempProxyC);
  float ep = tempModelInterpolate(chipTempC, false);
  if (isfinite(ep)) espEstimatedPpm = ep;
  else if (isfinite(espMeasuredPpm)) espEstimatedPpm = espMeasuredPpm;
  float op = tempModelInterpolate(ocxoTempProxyC, true);
  if (isfinite(op)) ocxoEstimatedPpm = op;
  else if (isfinite(ocxoPpm300)) ocxoEstimatedPpm = (float)ocxoPpm300;
  if (!gpsLocked() || millis() - lastTempLearnMs < 60000) return;
  lastTempLearnMs = millis();
  if (!espModelRefMono) {
    uint32_t period;
    snapshotPps(espModelRefSeq, period, espModelRefMono);
    return;
  }
  uint32_t seq, period;
  int64_t mono;
  snapshotPps(seq, period, mono);
  int64_t dt = mono - espModelRefMono;
  uint32_t ds = seq - espModelRefSeq;
  espModelRefMono = mono;
  espModelRefSeq = seq;
  if (ds < 55 || ds > 65 || dt <= 0 || idx < 0) return;
  double ppm = (double)dt / (double)ds - 1000000.0;
  if (fabs(ppm) > 200.0) return;
  espMeasuredPpm = (float)ppm;
  TempBin &b = tempBins[idx];
  if (b.espN < 1000000UL) b.espN++;
  b.espAvgPpm += (float)((ppm - b.espAvgPpm) / (double)b.espN);
  espEstimatedPpm = b.espAvgPpm;
  // Nur laengeres zusammenhaengendes und plausibles OCXO-Messfenster lernen.
  if (idxOcxo >= 0 && validCount(ocHist,300) >= 294 && ocxoSignal &&
      isfinite(ocxoPpm300) &&
      fabs(ocxoPpm300) < 50.0 &&
      counterOCOK && counter32OK) {
    TempBin &ob = tempBins[idxOcxo];
    if (ob.ocxoN < 1000000UL) ob.ocxoN++;
    ob.ocxoAvgPpm += (float)((ocxoPpm300 - ob.ocxoAvgPpm) / (double)ob.ocxoN);
    ocxoEstimatedPpm = ob.ocxoAvgPpm;
  }
  tempLearningCount++;
  // tempSave() wird auf dem Netzwerk-/IO-Core ausgefuehrt.
}

portMUX_TYPE clockDataMux = portMUX_INITIALIZER_UNLOCKED;
bool clockAnchorValid = false;
uint32_t lastGpsLockMs = 0;
// Nur der Loop publiziert GPS-Zustand. Worker auf dem anderen Core liest
// diese atomare Kopie (niemals gpsLocked()/clockState()/NMEA-Felder).
bool ntpGpsLockPublished = false;
uint32_t ntpGpsPublishedMs = 0;
float publishedEspPpm = 0.0f;
void publishNtpTimeState() {
  const bool lock = gpsLocked() && clockAnchorValid &&
                    (uint32_t)(millis()-lastGpsLockMs) < 3000;
  portENTER_CRITICAL(&clockDataMux);
  ntpGpsLockPublished = lock;
  ntpGpsPublishedMs = millis();
  publishedEspPpm = espEstimatedPpm;
  portEXIT_CRITICAL(&clockDataMux);
}
bool ntpGpsReady() {
  bool lock;
  uint32_t published;
  portENTER_CRITICAL(&clockDataMux);
  lock=ntpGpsLockPublished; published=ntpGpsPublishedMs;
  portEXIT_CRITICAL(&clockDataMux);
  return lock && (uint32_t)(millis()-published) <= 500;
}
uint64_t clockEpochAtPpsUs = 0;
int64_t clockMonoAtPpsUs = 0;
double clockOcxoAtPpsTicks = 0;
uint32_t lastAnchoredSeq = 0;
bool clockEverLocked = false;
uint32_t lastStatusLockedMs = 0;
uint64_t rtcBootEpoch = 0;
int64_t rtcBootMonoUs = 0;
bool rtcBootValid = false;

void clockAnchorTick() {
  // Ein NMEA-Satz folgt oft erst 100-900 ms nach PPS: die PPS-Sekunde
  // erst nach gemappter RMC-Datums-/Uhrzeit verwenden.
  if (!gpsLocked() || !gpsEpochMapped ||
      mappedPpsSeq == lastAnchoredSeq || mappedPpsMono <= 0) return;
  const int64_t age = esp_timer_get_time() - mappedPpsMono;
  if (age < 0 || age > 1800000) return;
  portENTER_CRITICAL(&clockDataMux);
  clockEpochAtPpsUs = mappedPpsEpoch * 1000000ULL;
  clockMonoAtPpsUs = mappedPpsMono;
  clockOcxoAtPpsTicks = 0; // PCNT niemals als Wandzeit-Anker benutzen
  clockAnchorValid = true;
  portEXIT_CRITICAL(&clockDataMux);
  lastAnchoredSeq = mappedPpsSeq;
  lastGpsLockMs = millis();
  lastStatusLockedMs = millis();
  clockEverLocked = true;
}

const char *clockState() {
  if (gpsLocked() && clockAnchorValid && millis() - lastGpsLockMs < 3000) return "GPS_LOCK";
  if (clockAnchorValid) {
    return "HOLD_ESP"; // PCNT nicht verlaesslich genug als Zeitbasis
  }
  if (rtcBootValid && !rtcOsf) return "RTC_ONLY";
  return "TIME_INVALID";
}

float estimatedUncertaintyMs() {
  if (!clockAnchorValid) return rtcBootValid ? 2000.0f : NAN;
  if (gpsLocked()) return 10.0f; // Schaetzung, KEINE zertifizierte absolute Genauigkeit
  double seconds = (millis() - lastGpsLockMs) / 1000.0;
  double ppm = 10.0; // ESP-Timer; OCXO zaehlt hier bewusst nicht als Referenz
  return (float)(10.0 + seconds * ppm / 1000.0);
}

uint64_t clockAtMonoUs(int64_t mono) {
  // Atomarer GPS-Anker-Snapshot (Core 0 NTP, Core 1 GNSS).
  bool valid;
  uint64_t epoch;
  int64_t anchor;
  float ppmPublished;
  portENTER_CRITICAL(&clockDataMux);
  valid=clockAnchorValid;
  epoch=clockEpochAtPpsUs;
  anchor=clockMonoAtPpsUs;
  ppmPublished=publishedEspPpm;
  portEXIT_CRITICAL(&clockDataMux);
  if (valid) {
    const int64_t elapsed=mono-anchor;
    const double ppm=isfinite(ppmPublished) ? (double)ppmPublished : 0.0;
    const double corrected=(double)elapsed/(1.0+ppm*1e-6);
    if (corrected < -1000000.0) return 0;
    const int64_t result=(int64_t)epoch+(int64_t)llround(corrected);
    return result>0 ? (uint64_t)result : 0;
  }
  if (rtcBootValid && !rtcOsf) {
    const int64_t result=(int64_t)(rtcBootEpoch*1000000ULL)+mono-rtcBootMonoUs;
    return result>0 ? (uint64_t)result : 0;
  }
  return 0;
}

uint64_t clockNowUs() { return clockAtMonoUs(esp_timer_get_time()); }

void rtcSyncTick() {
  if (!rtcPresent || !gpsLocked() || !clockAnchorValid) return;
  // Die RTC-Sekundenregister nur nahe PPS schreiben, niemals direkt im ISR.
  uint32_t seq; int64_t pps;
  uint32_t period;
  snapshotPps(seq, period, pps);
  static uint32_t handled = 0;
  if (!seq || seq == handled) return;
  int64_t age = esp_timer_get_time() - pps;
  if (age < 0 || age > 15000) return; // naechsten PPS abwarten
  uint64_t epoch = 0;
  if (!gpsEpochForPps(seq, epoch)) return;
  handled = seq;
  uint64_t rtc = 0;
  bool goodRtc = rtcReadEpoch(rtc) && !rtcOsf;
  int64_t diffSeconds = goodRtc ? ((int64_t)rtc - (int64_t)epoch) : 999999;
  bool badPhase = isfinite(rtcGpsPhaseMs) && fabs(rtcGpsPhaseMs) > 250.0;
  bool mustSync = !goodRtc ||
      (rtcSyncCount == 0 && (diffSeconds != 0 || badPhase)) ||
      (rtcSyncCount > 0 && (llabs(diffSeconds) >= 2 || badPhase));
  if (!mustSync) return;
  // Nur selten nachstellen (Ausnahme: Datum/Uhrzeit ungueltig).
  if (rtcLastSyncEpoch && epoch - rtcLastSyncEpoch < 3600 && goodRtc &&
      llabs(diffSeconds) < 2) return;
  if (rtcLastFailedWriteMs && (uint32_t)(millis()-rtcLastFailedWriteMs)<60000UL)
    return; // 1-min-Backoff bei fehlgeschlagenem I2C-Schreiben
  rtcOffsetBeforeSyncMs = (double)diffSeconds * 1000.0 +
                          (isfinite(rtcGpsPhaseMs) ? rtcGpsPhaseMs : 0.0);
  if (rtcWriteEpoch(epoch)) {
    rtcLastFailedWriteMs = 0;
    rtcLastSyncEpoch = epoch;
    rtcSyncCount++;
    rtcBaselineMs = NAN;
    rtcLiveRateReset();
    rtcCalReset(RTC_CAL_SYNC);
    Serial.printf("RTC GPS SYNC %llu (vorher %.1f ms)\n",
                  (unsigned long long)epoch, rtcOffsetBeforeSyncMs);
    enqueueRtcEvent(2, epoch, 0, rtcOffsetBeforeSyncMs);
  } else {
    rtcLastFailedWriteMs = millis();
  }
}

// Cross-Core-Vertrag: nur skalare Messwerte unter kurzem Spinlock kopieren.
// Kein HTTP/FS/String-JSON auf Core1. RTC-Wire nur ueber den I2C-Mutex.
struct MeasurementView {
  decltype(::gpsFix) gpsFix;
  decltype(::gpsSats) gpsSats;
  decltype(::gpsVisible) gpsVisible;
  decltype(::gpsBestCno) gpsBestCno;
  decltype(::gpsHdop) gpsHdop;
  decltype(::gpsFixType) gpsFixType;
  decltype(::antennaStatus) antennaStatus;
  decltype(::nmeaOk) nmeaOk;
  decltype(::nmeaBad) nmeaBad;
  decltype(::gnssTimeAnomalies) gnssTimeAnomalies;
  decltype(::gnssLostEvents) gnssLostEvents;
  decltype(::counterReadFailures) counterReadFailures;
  decltype(::pcntDriverReadErrors) pcntDriverReadErrors;
  decltype(::pcntOcxoUnrecoverable) pcntOcxoUnrecoverable;
  decltype(::pcntRtc32Unrecoverable) pcntRtc32Unrecoverable;
  decltype(::pcntOcxoUnrecoverableRun) pcntOcxoUnrecoverableRun;
  decltype(::pcntRtc32UnrecoverableRun) pcntRtc32UnrecoverableRun;
  PcntRunFailureSnapshot firstRunPcntFailure;
  decltype(::counterInvalidWindows) counterInvalidWindows;
  decltype(::ocxoOverflowSuspects) ocxoOverflowSuspects;
  decltype(::flashGuardRejectedPps) flashGuardRejectedPps;
  decltype(::ocxoPpsRejects) ocxoPpsRejects;
  decltype(::lastBadOcxoPpm) lastBadOcxoPpm;
  decltype(::counterDiscontinuities) counterDiscontinuities;
  decltype(::ppsExtrapolationWarnings) ppsExtrapolationWarnings;
  decltype(::chipTempC) chipTempC;
  decltype(::ocxoTempProxyC) ocxoTempProxyC;
  decltype(::espEstimatedPpm) espEstimatedPpm;
  decltype(::ocxoEstimatedPpm) ocxoEstimatedPpm;
  decltype(::tempLearningCount) tempLearningCount;
  decltype(::rtcLastSyncEpoch) rtcLastSyncEpoch;
  decltype(::rtcSyncCount) rtcSyncCount;
  decltype(::ocxoSignal) ocxoSignal;
  decltype(::rtc32Signal) rtc32Signal;
  decltype(::rtcGpsPhaseMs) rtcGpsPhaseMs;
  decltype(::rtcGpsDriftMs) rtcGpsDriftMs;
  decltype(::rtcAgingOffset) rtcAgingOffset;
  decltype(::rtcMeasuredRatePpm) rtcMeasuredRatePpm;
  decltype(::rtcLiveRatePpm) rtcLiveRatePpm;
  decltype(::rtcLiveRateSpanSec) rtcLiveRateSpanSec;
  decltype(::rtcAgingAdjustments) rtcAgingAdjustments;
  decltype(::rtcCalStartMs) rtcCalStartMs;
  decltype(::rtcCalTargetMs) rtcCalTargetMs;
  decltype(::rtcCalProvisionalPpm) rtcCalProvisionalPpm;
  decltype(::rtcCalLastReason) rtcCalLastReason;
  decltype(::rtcCalLastGpsFailMask) rtcCalLastGpsFailMask;
  uint32_t rtcCalGpsFailCount[GPS_FAIL_COUNT];
  decltype(::gpsUnlockEvents) gpsUnlockEvents;
  decltype(::gpsUnlockSinceMs) gpsUnlockSinceMs;
  decltype(::gpsUnlockLastDurationMs) gpsUnlockLastDurationMs;
  decltype(::gpsUnlockMaxDurationMs) gpsUnlockMaxDurationMs;
  decltype(::gpsUnlockCurrentReasonMask) gpsUnlockCurrentReasonMask;
  decltype(::gpsUnlockLastReasonMask) gpsUnlockLastReasonMask;
  decltype(::rtcLocalMs) rtcLocalMs;
  decltype(::ocxo1) ocxo1;
  decltype(::ocxo10) ocxo10;
  decltype(::ocxo60) ocxo60;
  decltype(::ocxo100) ocxo100;
  decltype(::ocxo300) ocxo300;
  decltype(::ocxo1000) ocxo1000;
  decltype(::ocxoPpm10) ocxoPpm10;
  decltype(::ocxoPpm60) ocxoPpm60;
  decltype(::ocxoPpm100) ocxoPpm100;
  decltype(::ocxoPpm300) ocxoPpm300;
  decltype(::ocxoPpm1000) ocxoPpm1000;
  decltype(::rtc321) rtc321;
  decltype(::rtc3210) rtc3210;
  decltype(::rtc3260) rtc3260;
  decltype(::rtc32Ppm60) rtc32Ppm60;
  decltype(::rtc32100) rtc32100;
  decltype(::rtc32300) rtc32300;
  decltype(::rtc321000) rtc321000;
  decltype(::rtc32Ppm1000) rtc32Ppm1000;
  decltype(::histCount) histCount;
  decltype(::badOcxoPps) badOcxoPps;
  decltype(::badRtc32Pps) badRtc32Pps;
  decltype(::badGpsPps) badGpsPps;
  decltype(::localGpsMs) localGpsMs;
  decltype(::poolGpsMs) poolGpsMs;
  decltype(::localPoolMs) localPoolMs;
  decltype(::runPhase) runPhase;
  decltype(::measurementStartedMs) measurementStartedMs;
  decltype(::rejectedGpsReference) rejectedGpsReference;
  decltype(::lastGpsLockMs) lastGpsLockMs;
  uint32_t rtcCalResetCount[RTC_CAL_REASON_COUNT];
  uint32_t badWindowsByPhase[3], rejectedOcxoByPhase[3], overflowSuspectsByPhase[3];
  uint16_t gpsGpVisible, gpsGpSignal, gpsBdVisible, gpsBdSignal, gpsGnVisible;
  uint8_t gpsGpCno, gpsBdCno;
  int64_t ppsMono;
  uint32_t ppsSeq, ppsPeriod, sqwPeriod;
  uint64_t gpsEpoch, rtcEpoch, clockRefEpochUs;
  bool haveGps, haveRtc, locked, anchorValid;
  float uncertaintyMs;
  char clockMode[16];
  uint16_t validO100, validO300, validO1000, validR100;
  TempBin bins[BIN_COUNT];
  uint32_t sampleMs;
};
portMUX_TYPE measurementViewMux = portMUX_INITIALIZER_UNLOCKED;
MeasurementView publishedMeasurement = {};
uint32_t measurementPublishedMs = 0;
void readMeasurement(MeasurementView &out) {
  portENTER_CRITICAL(&measurementViewMux);
  out=publishedMeasurement;
  portEXIT_CRITICAL(&measurementViewMux);
}
void publishMeasurement() {
  static uint32_t last=0;
  if ((uint32_t)(millis()-last)<100) return;
  last=millis();
  MeasurementView v = {};
  v.gpsFix = gpsFix;
  v.gpsSats = gpsSats;
  v.gpsVisible = gpsVisible;
  v.gpsBestCno = gpsBestCno;
  v.gpsHdop = gpsHdop;
  v.gpsFixType = gpsFixType;
  v.antennaStatus = antennaStatus;
  v.nmeaOk = nmeaOk;
  v.nmeaBad = nmeaBad;
  v.gnssTimeAnomalies = gnssTimeAnomalies;
  v.gnssLostEvents = gnssLostEvents;
  v.counterReadFailures = counterReadFailures;
  v.pcntDriverReadErrors = pcntDriverReadErrors;
  v.pcntOcxoUnrecoverable = pcntOcxoUnrecoverable;
  v.pcntRtc32Unrecoverable = pcntRtc32Unrecoverable;
  v.pcntOcxoUnrecoverableRun = pcntOcxoUnrecoverableRun;
  v.pcntRtc32UnrecoverableRun = pcntRtc32UnrecoverableRun;
  v.firstRunPcntFailure = firstRunPcntFailure;
  v.counterInvalidWindows = counterInvalidWindows;
  v.ocxoOverflowSuspects = ocxoOverflowSuspects;
  v.flashGuardRejectedPps = flashGuardRejectedPps;
  v.ocxoPpsRejects = ocxoPpsRejects;
  v.lastBadOcxoPpm = lastBadOcxoPpm;
  v.counterDiscontinuities = counterDiscontinuities;
  v.ppsExtrapolationWarnings = ppsExtrapolationWarnings;
  v.chipTempC = chipTempC;
  v.ocxoTempProxyC = ocxoTempProxyC;
  v.espEstimatedPpm = espEstimatedPpm;
  v.ocxoEstimatedPpm = ocxoEstimatedPpm;
  v.tempLearningCount = tempLearningCount;
  v.rtcLastSyncEpoch = rtcLastSyncEpoch;
  v.rtcSyncCount = rtcSyncCount;
  v.ocxoSignal = ocxoSignal;
  v.rtc32Signal = rtc32Signal;
  v.rtcGpsPhaseMs = rtcGpsPhaseMs;
  v.rtcGpsDriftMs = rtcGpsDriftMs;
  v.rtcAgingOffset = rtcAgingOffset;
  v.rtcMeasuredRatePpm = rtcMeasuredRatePpm;
  v.rtcLiveRatePpm = rtcLiveRatePpm;
  v.rtcLiveRateSpanSec = rtcLiveRateSpanSec;
  v.rtcAgingAdjustments = rtcAgingAdjustments;
  v.rtcCalStartMs = rtcCalStartMs;
  v.rtcCalTargetMs = rtcCalTargetMs;
  v.rtcCalProvisionalPpm = rtcCalProvisionalPpm;
  v.rtcCalLastReason = rtcCalLastReason;
  v.rtcCalLastGpsFailMask = rtcCalLastGpsFailMask;
  memcpy(v.rtcCalGpsFailCount,rtcCalGpsFailCount,sizeof(rtcCalGpsFailCount));
  v.gpsUnlockEvents = gpsUnlockEvents;
  v.gpsUnlockSinceMs = gpsUnlockSinceMs;
  v.gpsUnlockLastDurationMs = gpsUnlockLastDurationMs;
  v.gpsUnlockMaxDurationMs = gpsUnlockMaxDurationMs;
  v.gpsUnlockCurrentReasonMask = gpsUnlockCurrentReasonMask;
  v.gpsUnlockLastReasonMask = gpsUnlockLastReasonMask;
  v.rtcLocalMs = rtcLocalMs;
  v.ocxo1 = ocxo1;
  v.ocxo10 = ocxo10;
  v.ocxo60 = ocxo60;
  v.ocxo100 = ocxo100;
  v.ocxo300 = ocxo300;
  v.ocxo1000 = ocxo1000;
  v.ocxoPpm10 = ocxoPpm10;
  v.ocxoPpm60 = ocxoPpm60;
  v.ocxoPpm100 = ocxoPpm100;
  v.ocxoPpm300 = ocxoPpm300;
  v.ocxoPpm1000 = ocxoPpm1000;
  v.rtc321 = rtc321;
  v.rtc3210 = rtc3210;
  v.rtc3260 = rtc3260;
  v.rtc32Ppm60 = rtc32Ppm60;
  v.rtc32100 = rtc32100;
  v.rtc32300 = rtc32300;
  v.rtc321000 = rtc321000;
  v.rtc32Ppm1000 = rtc32Ppm1000;
  v.histCount = histCount;
  v.badOcxoPps = badOcxoPps;
  v.badRtc32Pps = badRtc32Pps;
  v.badGpsPps = badGpsPps;
  v.localGpsMs = localGpsMs;
  v.poolGpsMs = poolGpsMs;
  v.localPoolMs = localPoolMs;
  v.runPhase = runPhase;
  v.measurementStartedMs = measurementStartedMs;
  v.rejectedGpsReference = rejectedGpsReference;
  v.lastGpsLockMs = lastGpsLockMs;
  memcpy(v.rtcCalResetCount, rtcCalResetCount, sizeof(rtcCalResetCount));
  memcpy(v.badWindowsByPhase, badWindowsByPhase, sizeof(badWindowsByPhase));
  memcpy(v.rejectedOcxoByPhase, rejectedOcxoByPhase, sizeof(rejectedOcxoByPhase));
  memcpy(v.overflowSuspectsByPhase, overflowSuspectsByPhase, sizeof(overflowSuspectsByPhase));
  memcpy(v.bins, tempBins, sizeof(tempBins));
  v.gpsGpVisible=gnssVisibleNow(GNSS_GP);
  v.gpsGpSignal=gnssSignalNow(GNSS_GP);
  v.gpsGpCno=gnssCnoNow(GNSS_GP);
  const bool bd=gnssVisibleNow(GNSS_BD)>0;
  v.gpsBdVisible=gnssVisibleNow(bd?GNSS_BD:GNSS_GB);
  v.gpsBdSignal=gnssSignalNow(bd?GNSS_BD:GNSS_GB);
  v.gpsBdCno=gnssCnoNow(bd?GNSS_BD:GNSS_GB);
  v.gpsGnVisible=gnssVisibleNow(GNSS_GN);
  uint32_t sqwseq, sqwperiod;
  int64_t sqwmono;
  snapshotBothPulses(v.ppsSeq, v.ppsPeriod, v.ppsMono, sqwseq, sqwperiod, sqwmono);
  v.sqwPeriod=sqwperiod;
  v.haveGps=gpsRefValid() && gpsEpochForPps(v.ppsSeq,v.gpsEpoch);
  v.haveRtc=lastRtcReadValid && (uint32_t)(millis()-lastRtcReadMs) < 3000U;
  v.rtcEpoch=lastRtcReadEpoch;
  v.locked=gpsLocked();
  v.anchorValid=clockAnchorValid;
  v.uncertaintyMs=estimatedUncertaintyMs();
  v.clockRefEpochUs=clockEpochAtPpsUs;
  snprintf(v.clockMode,sizeof(v.clockMode),"%s",clockState());
  v.validO100=validCount(ocHist,100);
  v.validO300=validCount(ocHist,300);
  v.validO1000=validCount(ocHist,1000);
  v.validR100=validCount(r32Hist,100);
  v.sampleMs=millis();
  portENTER_CRITICAL(&measurementViewMux);
  publishedMeasurement=v;
  measurementPublishedMs=v.sampleMs;
  portEXIT_CRITICAL(&measurementViewMux);
}

void tempSave() {
  markFlashActivity();
  MeasurementView v;
  readMeasurement(v);
  Preferences p;
  if (p.begin("zktemp1", false)) {
    p.putBytes("bins", v.bins, sizeof(v.bins));
    p.end();
  }
  lastTempSaveMs = millis();
  markFlashActivity();
}


// ------------------------------------------------------------
// NTP-Server auf UDP/123 (lokaler NTP-Client nutzt separaten UDP-Socket)
// ------------------------------------------------------------
WiFiUDP ntpServer;
bool ntpServerStarted = false;
uint32_t ntpServed = 0;
uint32_t ntpRejected = 0;

void ntpStamp(uint8_t *buf, int offset, uint64_t unixUs) {
  const uint64_t NTP_DELTA = 2208988800ULL;
  uint32_t sec = (uint32_t)(unixUs / 1000000ULL + NTP_DELTA);
  uint32_t frac = (uint32_t)(((unixUs % 1000000ULL) << 32) / 1000000ULL);
  for (int i=0; i<4; ++i) {
    buf[offset+i] = (uint8_t)(sec >> (24 - i*8));
    buf[offset+4+i] = (uint8_t)(frac >> (24 - i*8));
  }
}

void ntpServerTick() {
  if (!ntpServerStarted) return;
  int n = ntpServer.parsePacket();
  if (n < 48) {
    if (n) { while (ntpServer.available()) ntpServer.read(); ntpRejected++; }
    return;
  }
  uint8_t q[48] = {};
  if (ntpServer.read(q, 48) != 48) return;
  while (ntpServer.available()) ntpServer.read();
  if ((q[0]&7) != 3 || (q[0]>>3&7) < 3) { ntpRejected++; return; }
  uint64_t tReceive = clockNowUs();
  MeasurementView v;
  readMeasurement(v);
  const char *state=v.clockMode;
  const bool snapshotFresh=v.sampleMs && (uint32_t)(millis()-v.sampleMs)<1000U;
  bool locked = snapshotFresh && ntpGpsReady() && strcmp(state,"GPS_LOCK") == 0;
  bool hold = snapshotFresh && strcmp(state,"HOLD_ESP") == 0;
  bool credible = locked || (hold && v.uncertaintyMs < 1000.0f &&
                             millis() - v.lastGpsLockMs < 86400000UL);
  uint8_t reply[48] = {};
  reply[0] = (credible ? 0x00 : 0xC0) | (q[0] & 0x38) | 4;
  // RMC/PPS-Sekundenzuordnung des ATGM336H ist noch nicht mit
  // unabhaengiger Hardware verifiziert. Kein irrefuehrendes Stratum 1.
  reply[1] = locked ? GPS_NTP_STRATUM : (credible ? GPS_NTP_STRATUM+1 : 16);
  reply[2] = q[2];
  reply[3] = (uint8_t)-20;
  // Root dispersion als 16.16 fixed point Sekunden.
  double dispersion = credible ? max(0.002, (double)v.uncertaintyMs/1000.0) : 16.0;
  uint32_t disp = (uint32_t)min(4294967295.0, dispersion * 65536.0);
  reply[8] = disp >> 24; reply[9] = disp >> 16;
  reply[10] = disp >> 8; reply[11] = disp;
  memcpy(reply+12, locked ? "GNSS" : (credible ? "HOLD" : "INIT"),4);
  // Letzter vertrauenswuerdiger PPS als NTP-Reference-Timestamp.
  if (v.anchorValid) ntpStamp(reply,16,v.clockRefEpochUs);
  memcpy(reply+24, q+40,8); // Originate = client transmit
  if (tReceive) ntpStamp(reply,32,tReceive);
  uint64_t tx = clockNowUs();
  if (tx) ntpStamp(reply,40,tx);
  ntpServer.beginPacket(ntpServer.remoteIP(),ntpServer.remotePort());
  ntpServer.write(reply,48);
  ntpServer.endPacket();
  ntpServed++;
}

// ------------------------------------------------------------
// Standalone Langzeit-Logger / interne Flash-Speicherung
// ------------------------------------------------------------

// Kein PC/Server noetig:
// - 1-s-Messwerte bleiben nur im RAM.
// - Alle 10 Minuten wird ein kompakter Langzeitpunkt in LittleFS geschrieben.
// - Persistente Statistik wird nur stuendlich aktualisiert.
// Das schont den internen Flash deutlich gegenueber 1-Hz-Logging.

constexpr uint32_t LOG_INTERVAL_MS = 10UL * 60UL * 1000UL;
constexpr uint32_t STATS_SAVE_EVERY_LOGS = 6; // 1 Stunde
// Die uebliche S3-Zero-Partition hat nur ~1.5MB FS, nicht 2MB!
constexpr size_t LOG_ROTATE_BYTES = 400UL * 1024UL;
constexpr size_t EVENT_ROTATE_BYTES = 64UL * 1024UL;
uint32_t logRows = 0;
uint32_t lastLogWriteMs = 0;
uint32_t logWriteFailures = 0;
uint32_t logBytesWritten = 0;
uint32_t fsFormatCode = 0; // steht NUR auf LCD und Serial, nicht in Webseite
bool fsReinitSucceeded = false;
uint32_t resetToken = 0; // CSRF-Schutz, nur als HTML-Formwert

// Friert genau den ersten PCNT-Fehler im echten Messbetrieb ein. Dadurch kann
// spaeter geprueft werden, ob Rohzaehler, Flash/Logger oder RTC-Kalibrierzyklus
// zeitlich mit dem Ereignis zusammenfallen. Keine Messlogik wird veraendert.
void captureFirstRunPcntFailure(PcntFailureKind kind, uint32_t ppsSeq,
                                const PpsCounterRead &r,
                                uint32_t intervalPrevRaw,
                                uint32_t intervalCurrRaw,
                                uint32_t intervalSpan,
                                double intervalPerSecond) {
  if (firstRunPcntFailure.valid || warmupActive() || !gpsLocked()) return;
  const uint32_t now=millis();
  PcntRunFailureSnapshot snap;
  snap.valid=true;
  snap.kind=kind;
  snap.uptimeMs=now;
  snap.ppsSeq=ppsSeq;
  snap.ocOk=r.ocOk; snap.r32Ok=r.r32Ok;
  snap.ocTrace=r.ocTrace; snap.r32Trace=r.r32Trace;
  snap.intervalPrevRaw=intervalPrevRaw;
  snap.intervalCurrRaw=intervalCurrRaw;
  snap.intervalSpan=intervalSpan;
  snap.intervalPerSecond=intervalPerSecond;
  snap.flashAgeMs=lastFlashActivityMs ? (uint32_t)(now-lastFlashActivityMs) : UINT32_MAX;
  snap.logAgeMs=lastLogWriteMs ? (uint32_t)(now-lastLogWriteMs) : UINT32_MAX;
  snap.rtcAging=rtcAgingOffset;
  snap.rtcCalElapsedMs=rtcCalStartMs ? (uint32_t)(now-rtcCalStartMs) : 0U;
  snap.rtcCalTargetMs=rtcCalTargetMs;
  snap.rtcCalLastReason=(uint8_t)rtcCalLastReason;
  firstRunPcntFailure=snap;
}

uint32_t lastLogMs = 0;
uint32_t logsSinceStatsSave = 0;
uint32_t persistentCount = 0;
double persistentPpmSum = 0.0;
double persistentPpmMin = NAN;
double persistentPpmMax = NAN;
double persistentPpmMean = NAN;

String csvSafeNum(double v, int decimals) {
  if (isnan(v) || isinf(v)) return "";
  return String(v, decimals);
}

uint64_t bestTimestampEpoch() {
  // NTP ist nur Vergleich, keine heimliche Zeitreferenz.
  return clockNowUs() / 1000000ULL;
}

void ensureLogHeader() {
  if (LittleFS.exists("/ocxo.csv")) return;

  markFlashActivity();
  File f = LittleFS.open("/ocxo.csv", "w");
  if (!f) return;

  f.println("epoch,state,temp_c,ocxo_hz_1000,ocxo_ppm_1000,rtc32_hz_1000,rtc32_ppm_1000,rtc_gps_phase_ms,rtc_gps_drift_ms,gps_sats,gps_visible,gps_cno,ntp_local_gps_ms,ntp_pool_gps_ms,nmea_bad,pps_loss,rtc_sync_count,uncertainty_ms");
  f.close();
  markFlashActivity();
}

void rotateLogIfNeeded() {
  File f = LittleFS.open("/ocxo.csv", "r");
  if (!f) return;

  size_t sz = f.size();
  f.close();

  if (sz < LOG_ROTATE_BYTES) return;
  markFlashActivity();

  if (LittleFS.exists("/ocxo_old.csv")) {
    LittleFS.remove("/ocxo_old.csv");
  }

  if (LittleFS.rename("/ocxo.csv", "/ocxo_old.csv")) {
    logRows=0; // neue aktuelle CSV, altes Log liegt in ocxo_old.csv
    ensureLogHeader();
  }
  markFlashActivity();
}

void rotateEventsIfNeeded() {
  if (!fsReady || !LittleFS.exists("/events.csv")) return;
  File f=LittleFS.open("/events.csv", "r");
  if (!f) return;
  size_t size=f.size(); f.close();
  if (size<EVENT_ROTATE_BYTES) return;
  markFlashActivity();
  if (LittleFS.exists("/events_old.csv")) LittleFS.remove("/events_old.csv");
  LittleFS.rename("/events.csv", "/events_old.csv");
  markFlashActivity();
}

bool readStatsFile(const char *path, uint32_t &count, double &sum,
                   double &low, double &high) {
  File f=LittleFS.open(path,"r");
  if (!f) return false;
  String line=f.readStringUntil('\n');
  f.close();
  line.trim();
  if (line.length()<7 || line.length()>120) return false;
  int p1=line.indexOf(','), p2=line.indexOf(',',p1+1), p3=line.indexOf(',',p2+1);
  if (p1<=0 || p2<=p1+1 || p3<=p2+1 || line.indexOf(',',p3+1)>=0) return false;
  String c=line.substring(0,p1), a=line.substring(p1+1,p2),
         b=line.substring(p2+1,p3), d=line.substring(p3+1);
  auto validNumber=[](const String &v, double &out)->bool {
    const char *str=v.c_str(); char *end=nullptr;
    out=strtod(str,&end);
    return end!=str && end && *end=='\0' && isfinite(out);
  };
  const char *cs=c.c_str(); char *end=nullptr;
  unsigned long readCount=strtoul(cs,&end,10);
  if (cs==end || !end || *end!='\0' || readCount==0) return false;
  double readSum, readMin, readMax;
  if (!validNumber(a,readSum) || !validNumber(b,readMin) ||
      !validNumber(d,readMax) || readMin>readMax) return false;
  count=(uint32_t)readCount; sum=readSum; low=readMin; high=readMax;
  return true;
}

void loadPersistentStats() {
  persistentCount=0;
  persistentPpmSum=0.0;
  persistentPpmMin=persistentPpmMax=persistentPpmMean=NAN;
  // Beim unterbrochenen Umbenennen bleibt die letzte gueltige .bak erhalten.
  uint32_t count; double sum,low,high;
  if (!readStatsFile("/stats.txt",count,sum,low,high) &&
      !readStatsFile("/stats.bak",count,sum,low,high)) return;
  persistentCount=count;
  persistentPpmSum=sum;
  persistentPpmMin=low;
  persistentPpmMax=high;
  persistentPpmMean=sum/(double)count;
}

void savePersistentStats() {
  if (!fsReady || !persistentCount || !isfinite(persistentPpmSum) ||
      !isfinite(persistentPpmMin) || !isfinite(persistentPpmMax)) return;
  markFlashActivity();
  File f=LittleFS.open("/stats.tmp","w");
  if (!f) { logWriteFailures++; return; }
  int n=f.printf("%lu,%.12f,%.9f,%.9f\n",
                 (unsigned long)persistentCount,persistentPpmSum,
                 persistentPpmMin,persistentPpmMax);
  f.flush(); f.close();
  if (n<=0) { logWriteFailures++; return; }
  if (LittleFS.exists("/stats.bak")) LittleFS.remove("/stats.bak");
  if (LittleFS.exists("/stats.txt") && !LittleFS.rename("/stats.txt","/stats.bak")) {
    logWriteFailures++; return;
  }
  if (!LittleFS.rename("/stats.tmp","/stats.txt")) {
    logWriteFailures++;
    // Restore prior stats on a recoverable rename failure.
    if (LittleFS.exists("/stats.bak")) LittleFS.rename("/stats.bak","/stats.txt");
  }
  markFlashActivity();
}

void updatePersistentStats(double ppm) {
  if (isnan(ppm) || isinf(ppm)) return;

  persistentCount++;
  persistentPpmSum += ppm;

  if (isnan(persistentPpmMin) || ppm < persistentPpmMin) persistentPpmMin = ppm;
  if (isnan(persistentPpmMax) || ppm > persistentPpmMax) persistentPpmMax = ppm;

  persistentPpmMean = persistentPpmSum / persistentCount;
}

void appendLongTermLog() {
  markFlashActivity();
  rotateLogIfNeeded();
  ensureLogHeader();

  File f = LittleFS.open("/ocxo.csv", "a");
  if (!f) { logWriteFailures++; return; }

  MeasurementView v;
  readMeasurement(v);
  uint64_t epoch = bestTimestampEpoch();

  String line;
  line.reserve(220);

  line += String((unsigned long long)epoch);
  line += "," + String(v.clockMode);
  line += "," + csvSafeNum(v.chipTempC, 2);
  line += "," + csvSafeNum(v.ocxo1000, 6);
  line += "," + csvSafeNum(v.ocxoPpm1000, 9);
  line += "," + csvSafeNum(v.rtc321000, 6);
  line += "," + csvSafeNum(v.rtc32Ppm1000, 6);
  line += "," + csvSafeNum(v.rtcGpsPhaseMs, 3);
  line += "," + csvSafeNum(v.rtcGpsDriftMs, 3);
  line += "," + String(v.gpsSats);
  line += "," + String(v.gpsVisible);
  line += "," + String(v.gpsBestCno);
  line += "," + csvSafeNum(v.localGpsMs, 3);
  line += "," + csvSafeNum(v.poolGpsMs, 3);
  line += "," + String(v.nmeaBad);
  line += "," + String(v.gnssLostEvents);
  line += "," + String(v.rtcSyncCount);
  line += "," + csvSafeNum(v.uncertaintyMs, 2);

  size_t written = f.println(line);
  f.flush();
  f.close();
  markFlashActivity();
  if (written < line.length() + 1) { logWriteFailures++; return; }
  logRows++;
  logBytesWritten += (uint32_t)written;
  lastLogWriteMs = millis();

  if (isfinite(v.ocxoPpm1000)) updatePersistentStats(v.ocxoPpm1000);

  logsSinceStatsSave++;
  if (logsSinceStatsSave >= STATS_SAVE_EVERY_LOGS) {
    logsSinceStatsSave = 0;
    savePersistentStats();
  }
}

void loadLogRows() {
  logRows=0;
  File f=LittleFS.open("/ocxo.csv","r");
  if (!f) return;
  bool first=true;
  while (f.available()) {
    String line=f.readStringUntil('\n');
    if (first) first=false;
    else if (line.length()>2) logRows++;
  }
  f.close();
}

bool deleteRecordedData() {
  markFlashActivity();
  if (!fsReady) return false;
  const char *paths[] = {"/ocxo.csv","/ocxo_old.csv","/stats.txt","/stats.tmp","/stats.bak","/events.csv","/events_old.csv"};
  for (const char *path : paths) {
    if (LittleFS.exists(path) && !LittleFS.remove(path)) return false;
  }
  ensureLogHeader();
  if (!LittleFS.exists("/ocxo.csv")) return false;
  logRows=0;
  logBytesWritten=0;
  lastLogWriteMs=0;
  logWriteFailures=0;
  logsSinceStatsSave=0;
  // kein sofortiger Re-Log der alten Daten
  lastLogMs=millis();
  persistentCount=0;
  persistentPpmSum=0;
  persistentPpmMean=NAN;
  persistentPpmMin=NAN;
  persistentPpmMax=NAN;
  return true;
}

void loggerTick() {
  if (!fsReady) return;
  MeasurementView v; readMeasurement(v);
  if (!v.sampleMs) return;

  if (lastLogMs == 0 || millis() - lastLogMs >= LOG_INTERVAL_MS) {
    lastLogMs = millis();
    appendLongTermLog();
  }
}

size_t logFileSize() {
  if (!fsReady) return 0;
  File f = LittleFS.open("/ocxo.csv", "r");
  if (!f) return 0;
  size_t s = f.size();
  f.close();
  return s;
}

// ------------------------------------------------------------
// Web-Verlauf (RAM, 1 Stunde bei 10-s-Abstand)
// ------------------------------------------------------------
struct TrendPoint {
  uint32_t uptime;
  float ocxoPpm300;
  uint8_t cno;
};
constexpr int TREND_MAX=360;
TrendPoint trendHistory[TREND_MAX] = {};
int trendCount=0, trendPos=0;
portMUX_TYPE trendDataMux = portMUX_INITIALIZER_UNLOCKED;


void trendTick() {
  static uint32_t last=0;
  if (millis()-last<10000) return;
  last=millis();
  TrendPoint p = {millis()/1000,
                  isfinite(ocxoPpm300)?(float)ocxoPpm300:NAN, gpsBestCno};
  portENTER_CRITICAL(&trendDataMux);
  trendHistory[trendPos]=p;
  trendPos=(trendPos+1)%TREND_MAX;
  if (trendCount<TREND_MAX) trendCount++;
  portEXIT_CRITICAL(&trendDataMux);
}
volatile uint32_t clearMeasureRequest = 0;
volatile uint32_t clearMeasureAck = 0;
void resetMeasurementRam() {
  // Nur RAM-Messhistorien und Auswertungen, nicht Zeitanker/RTC/GPS-Signal.
  resetFreqWindows();
  resetAdev();
  havePrevOcSnap=havePrevR32Snap=false;
  prevOcRaw=prevR32Raw=0;
  prevOcDelayS=prevR32DelayS=0.0;
  prevOcSeq=prevR32Seq=0;
  processedPps=0; prevPpsMono=0;
  lastCalibratedOcxoHz=NAN;
  ocxoEstimatedPpm=0;
  ocxoCalibrated=false;
  portENTER_CRITICAL(&trendDataMux);
  trendCount=0; trendPos=0;
  memset(trendHistory,0,sizeof(trendHistory));
  portEXIT_CRITICAL(&trendDataMux);  rtcBaselineMs=NAN; rtcGpsDriftMs=NAN;
  rtcLiveRateReset();
  rtcCalReset(RTC_CAL_CLEAR);
  memset(rtcCalGpsFailCount,0,sizeof(rtcCalGpsFailCount));
  rtcCalLastGpsFailMask=0;
  gpsUnlockEvents=0; gpsUnlockSinceMs=0;
  gpsUnlockLastDurationMs=0; gpsUnlockMaxDurationMs=0;
  gpsUnlockCurrentReasonMask=0; gpsUnlockLastReasonMask=0;
  gpsLockDiagSeenLock=false; gpsLockDiagPrevLocked=false;
  rtcGpsPhaseMs=NAN; rtcLocalMs=NAN;
  localGpsMs=NAN; poolGpsMs=NAN; localPoolMs=NAN;
  badOcxoPps=badRtc32Pps=badGpsPps=0;
  gnssLostEvents=ocxoPpsRejects=ocxoOverflowSuspects=0;
  counterReadFailures=counterInvalidWindows=counterDiscontinuities=pcntDriverReadErrors=0;
  pcntOcxoUnrecoverable=pcntRtc32Unrecoverable=0;
  pcntOcxoUnrecoverableRun=pcntRtc32UnrecoverableRun=0;
  firstRunPcntFailure=PcntRunFailureSnapshot{};
  pcntReadAttempts=pcntRetryReads=pcntRecoveredReads=pcntRecoveredSpans=0;
  flashGuardRejectedPps=0;
  ppsExtrapolationWarnings=0; ocxoSignalChanges=0;
  lastBadCounterMs=0;lastBadOcxoPpm=NAN;lastBadOcxoDelta=0;
  nmeaOk=nmeaBad=gnssTimeAnomalies=0;
  for (int i=0;i<3;i++) {
    badWindowsByPhase[i]=rejectedOcxoByPhase[i]=overflowSuspectsByPhase[i]=0;
  }
  rejectedGpsReference=0;
  memset(tempBins,0,sizeof(tempBins));
  espMeasuredPpm=NAN;
  ocxoEstimatedPpm=0.0f;
  ocxoCalibrated=false;
  lastCalibratedOcxoHz=NAN;
  espModelRefMono=0; espModelRefSeq=0;
  tempLearningCount=0;
  lastTempLearnMs=millis();
}
bool clearAllMeasurements() {
  if (!fsReady || !deleteRecordedData()) return false;
  markFlashActivity();
  Preferences p;
  if (!p.begin("zktemp1",false)) return false;
  bool cleared=p.clear(); p.end();
  markFlashActivity();
  if (!cleared) return false;
  // Nach Flash-Operationen RAM-Aenderungen auf dem Mess-Core ausfuehren.
  const uint32_t request=__atomic_load_n(&clearMeasureRequest,__ATOMIC_RELAXED)+1;
  __atomic_store_n(&clearMeasureRequest,request,__ATOMIC_RELEASE);
  uint32_t start=millis();
  while (__atomic_load_n(&clearMeasureAck,__ATOMIC_ACQUIRE)!=request && (uint32_t)(millis()-start)<2000)
    vTaskDelay(pdMS_TO_TICKS(2));
  if (__atomic_load_n(&clearMeasureAck,__ATOMIC_ACQUIRE)!=request) return false;
  lastTempSaveMs=millis();
  portENTER_CRITICAL(&ntpDataMux);
  ntpResetGeneration++;
  for (int i=0;i<2;i++) {
    ntpValid[i]=false; ntpEpochUs[i]=0; ntpMonoUs[i]=0;
    ntpAccepted[i]=ntpRejectedDelay[i]=ntpRejectedProtocol[i]=0;
    ntpLatestRawRttUs[i]=ntpLatestNetRttUs[i]=ntpLatestProcUs[i]=0;
    ntpRttUs[i]=ntpRawRttUs[i]=ntpServerProcUs[i]=0;
    ntpLastOkMs[i]=ntpWindowStartMs[i]=ntpLastAttemptMs[i]=0;
  }
  portEXIT_CRITICAL(&ntpDataMux);
  ntpWorkerLocalQueries=ntpWorkerPoolQueries=0;
  ntpWorkerFailedQueries[0]=ntpWorkerFailedQueries[1]=0;
  ntpWorkerSkippedNoGps[0]=ntpWorkerSkippedNoGps[1]=0;
  return true;
}

String pcntDiagJson() {
  MeasurementView v; readMeasurement(v);
  const PcntRunFailureSnapshot &fe=v.firstRunPcntFailure;
  String j; j.reserve(1500);
  j="{\"valid\":"; j+=fe.valid?"true":"false";
  j+=",\"kind\":"+String((int)fe.kind);
  j+=",\"uptimeMs\":"+String(fe.uptimeMs);
  j+=",\"ppsSeq\":"+String(fe.ppsSeq);
  j+=",\"ocOk\":"; j+=fe.ocOk?"true":"false";
  j+=",\"r32Ok\":"; j+=fe.r32Ok?"true":"false";
  j+=",\"ocA\":"+String(fe.ocTrace.a.count)+",\"ocB\":"+String(fe.ocTrace.b.count)+",\"ocC\":"+String(fe.ocTrace.c.count);
  j+=",\"ocAUs\":"+String((long long)fe.ocTrace.a.mono)+",\"ocBUs\":"+String((long long)fe.ocTrace.b.mono)+",\"ocCUs\":"+String((long long)fe.ocTrace.c.mono);
  j+=",\"ocADurUs\":"+String(fe.ocTrace.a.callUs)+",\"ocBDurUs\":"+String(fe.ocTrace.b.callUs)+",\"ocCDurUs\":"+String(fe.ocTrace.c.callUs);
  j+=",\"ocThird\":"; j+=fe.ocTrace.thirdRead?"true":"false";
  j+=",\"r32A\":"+String(fe.r32Trace.a.count)+",\"r32B\":"+String(fe.r32Trace.b.count)+",\"r32C\":"+String(fe.r32Trace.c.count);
  j+=",\"r32AUs\":"+String((long long)fe.r32Trace.a.mono)+",\"r32BUs\":"+String((long long)fe.r32Trace.b.mono)+",\"r32CUs\":"+String((long long)fe.r32Trace.c.mono);
  j+=",\"r32ADurUs\":"+String(fe.r32Trace.a.callUs)+",\"r32BDurUs\":"+String(fe.r32Trace.b.callUs)+",\"r32CDurUs\":"+String(fe.r32Trace.c.callUs);
  j+=",\"r32Third\":"; j+=fe.r32Trace.thirdRead?"true":"false";
  j+=",\"intervalPrev\":"+String(fe.intervalPrevRaw)+",\"intervalCurr\":"+String(fe.intervalCurrRaw)+",\"intervalSpan\":"+String(fe.intervalSpan);
  j+=",\"intervalHz\":"+jnum(fe.intervalPerSecond,6);
  j+=",\"flashAgeMs\":"+String(fe.flashAgeMs)+",\"logAgeMs\":"+String(fe.logAgeMs);
  j+=",\"rtcAging\":"+String((int)fe.rtcAging)+",\"rtcCalElapsedMs\":"+String(fe.rtcCalElapsedMs)+",\"rtcCalTargetMs\":"+String(fe.rtcCalTargetMs);
  j+=",\"rtcCalLastReason\":\""+String(rtcCalReasonName((RtcCalResetReason)fe.rtcCalLastReason))+"\"}";
  return j;
}

String trendJson() {
  static TrendPoint points[TREND_MAX];
  int count=0;
  portENTER_CRITICAL(&trendDataMux);
  count=trendCount;
  for (int i=0;i<count;i++) points[i]=trendHistory[(trendPos-count+i+TREND_MAX)%TREND_MAX];
  portEXIT_CRITICAL(&trendDataMux);
  String j="[";j.reserve(18000);
  for(int i=0;i<count;++i) {
    const TrendPoint &p=points[i];
    if(i)j+=",";
    j+="{\"t\":"+String(p.uptime)+",\"p\":"+jnum(p.ocxoPpm300,6)+
       ",\"c\":"+String(p.cno)+"}";
  }
  j+="]";return j;
}


String adevJson() {
  AdevAccumulator a[ADEV_TAU_COUNT];
  uint32_t validSeconds,rejectedSeconds,currentRun;
  portENTER_CRITICAL(&adevDataMux);
  memcpy(a,adevAcc,sizeof(a));
  validSeconds=adevValidSeconds;
  rejectedSeconds=adevRejectedSeconds;
  currentRun=adevCurrentRun;
  portEXIT_CRITICAL(&adevDataMux);

  String j; j.reserve(900);
  j="{\"method\":\"classic_nonoverlap\",\"validSeconds\":";
  j+=String(validSeconds);
  j+=",\"rejectedSeconds\":"; j+=String(rejectedSeconds);
  j+=",\"currentRun\":"; j+=String(currentRun);
  j+=",\"points\":[";
  for (int i=0;i<ADEV_TAU_COUNT;i++) {
    if (i) j+=",";
    const double av=a[i].pairs ? sqrt(0.5*a[i].sumSqDiff/(double)a[i].pairs) : NAN;
    j+="{\"tau\":"+String(ADEV_TAU_S[i])+",\"adev\":"+jnum(av,12)+
       ",\"pairs\":"+String(a[i].pairs)+"}";
  }
  j+="]}";
  return j;
}

// ------------------------------------------------------------
// WLAN / Web
// ------------------------------------------------------------

void loadConfig() {
  prefs.begin("rtcdrift", true);
  wifiSsid = prefs.getString("ssid", "");
  wifiPass = prefs.getString("pass", "");
  localNtpHost = prefs.getString("localntp", "192.168.178.62");
  poolNtpHost = prefs.getString("poolntp", "pool.ntp.org");
  prefs.end();
}

void startAp() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  dns.start(53, "*", WiFi.softAPIP());
  apMode = true;

  Serial.printf("Setup AP: %s / %s / %s\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
}

bool connectWifi() {
  if (wifiSsid.isEmpty()) return false;

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());

  uint32_t start = millis();

  while (WiFi.status() != WL_CONNECTED &&
         millis() - start < 15000) {
    // Mess-Task startet erst nach der WLAN-Initialisierung; kein UART-Zugriff hier.
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) return false;

  Serial.printf("WiFi: %s\n", WiFi.localIP().toString().c_str());

  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
  }

  return true;
}

String jnum(double v, int d = 3) {
  return isfinite(v) ? String(v,d) : "null";
}

String statusJson() {
  uint32_t seq, pper, sseq, sper;
  int64_t pmono, smono;
  snapshotBothPulses(seq, pper, pmono, sseq, sper, smono);

  MeasurementView v;
  readMeasurement(v);
  const uint64_t gpsEpoch=v.gpsEpoch, rtcEpoch=v.rtcEpoch;
  const bool haveGps=v.haveGps, haveRtc=v.haveRtc;

  String j;
  j.reserve(8192);
  j = "{";
  j += "\"gpsFix\":" + String(v.gpsFix ? "true" : "false") + ",";
  j += "\"clockState\":\"" + String(v.clockMode) + "\",";
  j += "\"clockTime\":\"" + fmtTime(clockNowUs(), true, true) + "\",";
  j += "\"clockUtc\":\"" + fmtTime(clockNowUs(), false, true) + "\",";
  j += "\"clockDate\":\"" + fmtDate(clockNowUs(), true) + "\",";
  j += "\"clockUncertaintyMs\":" + jnum(v.uncertaintyMs,2) + ",";
  j += "\"gpsPpsAssumption\":\"RMC=vorausgehender PPS\",";
  j += "\"ntpStratumGps\":" + String(GPS_NTP_STRATUM) + ",";
  j += "\"measureCore\":1,";
  j += "\"measureAgeMs\":" + String(v.sampleMs ? (uint32_t)(millis()-v.sampleMs) : 4294967295UL) + ",";
  j += "\"webCore\":0,";
  j += "\"phase\":\"" + String(runPhaseText(v.runPhase)) + "\",";
  j += "\"warmupSeconds\":" + String((unsigned long)(OCXO_STARTUP_OBSERVATION_MS/1000U)) + ",";
  j += "\"phaseElapsedSeconds\":" + String(v.measurementStartedMs ? (uint32_t)(millis()-v.measurementStartedMs)/1000U : 0U) + ",";
  j += "\"badWindowWarmup\":" + String(v.badWindowsByPhase[0]) + ",";
  j += "\"badWindowGpsWait\":" + String(v.badWindowsByPhase[1]) + ",";
  j += "\"badWindowRun\":" + String(v.badWindowsByPhase[2]) + ",";
  j += "\"ocxoRejectWarmup\":" + String(v.rejectedOcxoByPhase[0]) + ",";
  j += "\"ocxoRejectGpsWait\":" + String(v.rejectedOcxoByPhase[1]) + ",";
  j += "\"ocxoRejectRun\":" + String(v.rejectedOcxoByPhase[2]) + ",";
  j += "\"overflowWarmup\":" + String(v.overflowSuspectsByPhase[0]) + ",";
  j += "\"overflowGpsWait\":" + String(v.overflowSuspectsByPhase[1]) + ",";
  j += "\"overflowRun\":" + String(v.overflowSuspectsByPhase[2]) + ",";
  j += "\"rejectedGpsReference\":" + String(v.rejectedGpsReference) + ",";
  j += "\"rtcEventsDropped\":" + String(rtcEventsDropped) + ",";
  j += "\"counterReadFailures\":" + String(v.counterReadFailures) + ",";
  j += "\"pcntDriverReadErrors\":" + String(v.pcntDriverReadErrors) + ",";
  j += "\"pcntOcxoUnrecoverable\":" + String(v.pcntOcxoUnrecoverable) + ",";
  j += "\"pcntRtc32Unrecoverable\":" + String(v.pcntRtc32Unrecoverable) + ",";
  j += "\"pcntOcxoUnrecoverableRun\":" + String(v.pcntOcxoUnrecoverableRun) + ",";
  j += "\"pcntRtc32UnrecoverableRun\":" + String(v.pcntRtc32UnrecoverableRun) + ",";
  j += "\"pcntReadAttempts\":" + String(pcntReadAttempts) + ",";
  j += "\"pcntRetryReads\":" + String(pcntRetryReads) + ",";
  j += "\"pcntRecoveredReads\":" + String(pcntRecoveredReads) + ",";
  j += "\"pcntRecoveredSpans\":" + String(pcntRecoveredSpans) + ",";
  j += "\"counterInvalidWindows\":" + String(v.counterInvalidWindows) + ",";
  j += "\"ocxoOverflowSuspects\":" + String(v.ocxoOverflowSuspects) + ",";
  j += "\"pcntOverflowLimit\":" + String(PCNT_OVERFLOW_LIMIT) + ",";
  j += "\"flashGuardRejects\":" + String(v.flashGuardRejectedPps) + ",";
  j += "\"flashLastMs\":" + String((unsigned long)lastFlashActivityMs) + ",";
  j += "\"pcntIramSafe\":" + String(CONFIG_PCNT_ISR_IRAM_SAFE ? "true" : "false") + ",";
  j += "\"gpioIsrIram\":" + String(CONFIG_ARDUINO_ISR_IRAM ? "true" : "false") + ",";
  j += "\"ocxoPpsRejects\":" + String(v.ocxoPpsRejects) + ",";
  j += "\"lastBadOcxoPpm\":" + jnum(v.lastBadOcxoPpm,3) + ",";
  j += "\"counterDiscontinuities\":" + String(v.counterDiscontinuities) + ",";
  j += "\"ppsExtrapolationWarnings\":" + String(v.ppsExtrapolationWarnings) + ",";
  j += "\"chipTempC\":" + jnum(v.chipTempC,2) + ",";
  j += "\"ocxoTempProxyC\":" + jnum(v.ocxoTempProxyC,2) + ",";
  j += "\"espPpm\":" + jnum(v.espEstimatedPpm,4) + ",";
  j += "\"ocxoTempPpm\":" + jnum(v.ocxoEstimatedPpm,6) + ",";
  j += "\"tempLearnCount\":" + String(v.tempLearningCount) + ",";
  j += "\"nmeaOk\":" + String(v.nmeaOk) + ",";
  j += "\"nmeaBad\":" + String(v.nmeaBad) + ",";
  j += "\"timeAnomalies\":" + String(v.gnssTimeAnomalies) + ",";
  j += "\"ppsLossEvents\":" + String(v.gnssLostEvents) + ",";
  j += "\"rtcLastSync\":" + String((unsigned long long)v.rtcLastSyncEpoch) + ",";
  j += "\"rtcSyncCount\":" + String(v.rtcSyncCount) + ",";
  j += "\"ocxoSignal\":" + String(v.ocxoSignal ? "true" : "false") + ",";
  j += "\"rtc32Signal\":" + String(v.rtc32Signal ? "true" : "false") + ",";
  j += "\"ntpServed\":" + String(ntpServed) + ",";
  j += "\"ntpWorkerCore\":" + String(ntpWorkerAssignedCore) + ",";
  j += "\"loopCore\":" + String(xPortGetCoreID()) + ",";
  j += "\"ntpWorkerLocalQueries\":" + String(ntpWorkerLocalQueries) + ",";
  j += "\"ntpWorkerPoolQueries\":" + String(ntpWorkerPoolQueries) + ",";
  j += "\"ntpWorkerErrorsLocal\":" + String(ntpWorkerFailedQueries[0]) + ",";
  j += "\"ntpWorkerErrorsPool\":" + String(ntpWorkerFailedQueries[1]) + ",";
  j += "\"ntpWorkerSkippedGpsLocal\":" + String(ntpWorkerSkippedNoGps[0]) + ",";
  j += "\"ntpWorkerSkippedGpsPool\":" + String(ntpWorkerSkippedNoGps[1]) + ",";
  j += "\"wifiRssi\":" + String(WiFi.status()==WL_CONNECTED ? WiFi.RSSI() : -127) + ",";
  j += "\"wifiStatus\":" + String((int)WiFi.status()) + ",";
  j += "\"wifiDisconnects\":" + String(wifiDisconnectEvents) + ",";
  j += "\"wifiGotIp\":" + String(wifiGotIpEvents) + ",";
  j += "\"wifiLostIp\":" + String(wifiLostIpEvents) + ",";
  j += "\"wifiLastReason\":" + String(wifiLastDisconnectReason) + ",";
  j += "\"wifiLastEventAgeMs\":" + String(wifiLastEventMs ? (uint32_t)(millis()-wifiLastEventMs) : 0U) + ",";
  j += "\"heapFree\":" + String((uint32_t)heap_caps_get_free_size(MALLOC_CAP_8BIT)) + ",";
  j += "\"heapMin\":" + String((uint32_t)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT)) + ",";
  j += "\"heapLargest\":" + String((uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)) + ",";
  j += "\"loopMaxGapUs\":" + String(loopMaxGapUs) + ",";
  j += "\"loopMaxWorkUs\":" + String(loopMaxWorkUs) + ",";
  j += "\"webHandleMaxUs\":" + String(webHandleMaxUs) + ",";
  j += "\"lcdMaxUs\":" + String(lcdMaxUs) + ",";
  j += "\"lcdChunkMaxUs\":" + String(lcdChunkMaxUs) + ",";
  j += "\"lcdWriteChunks\":" + String(lcdWriteChunks) + ",";
  j += "\"lcdWriteChars\":" + String(lcdWriteChars) + ",";
  j += "\"lcdBusBusySkips\":" + String(lcdBusBusySkips) + ",";
  j += "\"loggerMaxUs\":" + String(loggerMaxUs) + ",";
  j += "\"loopGapOver50ms\":" + String(loopGapOver50ms) + ",";
  j += "\"loopGapOver200ms\":" + String(loopGapOver200ms) + ",";
  j += "\"webOver50ms\":" + String(webOver50ms) + ",";
  j += "\"ntpDnsFailLocal\":" + String(ntpDnsFailures[0]) + ",";
  j += "\"ntpDnsFailPool\":" + String(ntpDnsFailures[1]) + ",";
  j += "\"ntpSendFailLocal\":" + String(ntpSendFailures[0]) + ",";
  j += "\"ntpSendFailPool\":" + String(ntpSendFailures[1]) + ",";
  j += "\"ntpTimeoutLocal\":" + String(ntpTimeouts[0]) + ",";
  j += "\"ntpTimeoutPool\":" + String(ntpTimeouts[1]) + ",";
  j += "\"ntpClockFailLocal\":" + String(ntpClockFailures[0]) + ",";
  j += "\"ntpClockFailPool\":" + String(ntpClockFailures[1]) + ",";
  j += "\"ntpDnsLastLocalUs\":" + String(ntpDnsLastUs[0]) + ",";
  j += "\"ntpDnsLastPoolUs\":" + String(ntpDnsLastUs[1]) + ",";
  j += "\"ntpDnsMaxLocalUs\":" + String(ntpDnsMaxUs[0]) + ",";
  j += "\"ntpDnsMaxPoolUs\":" + String(ntpDnsMaxUs[1]) + ",";
  j += "\"ntpQueryMaxLocalUs\":" + String(ntpQueryMaxUs[0]) + ",";
  j += "\"ntpQueryMaxPoolUs\":" + String(ntpQueryMaxUs[1]) + ",";
  j += "\"gpsSats\":" + String(v.gpsSats) + ",";
  j += "\"gpsVisible\":" + String(v.gpsVisible) + ",";
  j += "\"gpsCno\":" + String(v.gpsBestCno) + ",";
  j += "\"gpsGpVisible\":" + String(v.gpsGpVisible) + ",";
  j += "\"gpsGpSignal\":" + String(v.gpsGpSignal) + ",";
  j += "\"gpsGpCno\":" + String(v.gpsGpCno) + ",";
  j += "\"gpsBdVisible\":" + String(v.gpsBdVisible) + ",";
  j += "\"gpsBdSignal\":" + String(v.gpsBdSignal) + ",";
  j += "\"gpsBdCno\":" + String(v.gpsBdCno) + ",";
  j += "\"gpsGnVisible\":" + String(v.gpsGnVisible) + ",";
  j += "\"gpsHdop\":" + jnum(v.gpsHdop,2) + ",";
  j += "\"gpsFixType\":" + String(v.gpsFixType) + ",";
  j += "\"gpsAntenna\":\"" + String(antennaTextFor(v.antennaStatus)) + "\",";
  j += "\"gpsPpsUs\":" + String(pper) + ",";
  j += "\"gpsPpsCount\":" + String(seq) + ",";
  const int64_t ppsAgeUs = pmono ? esp_timer_get_time() - pmono : 0;
  j += "\"gpsPpsAgeMs\":" + String(pmono ? (ppsAgeUs / 1000.0) : -1.0, 1) + ",";
  j += "\"gpsTime\":\"" + String(haveGps ? fmtTime(gpsEpoch*1000000ULL,true,true) : "--") + "\",";

  j += "\"rtcTime\":\"" + String(haveRtc ? fmtTime(rtcEpoch*1000000ULL,true,false) : "--") + "\",";
  j += "\"rtcDate\":\"" + String(haveRtc ? fmtDate(rtcEpoch*1000000ULL,true) : "--") + "\",";
  j += "\"rtcSqwUs\":" + String(sper) + ",";
  j += "\"rtcGpsPhaseMs\":" + jnum(v.rtcGpsPhaseMs) + ",";
  j += "\"rtcGpsDriftMs\":" + jnum(v.rtcGpsDriftMs) + ",";
  j += "\"rtcAgingOffset\":" + String((int)v.rtcAgingOffset) + ",";
  j += "\"rtcCalRatePpm\":" + jnum(v.rtcMeasuredRatePpm,5) + ",";
  j += "\"rtcLiveRatePpm\":" + jnum(v.rtcLiveRatePpm,5) + ",";
  j += "\"rtcLiveRateSpanSec\":" + String(v.rtcLiveRateSpanSec) + ",";
  j += "\"rtcCalAdjustments\":" + String(v.rtcAgingAdjustments) + ",";
  j += "\"rtcCalHours\":" + String(v.rtcCalStartMs ? (millis()-v.rtcCalStartMs)/3600000.0 : 0.0,2) + ",";
  j += "\"rtcCalTargetHours\":" + String(v.rtcCalTargetMs/3600000.0,2) + ",";
  j += "\"rtcCalProvisionalPpm\":" + jnum(v.rtcCalProvisionalPpm,5) + ",";
  j += "\"rtcCalLastReset\":\"" + String(rtcCalReasonName(v.rtcCalLastReason)) + "\",";
  j += "\"rtcCalResetGps\":" + String(v.rtcCalResetCount[RTC_CAL_GPS]) + ",";
  j += "\"rtcCalResetSqw\":" + String(v.rtcCalResetCount[RTC_CAL_SQW]) + ",";
  j += "\"rtcCalResetTemperature\":" + String(v.rtcCalResetCount[RTC_CAL_TEMPERATURE]) + ",";
  j += "\"rtcCalResetPhase\":" + String(v.rtcCalResetCount[RTC_CAL_PHASE]) + ",";
  j += "\"rtcCalResetRate\":" + String(v.rtcCalResetCount[RTC_CAL_RATE]) + ",";
  j += "\"rtcCalResetSync\":" + String(v.rtcCalResetCount[RTC_CAL_SYNC]) + ",";
  j += "\"rtcCalResetRtcState\":" + String(v.rtcCalResetCount[RTC_CAL_RTC_STATE]) + ",";
  j += "\"rtcCalGpsFailFix\":" + String(v.rtcCalGpsFailCount[GPS_FAIL_FIX]) + ",";
  j += "\"rtcCalGpsFailSats\":" + String(v.rtcCalGpsFailCount[GPS_FAIL_SATS]) + ",";
  j += "\"rtcCalGpsFailPps\":" + String(v.rtcCalGpsFailCount[GPS_FAIL_PPS]) + ",";
  j += "\"rtcCalGpsFailRmc\":" + String(v.rtcCalGpsFailCount[GPS_FAIL_RMC]) + ",";
  j += "\"rtcCalGpsFailStable\":" + String(v.rtcCalGpsFailCount[GPS_FAIL_STABLE]) + ",";
  j += "\"rtcCalGpsFailMap\":" + String(v.rtcCalGpsFailCount[GPS_FAIL_MAP]) + ",";
  j += "\"rtcCalGpsFailCno\":" + String(v.rtcCalGpsFailCount[GPS_FAIL_CNO]) + ",";
  j += "\"rtcCalGpsFailAnt\":" + String(v.rtcCalGpsFailCount[GPS_FAIL_ANT]) + ",";
  j += "\"rtcCalLastGpsFailMask\":" + String(v.rtcCalLastGpsFailMask) + ",";
  j += "\"gpsUnlockEvents\":" + String(v.gpsUnlockEvents) + ",";
  j += "\"gpsUnlockCurrentMs\":" + String(v.gpsUnlockSinceMs ? (uint32_t)(millis()-v.gpsUnlockSinceMs) : 0U) + ",";
  j += "\"gpsUnlockLastMs\":" + String(v.gpsUnlockLastDurationMs) + ",";
  j += "\"gpsUnlockMaxMs\":" + String(v.gpsUnlockMaxDurationMs) + ",";
  j += "\"gpsUnlockCurrentReasonMask\":" + String(v.gpsUnlockCurrentReasonMask) + ",";
  j += "\"gpsUnlockLastReasonMask\":" + String(v.gpsUnlockLastReasonMask) + ",";
  j += "\"rtcLocalMs\":" + jnum(v.rtcLocalMs) + ",";

  j += "\"ocxo1\":" + jnum(v.ocxo1) + ",";
  j += "\"ocxo10\":" + jnum(v.ocxo10) + ",";
  j += "\"ocxo60\":" + jnum(v.ocxo60) + ",";
  j += "\"ocxoPpm10\":" + jnum(v.ocxoPpm10,6) + ",";
  j += "\"ocxoPpm60\":" + jnum(v.ocxoPpm60,6) + ",";
  j += "\"ocxo100\":" + jnum(v.ocxo100) + ",";
  j += "\"ocxo300\":" + jnum(v.ocxo300) + ",";
  j += "\"ocxo1000\":" + jnum(v.ocxo1000) + ",";
  j += "\"ocxoPpm100\":" + jnum(v.ocxoPpm100,6) + ",";
  j += "\"ocxoPpm300\":" + jnum(v.ocxoPpm300,6) + ",";
  j += "\"ocxoPpm1000\":" + jnum(v.ocxoPpm1000,9) + ",";

  j += "\"rtc321\":" + jnum(v.rtc321) + ",";
  j += "\"rtc3210\":" + jnum(v.rtc3210) + ",";
  j += "\"rtc3260\":" + jnum(v.rtc3260) + ",";
  j += "\"rtc32Ppm60\":" + jnum(v.rtc32Ppm60,6) + ",";
  j += "\"rtc32100\":" + jnum(v.rtc32100) + ",";
  j += "\"rtc32300\":" + jnum(v.rtc32300) + ",";
  j += "\"rtc321000\":" + jnum(v.rtc321000) + ",";
  j += "\"rtc32Ppm1000\":" + jnum(v.rtc32Ppm1000,6) + ",";

  j += "\"logBytes\":" + String((unsigned long)logFileSize()) + ",";
  j += "\"fsUsed\":" + String(fsReady ? (unsigned long)LittleFS.usedBytes() : 0UL) + ",";
  j += "\"fsTotal\":" + String(fsReady ? (unsigned long)LittleFS.totalBytes() : 0UL) + ",";
  j += "\"fsReady\":" + String(fsReady?"true":"false") + ",";
  j += "\"logRows\":" + String(logRows) + ",";
  j += "\"logWriteFailures\":" + String(logWriteFailures) + ",";
  j += "\"lastLogAgeSec\":" + String(lastLogWriteMs ? (long)((millis()-lastLogWriteMs)/1000) : -1L) + ",";
  j += "\"histCount\":" + String(v.histCount) + ",";
  j += "\"ocxoWindowCoverageMode\":\"98pct\",";
  const int validO100=v.validO100, validO300=v.validO300;
  const int validO1000=v.validO1000, validR100=v.validR100;
  j += "\"ocxoValid100\":" + String(validO100) + ",";
  j += "\"ocxoValid300\":" + String(validO300) + ",";
  j += "\"ocxoValid1000\":" + String(validO1000) + ",";
  j += "\"rtcValid100\":" + String(validR100) + ",";
  j += "\"badOcxoPps\":" + String(v.badOcxoPps) + ",";
  j += "\"badRtc32Pps\":" + String(v.badRtc32Pps) + ",";
  j += "\"badGpsPps\":" + String(v.badGpsPps) + ",";
  j += "\"persistentCount\":" + String(persistentCount) + ",";
  j += "\"persistentPpmMean\":" + jnum(persistentPpmMean,9) + ",";
  j += "\"persistentPpmMin\":" + jnum(persistentPpmMin,9) + ",";
  j += "\"persistentPpmMax\":" + jnum(persistentPpmMax,9) + ",";

  j += "\"ntpLocal\":\"" + String(ntpFresh(NTP_LOCAL) ? fmtTime(ntpNow(NTP_LOCAL),true,true) : "--") + "\",";
  j += "\"ntpPool\":\"" + String(ntpFresh(NTP_POOL) ? fmtTime(ntpNow(NTP_POOL),true,true) : "--") + "\",";
  j += "\"localRttMs\":" + jnum(ntpFresh(NTP_LOCAL) ? ntpRttUs[NTP_LOCAL]/1000.0 : NAN,3) + ",";
  j += "\"localRejectDelay\":" + String(ntpRejectedDelay[NTP_LOCAL]) + ",";
  j += "\"localRejectProtocol\":" + String(ntpRejectedProtocol[NTP_LOCAL]) + ",";
  j += "\"poolRejectProtocol\":" + String(ntpRejectedProtocol[NTP_POOL]) + ",";
  j += "\"poolRejectDelay\":" + String(ntpRejectedDelay[NTP_POOL]) + ",";
  j += "\"localLatestRawMs\":" + jnum(ntpLatestRawRttUs[NTP_LOCAL] ? ntpLatestRawRttUs[NTP_LOCAL]/1000.0 : NAN,3) + ",";
  j += "\"poolLatestRawMs\":" + jnum(ntpLatestRawRttUs[NTP_POOL] ? ntpLatestRawRttUs[NTP_POOL]/1000.0 : NAN,3) + ",";
  j += "\"localSampleAgeSec\":" + String(ntpFresh(NTP_LOCAL) ? (long)((millis()-ntpLastOkMs[NTP_LOCAL])/1000) : -1L) + ",";
  j += "\"poolSampleAgeSec\":" + String(ntpFresh(NTP_POOL) ? (long)((millis()-ntpLastOkMs[NTP_POOL])/1000) : -1L) + ",";
  j += "\"poolRttMs\":" + jnum(ntpFresh(NTP_POOL) ? ntpRttUs[NTP_POOL]/1000.0 : NAN,3) + ",";
  j += "\"localRawRttMs\":" + jnum(ntpFresh(NTP_LOCAL) ? ntpRawRttUs[NTP_LOCAL]/1000.0 : NAN,3) + ",";
  j += "\"poolRawRttMs\":" + jnum(ntpFresh(NTP_POOL) ? ntpRawRttUs[NTP_POOL]/1000.0 : NAN,3) + ",";
  j += "\"localProcMs\":" + jnum(ntpFresh(NTP_LOCAL) ? ntpServerProcUs[NTP_LOCAL]/1000.0 : NAN,3) + ",";
  j += "\"poolProcMs\":" + jnum(ntpFresh(NTP_POOL) ? ntpServerProcUs[NTP_POOL]/1000.0 : NAN,3) + ",";
  j += "\"localGpsMs\":" + jnum(v.localGpsMs) + ",";
  j += "\"poolGpsMs\":" + jnum(v.poolGpsMs) + ",";
  j += "\"localPoolMs\":" + jnum(v.localPoolMs);
  j += "}";
  return j;
}

String page() {
  return R"rawliteral(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Zeitkiste</title>
<style>
body{font-family:system-ui;max-width:850px;margin:25px auto;padding:0 15px;background:#111;color:#eee}
.card{background:#1d1d1d;border:1px solid #333;border-radius:12px;padding:14px;margin:12px 0}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.v{font:600 1.2rem monospace}
small{color:#aaa}
input,button{box-sizing:border-box;width:100%;padding:10px;margin:5px 0 10px;background:#222;color:#fff;border:1px solid #444;border-radius:7px}
button{background:#386fa8}
button.danger{background:#713333;border-color:#9b5151}
button.danger:hover{background:#933939}
table{width:100%}td{padding:5px}td:last-child{text-align:right;font-family:monospace}
.gauge{display:grid;grid-template-columns:4.5rem 1fr 9.5rem;gap:10px;align-items:center;margin:11px 0}
.glabel{font-weight:700;font-family:monospace}.gtrack{position:relative;height:20px;border:1px solid #555;border-radius:10px;background:linear-gradient(90deg,#262626,#171717 49.5%,#555 49.5%,#555 50.5%,#171717 50.5%,#262626)}
.gmark{position:absolute;top:-4px;width:5px;height:28px;border-radius:2px;background:#e8e8e8;transform:translateX(-50%);transition:left .9s linear}.gnum{text-align:right;font:600 1rem monospace}.gmeta{grid-column:2/4;color:#999;font:0.78rem monospace;margin-top:-8px}.invalid .gmark{display:none}.invalid .gnum{color:#888}
.statline{font:0.95rem monospace;margin:5px 0}.canv{width:100%;height:190px}
@media(max-width:600px){.grid{grid-template-columns:1fr}.gauge{grid-template-columns:3.6rem 1fr 7.2rem}.gmeta{grid-column:1/4}.canv{height:175px}}
</style></head><body>
<h1>Zeitkiste %FW_VERSION% | ESP32-S3</h1>
<div class="grid">
<div class="card"><b>Zeitbasis / Zustand</b><div id="clock" class="v">--</div><small id="clock2"></small></div>
<div class="card"><b>Temperaturmodell</b><div id="temp" class="v">--</div><small id="temp2"></small></div>
<div class="card"><b>GPS</b><div id="gps" class="v">--</div><small id="gps2"></small></div>
<div class="card"><b>OCXO</b><div id="ocxo" class="v">--</div><small id="ocxo2"></small></div>
<div class="card"><b>RTC</b><div id="rtc" class="v">--</div><small id="rtc2"></small></div>
<div class="card"><b>NTP lokal</b><div id="ntp" class="v">--</div><small id="ntp2"></small></div>
</div>

<div class="card"><b>Gedämpfte Nullinstrumente</b>
<small>Zeiger und Zahl benutzen denselben trägen Tiefpass. Zeitkonstanten: OFEN 100 s, RTC 20 s, ESP 30 s, NTP 60 s. RTC-Gang live wird nach 30 s sichtbar und nutzt rollend bis 120 s SQW gegen GPS-PPS. Die Skala ist je Messgröße separat.</small>
<div class="gauge" id="gOcxo"><div class="glabel">OFEN</div><div class="gtrack"><div class="gmark"></div></div><div class="gnum">--</div><div class="gmeta">--</div></div>
<div class="gauge" id="gRtc"><div class="glabel">RTC</div><div class="gtrack"><div class="gmark"></div></div><div class="gnum">--</div><div class="gmeta">--</div></div>
<div class="gauge" id="gEsp"><div class="glabel">ESP</div><div class="gtrack"><div class="gmark"></div></div><div class="gnum">--</div><div class="gmeta">--</div></div>
<div class="gauge" id="gNtp"><div class="glabel">NTP</div><div class="gtrack"><div class="gmark"></div></div><div class="gnum">--</div><div class="gmeta">--</div></div>
</div>

<div class="card"><b>Vergleiche</b><table>
<tr><td>NTP lokal − GPS</td><td id="lg">--</td></tr>
<tr><td>Pool − GPS</td><td id="pg">--</td></tr>
<tr><td>Lokal − Pool</td><td id="lp">--</td></tr>
<tr><td>RTC Phase − GPS</td><td id="rgp">--</td></tr>
<tr><td>RTC Drift seit Start</td><td id="rgd">--</td></tr>
<tr><td>RTC − NTP lokal</td><td id="rl">--</td></tr>
</table></div>

<div class="card"><b>WLAN / NTP</b>
<form method="POST" action="/save">
SSID<input name="ssid" value="%SSID%">
Passwort<input type="password" name="pass" placeholder="leer = unveraendert">
Lokaler NTP<input name="localntp" value="%LOCAL%">
Pool-NTP<input name="poolntp" value="%POOL%">
<button>Speichern & Neustart</button>
</form>
</div>

<div class="card"><b>Standalone-Logger</b>
<div id="store" class="v">--</div>
<small id="store2"></small><br>
<small id="store3"></small><br><br>
<a href="/log.csv" style="color:#8bc4ff">CSV herunterladen</a>
<form method="POST" action="/data/delete" onsubmit="return confirmDelete('ALLE Messwerte, CSV-Dateien, Statistiken, Verlaeufe und Temperatur-Lernmodelle')">
<input type="hidden" name="token" value="%RESETTOKEN%">
<input type="hidden" name="kind" value="all">
<input type="hidden" name="confirm" value="LOESCHEN">
<button type="submit" class="danger">ALLE Messwerte loeschen</button></form>
<small>Loescht CSV, Ereignisse, Statistik, Messpuffer, Trend und BEIDE Temperatur-Lernmodelle. WLAN- und NTP-Konfiguration sowie die aktuelle RTC-Uhrzeit bleiben unveraendert. Die aktuell verwendete ESP-Taktkorrektur bleibt bis zur naechsten Neuberechnung aktiv.</small>
<div id="fsinit" style="display:none;margin-top:15px;border-top:1px solid #555;padding-top:12px">
<b>Dateisystem nicht eingebunden</b>
<p>Eine Erstformatierung loescht ALLE bisherigen Dateien in der Dateisystempartition, aber nicht die Firmware. Nur wenn diese Daten nicht mehr gebraucht werden.</p>
<form method="POST" action="/fs/initialize" onsubmit="return confirm('Dateisystem unwiderruflich loeschen?')">
Code vom LCD (FS ERR): <input name="code" inputmode="numeric" pattern="[0-9]{6}" required>
Zur Bestaetigung FORMATIEREN eingeben: <input name="confirm" pattern="FORMATIEREN" required>
<button>Nur Dateisystempartition initialisieren</button>
</form></div>
</div>

<div class="card"><b>RTC-Synchronisierung / Aging</b><p>GPS-PPS stellt die RTC nur bei relevanter Abweichung. DS3231M-Aging-Kalibrierung adaptiv: &gt;=0.5 ppm 10 min, 0.1..0.5 ppm 1 h, 0.03..0.1 ppm 4 h, darunter bis 24 h; maximal 2 Registerschritte pro Zyklus. Die ESP-Chiptemperatur verwirft kein RTC-Kalibrierfenster mehr. Der 32-kHz-Ausgang ist UNKOMPENSIERT.</p></div>
<div class="card"><b>Fehleranalyse nach Betriebsphase</b>
<p><small>Anlauf (erste 30 min) und fehlender GPS-Fix werden unabhaengig gezaehlt; die Spalten sind daher nicht exklusiv. Dies misst nicht die tatsaechliche OCXO-Ofentemperatur.</small></p>
<div id="phaseInfo" class="v">--</div>
<table><tr><th>Diagnose</th><th>Anlauf</th><th>GPS warten</th><th>Betrieb</th></tr>
<tr><td>PCNT-PPS-Snapshot unrettbar</td><td id="qbw">--</td><td id="qbg">--</td><td id="qbr">--</td></tr>
<tr><td>OCXO-PPS verworfen</td><td id="qow">--</td><td id="qog">--</td><td id="qor">--</td></tr>
<tr><td>Overflow-Verdacht*</td><td id="qfw">--</td><td id="qfg">--</td><td id="qfr">--</td></tr>
</table><small>* Anlauf und GPS-warten sind unabhaengige Diagnoseachsen und koennen denselben Fehler beide zaehlen. Overflow ist nur ein Musterverdacht, kein bestaetigter Hardwarefehler.</small>
<div id="firstPcntErr" class="statline">Erster Betriebsfehler: --</div>
</div>
<div class="card"><b>Messqualitaet</b><div id="quality">--</div>
<p><small>Frequenzwerte sind PPS-nahe, softwarezeitgestempelte PCNT-Schaetzungen ohne Hardware-Latch. PCNT wird nur noch PPS-bezogen gelesen; Einzel-Read-Ausreisser verschieben die letzte gute Basis nicht. GPS-RMC/PPS-Sekundenzuordnung muss extern bestaetigt werden. Die Unsicherheit ist eine Schaetzung, keine Kalibrierbescheinigung. Hochlatente NTP-Proben werden verworfen.</small></p></div>
<div class="card"><b>OCXO-Statistik / Verlauf (letzte Stunde im RAM)</b>
<small>Statistik über die gültigen 300-s-Punkte des dargestellten 10-s-Trends. Die drei Linien sind Mittelwert sowie Mittelwert ±1σ.</small>
<div id="ocxoStats" class="statline">--</div>
<canvas id="ocxoTrend" class="canv" width="760" height="190"></canvas>
<canvas id="cnoTrend" class="canv" width="760" height="175"></canvas>
</div>
<div class="card"><b>Allan Deviation des OCXO</b>
<small>Klassische ADEV aus nicht überlappenden Mittelwertblöcken echter, zusammenhängender 1-s-OCXO/PPS-Intervalle. Verworfene oder über Mehrsekunden-Spans gerettete Intervalle trennen die Serie. Angezeigt wird damit die Stabilität des Messpfads OCXO gegen GPS/PPS, nicht eine vom Referenz- und Messjitter getrennte Labor-ADEV des OCXO allein.</small>
<div id="adevInfo" class="statline">--</div>
<table><thead><tr><th>τ</th><th>ADEV</th><th>Blockpaare</th></tr></thead><tbody id="adevTable"><tr><td colspan="3">--</td></tr></tbody></table>
<canvas id="adevTrend" class="canv" width="760" height="190"></canvas>
</div>

<script>
function confirmDelete(what){
  if(!confirm(what+' unwiderruflich loeschen?'))return false;
  return prompt('Zur Bestaetigung LOESCHEN eingeben:')==='LOESCHEN';
}
function n(v,d=3,u=''){return v==null?'--':(v>=0?'+':'')+v.toFixed(d)+u}
function f(v,d=3,u=''){return v==null?'--':v.toFixed(d)+u}
const gaugeState={};
const GAUGE_TAU_OCXO=100.0;
const GAUGE_TAU_RTC=20.0;
const GAUGE_TAU_ESP=30.0;
const GAUGE_TAU_NTP=60.0;
function gauge(id,raw,fullScale,dec,unit,source,tau,valid=true){
  const el=document.getElementById(id),mark=el.querySelector('.gmark'),num=el.querySelector('.gnum'),meta=el.querySelector('.gmeta');
  valid=valid&&Number.isFinite(raw)&&Number.isFinite(fullScale)&&fullScale>0&&Number.isFinite(tau)&&tau>0;
  if(!valid){el.classList.add('invalid');num.textContent='--';meta.textContent='Skala ±'+fullScale+' '+unit+(source?' | '+source:'');return;}
  el.classList.remove('invalid');
  const now=performance.now()/1000;
  let st=gaugeState[id];
  if(!st||!Number.isFinite(st.v)){st={v:raw,t:now};gaugeState[id]=st;}
  else {let dt=Math.max(.001,Math.min(2,now-st.t));let a=1-Math.exp(-dt/tau);st.v+=a*(raw-st.v);st.t=now;}
  let q=Math.max(-1,Math.min(1,st.v/fullScale));mark.style.left=((q+1)*50)+'%';
  num.textContent=(st.v>=0?'+':'')+st.v.toFixed(dec)+' '+unit;
  meta.textContent='Skala ±'+fullScale.toFixed(dec)+' '+unit+' | τ '+tau.toFixed(0)+' s'+(source?' | '+source:'');
}
function gpsFailText(mask){
  const a=[];
  if(mask&1)a.push('FIX'); if(mask&2)a.push('SATS'); if(mask&4)a.push('PPS'); if(mask&8)a.push('RMC');
  if(mask&16)a.push('STABIL'); if(mask&32)a.push('MAP'); if(mask&64)a.push('CNO'); if(mask&128)a.push('ANT');
  return a.length?a.join('+'):'--';
}
function bestOcxo(x){
  if(x.ocxoValid1000>=980&&Number.isFinite(x.ocxoPpm1000))return [x.ocxoPpm1000,'1000 s'];
  if(x.ocxoValid300>=294&&Number.isFinite(x.ocxoPpm300))return [x.ocxoPpm300,'300 s'];
  if(x.ocxoValid100>=98&&Number.isFinite(x.ocxoPpm100))return [x.ocxoPpm100,'100 s'];
  return [null,'kein gültiges Fenster'];
}
async function upd(){
 try{
  const x=await fetchJsonTimed('/status',4000);

  document.getElementById('clock').textContent=x.clockDate+' '+x.clockTime+' '+x.clockState;
  document.getElementById('clock2').textContent='UTC '+x.clockUtc+
    ' | Unsicherheit '+f(x.clockUncertaintyMs,2,' ms')+
    ' | RTC-Sync '+x.rtcSyncCount+' | NTP beantwortet '+x.ntpServed;
  document.getElementById('temp').textContent=f(x.chipTempC,1,' C');
  document.getElementById('temp2').textContent='Ofen-Proxy '+f(x.ocxoTempProxyC,1,' C')+' | ESP '+n(x.espPpm,3,' ppm')+
    ' | OCXO '+n(x.ocxoTempPpm,5,' ppm')+
    ' | Lernungen '+x.tempLearnCount;
  document.getElementById('gps').textContent=x.gpsTime+' '+(x.gpsFix?'FIX':'NO FIX');
  document.getElementById('gps2').textContent=
    'Benutzt '+x.gpsSats+' / sichtbar '+x.gpsVisible+
    ' | GPS '+x.gpsGpVisible+' ('+x.gpsGpSignal+' Signal, max '+x.gpsGpCno+' dB-Hz)'+
    ' | BeiDou '+x.gpsBdVisible+' ('+x.gpsBdSignal+' Signal, max '+x.gpsBdCno+' dB-Hz)'+
    ' | max '+x.gpsCno+' dB-Hz'+
    ' | HDOP '+f(x.gpsHdop,2)+
    ' | Antenne '+x.gpsAntenna+
    ' | PPS #'+x.gpsPpsCount+
    ' '+x.gpsPpsUs+' us'+
    ' | age '+x.gpsPpsAgeMs.toFixed(1)+' ms'+
    ' | NMEA '+x.nmeaOk+'/'+x.nmeaBad;

  document.getElementById('ocxo').textContent=f(x.ocxo10,3,' Hz');
  document.getElementById('ocxo2').textContent=
    (x.ocxoSignal ? 'SIGNAL OK' : 'KEIN SIGNAL')+
    ' | 1s '+f(x.ocxo1,0,' Hz')+
    ' | 100s '+n(x.ocxoPpm100,6,' ppm')+
    ' | 300s '+n(x.ocxoPpm300,6,' ppm')+
    ' | 1000s '+n(x.ocxoPpm1000,9,' ppm');

  document.getElementById('rtc').textContent=x.rtcDate+' '+x.rtcTime;
  document.getElementById('rtc2').textContent=
    '32k '+f(x.rtc3210,3,' Hz')+
    ' | 60s '+n(x.rtc32Ppm60,3,' ppm (ROH)')+
    ' | Aging '+x.rtcAgingOffset+
    ' | Gang live '+n(x.rtcLiveRatePpm,4,' ppm')+' ('+x.rtcLiveRateSpanSec+'s)'+
    ' | Gang kal '+n(x.rtcCalRatePpm,4,' ppm')+
    ' | Kal '+x.rtcCalAdjustments+' / '+x.rtcCalHours+'h von '+x.rtcCalTargetHours+'h'+
    ' | vorl. '+n(x.rtcCalProvisionalPpm,4,' ppm')+
    ' | letzter Reset '+x.rtcCalLastReset+
    ' | GPS/SQW/Temp/Phase/Gang/Sync/RTC '+[x.rtcCalResetGps,x.rtcCalResetSqw,x.rtcCalResetTemperature,x.rtcCalResetPhase,x.rtcCalResetRate,x.rtcCalResetSync,x.rtcCalResetRtcState].join('/')+
    ' | GPS-Resetdetails Fix/Sats/PPS/RMC/Stabil/Map/CNo/Ant '+[x.rtcCalGpsFailFix,x.rtcCalGpsFailSats,x.rtcCalGpsFailPps,x.rtcCalGpsFailRmc,x.rtcCalGpsFailStable,x.rtcCalGpsFailMap,x.rtcCalGpsFailCno,x.rtcCalGpsFailAnt].join('/');

  document.getElementById('ntp').textContent=x.ntpLocal;
  document.getElementById('ntp2').textContent=
    'L net '+f(x.localRttMs,3,' ms')+
    ' / raw '+f(x.localRawRttMs,3,' ms')+
    ' / proc '+f(x.localProcMs,3,' ms')+
    ' | Pool '+x.ntpPool;

  document.getElementById('lg').textContent=n(x.localGpsMs,3,' ms');
  document.getElementById('pg').textContent=n(x.poolGpsMs,3,' ms');
  document.getElementById('lp').textContent=n(x.localPoolMs,3,' ms');
  document.getElementById('rgp').textContent=n(x.rtcGpsPhaseMs,3,' ms');
  document.getElementById('rgd').textContent=n(x.rtcGpsDriftMs,3,' ms');
  document.getElementById('rl').textContent=n(x.rtcLocalMs,3,' ms');

  const bo=bestOcxo(x);
  gauge('gOcxo',bo[0],0.010,6,'ppm',bo[1],GAUGE_TAU_OCXO,Number.isFinite(bo[0]));
  gauge('gRtc',x.rtcLiveRatePpm,5.0,4,'ppm','Gang live '+x.rtcLiveRateSpanSec+' s',GAUGE_TAU_RTC,Number.isFinite(x.rtcLiveRatePpm));
  gauge('gEsp',x.espPpm,50.0,2,'ppm','Taktmodell',GAUGE_TAU_ESP,Number.isFinite(x.espPpm));
  gauge('gNtp',x.localGpsMs,20.0,2,'ms','lokal − GPS',GAUGE_TAU_NTP,x.localSampleAgeSec>=0&&Number.isFinite(x.localGpsMs));

  document.getElementById('store').textContent=
    'Messfenster '+x.histCount+'/1000 | OCXO gueltig (98% Mindestquote) 100/300/1000s: '+x.ocxoValid100+'/'+x.ocxoValid300+'/'+x.ocxoValid1000+' | CSV-Zeilen '+x.logRows;

  document.getElementById('store2').textContent=
    'Dateisystem '+(x.fsReady?'OK':'FEHLER')+
    ' | Log '+x.logBytes+' Bytes'+
    ' | Flash '+Math.round(x.fsUsed/1024)+' / '+Math.round(x.fsTotal/1024)+' kB'+
    ' | Langzeitmittel '+n(x.persistentPpmMean,9,' ppm')+
    ' | min '+n(x.persistentPpmMin,9,' ppm')+
    ' | max '+n(x.persistentPpmMax,9,' ppm');
  document.getElementById('store3').textContent=
    'Letzter Logeintrag '+(x.lastLogAgeSec<0?'noch keiner':x.lastLogAgeSec+'s')+
    ' | Schreibfehler '+x.logWriteFailures;
  document.getElementById('fsinit').style.display=x.fsReady?'none':'block';
  document.getElementById('phaseInfo').textContent=x.phase+
    ' | Laufzeit '+Math.floor(x.phaseElapsedSeconds/60)+' min'+
    ' | Mess-Snapshot '+(x.measureAgeMs===4294967295?'FEHLT':x.measureAgeMs+' ms alt');
  [['qbw',x.badWindowWarmup],['qbg',x.badWindowGpsWait],['qbr',x.badWindowRun],
   ['qow',x.ocxoRejectWarmup],['qog',x.ocxoRejectGpsWait],['qor',x.ocxoRejectRun],
   ['qfw',x.overflowWarmup],['qfg',x.overflowGpsWait],['qfr',x.overflowRun]]
  .forEach(([id,value])=>document.getElementById(id).textContent=value);
  document.getElementById('quality').textContent=
    'Core 1 Zeitmessung / Core 0 WLAN-Web-FS-NTP-TempIO | GPS-Referenz ungeeignet '+
    x.rejectedGpsReference+' | RTC-Logqueue ueberlaufen '+x.rtcEventsDropped+' | '+
    'NTP Stratum '+x.ntpStratumGps+' (GPS), PCNT-Snapshotfehler '+x.counterReadFailures+
    ' (Reads '+x.pcntReadAttempts+', Treiberfehler '+x.pcntDriverReadErrors+', Rohfehler Betrieb OC/32k '+x.pcntOcxoUnrecoverableRun+'/'+x.pcntRtc32UnrecoverableRun+' gesamt '+x.pcntOcxoUnrecoverable+'/'+x.pcntRtc32Unrecoverable+', Retries '+x.pcntRetryReads+', Read gerettet '+x.pcntRecoveredReads+', Sekunden ueberbrueckt '+x.pcntRecoveredSpans+')'+
    ', unrettbar '+x.counterInvalidWindows+
    ' | OCXO Overflow-Verdacht '+x.ocxoOverflowSuspects+
    ' (Bloecke a '+x.pcntOverflowLimit+'), PPS verworfen '+x.ocxoPpsRejects+
    ' | letzter OCXO-Fehler '+n(x.lastBadOcxoPpm,1,' ppm')+
    ' | PPS waehrend Flashaktivitaet '+x.flashGuardRejects+' (nicht verworfen) | PCNT/GPIO-ISR IRAM '+(x.pcntIramSafe?'JA':'NEIN')+'/'+(x.gpioIsrIram?'JA':'NEIN')+
    ' | Worker Core'+x.ntpWorkerCore+' NTP L/P '+x.ntpWorkerLocalQueries+'/'+x.ntpWorkerPoolQueries+
    ' (Abfragefehler '+x.ntpWorkerErrorsLocal+'/'+x.ntpWorkerErrorsPool+
    ', ohne GPS ausgelassen '+x.ntpWorkerSkippedGpsLocal+'/'+x.ntpWorkerSkippedGpsPool+')'+
    ' | NTP Protokollfehler L/P: '+x.localRejectProtocol+'/'+x.poolRejectProtocol+
    ' | NTP verworfene RTT L/P: '+x.localRejectDelay+'/'+x.poolRejectDelay+
    ' | letztes lokales RTT '+f(x.localLatestRawMs,1,' ms')+
    ' | akzeptierter Messwert Alter '+(x.localSampleAgeSec<0?'keiner':x.localSampleAgeSec+' s')+
    ' | WLAN RSSI '+x.wifiRssi+' dBm, Disconnects '+x.wifiDisconnects+' (Grund '+x.wifiLastReason+'), LostIP '+x.wifiLostIp+
    ' | Heap frei/min/groesster '+x.heapFree+'/'+x.heapMin+'/'+x.heapLargest+
    ' | Loop max Gap/Arbeit '+(x.loopMaxGapUs/1000).toFixed(1)+'/'+(x.loopMaxWorkUs/1000).toFixed(1)+' ms'+
    ' | AsyncWeb/LCD/Log max '+(x.webHandleMaxUs/1000).toFixed(1)+'/'+(x.lcdMaxUs/1000).toFixed(1)+'/'+(x.loggerMaxUs/1000).toFixed(1)+' ms'+
    ' | GPS-Lock-Aussetzer '+x.gpsUnlockEvents+' (aktuell '+x.gpsUnlockCurrentMs+' ms, letzter '+x.gpsUnlockLastMs+' ms, max '+x.gpsUnlockMaxMs+' ms, Grund '+gpsFailText(x.gpsUnlockCurrentMs?x.gpsUnlockCurrentReasonMask:x.gpsUnlockLastReasonMask)+')'+
    ' | NTP Timeout L/P '+x.ntpTimeoutLocal+'/'+x.ntpTimeoutPool+
    ', DNSfail '+x.ntpDnsFailLocal+'/'+x.ntpDnsFailPool+
    ', Sendfail '+x.ntpSendFailLocal+'/'+x.ntpSendFailPool+
    ', DNSmax '+(x.ntpDnsMaxLocalUs/1000).toFixed(1)+'/'+(x.ntpDnsMaxPoolUs/1000).toFixed(1)+' ms'+
    ', Querymax '+(x.ntpQueryMaxLocalUs/1000).toFixed(1)+'/'+(x.ntpQueryMaxPoolUs/1000).toFixed(1)+' ms';
 }catch(e){console.log(e);}
 finally{setTimeout(upd,1000);}
}
function trendStats(vals){
  if(vals.length<2)return null;
  const mean=vals.reduce((a,b)=>a+b,0)/vals.length;
  const variance=vals.reduce((a,b)=>a+(b-mean)*(b-mean),0)/vals.length;
  return {mean:mean,sigma:Math.sqrt(Math.max(0,variance)),n:vals.length};
}
function drawTrend(id,items,key,label,statLines=false){
  const c=document.getElementById(id),g=c.getContext('2d');
  g.clearRect(0,0,c.width,c.height);g.fillStyle='#141414';g.fillRect(0,0,c.width,c.height);
  const vals=items.map(x=>x[key]).filter(x=>Number.isFinite(x));
  g.fillStyle='#ccc';g.font='12px monospace';
  if(vals.length<2){g.fillText(label+' - warte auf Messwerte',12,24);if(statLines)document.getElementById('ocxoStats').textContent='warte auf mindestens 2 gültige Punkte';return;}
  const st=statLines?trendStats(vals):null;
  let lo=Math.min(...vals),hi=Math.max(...vals);
  if(st){lo=Math.min(lo,st.mean-st.sigma);hi=Math.max(hi,st.mean+st.sigma);}
  if(hi-lo<.00001){hi+=.001;lo-=.001}
  const pad=(hi-lo)*.12;lo-=pad;hi+=pad;
  g.fillText(label+'    '+lo.toFixed(6)+' .. '+hi.toFixed(6),12,18);
  g.strokeStyle='#333';
  for(let k=0;k<4;k++){let y=28+k*(c.height-48)/3;g.beginPath();g.moveTo(35,y);g.lineTo(c.width-8,y);g.stroke()}
  const yof=v=>28+(hi-v)/(hi-lo)*(c.height-48);
  if(st){
    const lines=[[st.mean-st.sigma,'−1σ'],[st.mean,'Mittel'],[st.mean+st.sigma,'+1σ']];
    lines.forEach((z,i)=>{const yy=yof(z[0]);g.save();g.setLineDash(i===1?[]:[6,4]);g.strokeStyle=i===1?'#ffcf66':'#a9a9a9';g.lineWidth=i===1?2:1;g.beginPath();g.moveTo(35,yy);g.lineTo(c.width-8,yy);g.stroke();g.restore();});
    document.getElementById('ocxoStats').textContent='Mittel '+n(st.mean,9,' ppm')+' | σ '+f(st.sigma,9,' ppm')+' | −1σ '+n(st.mean-st.sigma,9,' ppm')+' | +1σ '+n(st.mean+st.sigma,9,' ppm')+' | N='+st.n;
  }
  g.strokeStyle='#8bc4ff';g.lineWidth=2;g.beginPath();let started=false;
  items.forEach((x,i)=>{let v=x[key];if(!Number.isFinite(v)){started=false;return}
    let xx=35+i/Math.max(1,items.length-1)*(c.width-45),yy=yof(v);
    if(!started){g.moveTo(xx,yy);started=true}else g.lineTo(xx,yy)
  });g.stroke();
}
function sci(v){return Number.isFinite(v)?v.toExponential(3):'--'}
function drawAdev(data){
  const pts=(data.points||[]).filter(p=>Number.isFinite(p.adev)&&p.adev>0&&p.pairs>0);
  document.getElementById('adevInfo').textContent='gültige 1-s-Sekunden '+data.validSeconds+' | aktuelle zusammenhängende Serie '+data.currentRun+' s | verworfene/ungeeignete Sekunden '+data.rejectedSeconds;
  document.getElementById('adevTable').innerHTML=(data.points||[]).map(p=>'<tr><td>'+p.tau+' s</td><td>'+sci(p.adev)+'</td><td>'+p.pairs+'</td></tr>').join('');
  const c=document.getElementById('adevTrend'),g=c.getContext('2d');g.clearRect(0,0,c.width,c.height);g.fillStyle='#141414';g.fillRect(0,0,c.width,c.height);g.fillStyle='#ccc';g.font='12px monospace';
  if(pts.length<2){g.fillText('ADEV - warte auf mindestens zwei τ-Punkte',12,24);return;}
  const lx=pts.map(p=>Math.log10(p.tau)),ly=pts.map(p=>Math.log10(p.adev));let xmin=Math.min(...lx),xmax=Math.max(...lx),ymin=Math.min(...ly),ymax=Math.max(...ly);if(ymax-ymin<.2){ymax+=.1;ymin-=.1}
  const xp=x=>45+(Math.log10(x)-xmin)/(xmax-xmin)*(c.width-60),yp=y=>28+(ymax-Math.log10(y))/(ymax-ymin)*(c.height-55);
  g.strokeStyle='#333';for(let k=0;k<4;k++){let y=28+k*(c.height-55)/3;g.beginPath();g.moveTo(45,y);g.lineTo(c.width-15,y);g.stroke();}
  g.strokeStyle='#8bc4ff';g.lineWidth=2;g.beginPath();pts.forEach((p,i)=>{const x=xp(p.tau),y=yp(p.adev);if(i)g.lineTo(x,y);else g.moveTo(x,y)});g.stroke();
  pts.forEach(p=>{const x=xp(p.tau),y=yp(p.adev);g.fillStyle='#eee';g.beginPath();g.arc(x,y,3,0,Math.PI*2);g.fill();g.fillStyle='#aaa';g.fillText(p.tau+'s',Math.max(2,x-12),c.height-8);});
  g.fillStyle='#ccc';g.fillText('ADEV '+Math.pow(10,ymin).toExponential(1)+' .. '+Math.pow(10,ymax).toExponential(1),12,18);
}
async function fetchJsonTimed(url,timeoutMs){
  const ctl=new AbortController();
  const timer=setTimeout(()=>ctl.abort(),timeoutMs);
  try{
    const r=await fetch(url,{cache:'no-store',signal:ctl.signal});
    if(!r.ok)throw new Error(url+' HTTP '+r.status);
    return await r.json();
  }finally{clearTimeout(timer)}
}
async function trend(){
  try{
    const xs=await fetchJsonTimed('/trend',6000);
    drawTrend('ocxoTrend',xs,'p','OCXO 300s ppm',true);
    drawTrend('cnoTrend',xs,'c','GNSS max C/N0 dB-Hz',false);
  }catch(e){console.log(e)}
  finally{setTimeout(trend,10000)}
}
async function adev(){
  try{drawAdev(await fetchJsonTimed('/adev',6000))}
  catch(e){console.log(e)}
  finally{setTimeout(adev,10000)}
}
function hx(v){return '0x'+(Number(v)>>>0).toString(16).padStart(8,'0')}
function dd(a,b){return Number(b)-Number(a)}
function traceTxt(p,pre){
  const third=p[pre+'Third'];
  let t=hx(p[pre+'A'])+'/'+hx(p[pre+'B']);
  t+=' dAB '+dd(p[pre+'A'],p[pre+'B'])+' dt '+dd(p[pre+'AUs'],p[pre+'BUs'])+'us';
  t+=' rdA/B '+p[pre+'ADurUs']+'/'+p[pre+'BDurUs']+'us';
  if(third) t+='/'+hx(p[pre+'C'])+' dBC '+dd(p[pre+'B'],p[pre+'C'])+' dt '+dd(p[pre+'BUs'],p[pre+'CUs'])+'us rdC '+p[pre+'CDurUs']+'us';
  return t;
}
async function pcntdiag(){
  try{
    const p=await fetchJsonTimed('/pcntdiag',6000);
    let s='Erster Betriebsfehler: --';
    if(p.valid){
      const kinds=['--','SNAPSHOT','OCXO-INTERVALL','RTC32-INTERVALL'];
      const flash=p.flashAgeMs===4294967295?'--':p.flashAgeMs+'ms';
      const log=p.logAgeMs===4294967295?'--':p.logAgeMs+'ms';
      s='Erster Betriebsfehler: '+(kinds[p.kind]||p.kind)+' | t '+(p.uptimeMs/60000).toFixed(3)+'min | PPS '+p.ppsSeq+
        ' | OC '+(p.ocOk?'OK':'FEHLER')+' '+traceTxt(p,'oc')+
        ' | 32k '+(p.r32Ok?'OK':'FEHLER')+' '+traceTxt(p,'r32')+
        (p.intervalSpan?(' | Intervall '+p.intervalSpan+'s '+hx(p.intervalPrev)+'->'+hx(p.intervalCurr)+' '+n(p.intervalHz,3,' Hz')):'')+
        ' | Flash '+flash+' | Log '+log+' | RTC AG '+p.rtcAging+
        ' Kal '+(p.rtcCalElapsedMs/3600000).toFixed(2)+'/'+(p.rtcCalTargetMs/3600000).toFixed(2)+'h '+p.rtcCalLastReason;
    }
    document.getElementById('firstPcntErr').textContent=s;
  }catch(e){console.log(e)}
  finally{setTimeout(pcntdiag,10000)}
}
upd();trend();adev();pcntdiag();

</script></body></html>
)rawliteral";
}

String htmlEscape(String v) {
  v.replace("&","&amp;");
  v.replace("\"","&quot;");
  v.replace("<","&lt;");
  v.replace(">","&gt;");
  v.replace("'","&#39;");
  return v;
}
String buildPage() {
  String h = page();
  h.replace("%SSID%", htmlEscape(wifiSsid));
  h.replace("%LOCAL%", htmlEscape(localNtpHost));
  h.replace("%POOL%", htmlEscape(poolNtpHost));
  h.replace("%RESETTOKEN%", String(resetToken));
  h.replace("%FW_VERSION%", FW_VERSION);
  return h;
}

void initFilesystem() {
  fsReady=LittleFS.begin(false);
  if (!fsReady) { Serial.println("LittleFS: NICHT GEMOUNTET (keine Autoformatierung)"); return; }
  ensureLogHeader();
  loadPersistentStats();
  loadLogRows();
  Serial.printf("LittleFS OK: %u/%u Bytes, Zeilen=%lu\n",
       (unsigned)LittleFS.usedBytes(),(unsigned)LittleFS.totalBytes(),
       (unsigned long)logRows);
}

void setupWeb() {
  web.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    const int64_t t0=esp_timer_get_time();
    String body=buildPage();
    request->send(200,"text/html; charset=utf-8",body);
    noteWebCallback(t0);
  });
  web.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    const int64_t t0=esp_timer_get_time();
    String body=statusJson();
    request->send(200,"application/json",body);
    noteWebCallback(t0);
  });
  web.on("/trend", HTTP_GET, [](AsyncWebServerRequest *request){
    const int64_t t0=esp_timer_get_time();
    String body=trendJson();
    request->send(200,"application/json",body);
    noteWebCallback(t0);
  });
  web.on("/adev", HTTP_GET, [](AsyncWebServerRequest *request){
    const int64_t t0=esp_timer_get_time();
    String body=adevJson();
    request->send(200,"application/json",body);
    noteWebCallback(t0);
  });
  web.on("/pcntdiag", HTTP_GET, [](AsyncWebServerRequest *request){
    const int64_t t0=esp_timer_get_time();
    String body=pcntDiagJson();
    request->send(200,"application/json",body);
    noteWebCallback(t0);
  });

  web.on("/data/delete", HTTP_POST, [](AsyncWebServerRequest *request){
    const int64_t t0=esp_timer_get_time();
    if (!resetToken || asyncPostArg(request,"token") != String(resetToken) ||
        asyncPostArg(request,"confirm") != "LOESCHEN") {
      request->send(403,"text/plain","Ungueltige Bestaetigung. Seite neu laden.");
      noteWebCallback(t0); return;
    }
    if (asyncPostArg(request,"kind") != "all") {
      request->send(400,"text/plain","Nur Gesamtloeschung vorgesehen.");
      noteWebCallback(t0); return;
    }
    if (!queueWebMaintenance(WEB_MAINT_CLEAR_ALL)) {
      request->send(503,"text/plain","Wartungsaktion laeuft bereits. Spaeter erneut versuchen.");
      noteWebCallback(t0); return;
    }
    request->send(202,"text/plain","Loeschung gestartet. Seite nach einigen Sekunden neu laden.");
    noteWebCallback(t0);
  });

  web.on("/fs/initialize", HTTP_POST, [](AsyncWebServerRequest *request){
    const int64_t t0=esp_timer_get_time();
    if (fsReady) {
      request->send(409,"text/plain","Dateisystem ist bereits eingebunden. Kein Format erlaubt.");
      noteWebCallback(t0); return;
    }
    String code=asyncPostArg(request,"code");
    if (!fsFormatCode || code != String(fsFormatCode) ||
        asyncPostArg(request,"confirm") != "FORMATIEREN") {
      request->send(403,"text/plain","Falscher Code oder keine ausdrueckliche Bestaetigung.");
      noteWebCallback(t0); return;
    }
    if (!queueWebMaintenance(WEB_MAINT_INIT_FS)) {
      request->send(503,"text/plain","Wartungsaktion laeuft bereits. Spaeter erneut versuchen.");
      noteWebCallback(t0); return;
    }
    fsFormatCode=0; // einmalig ab akzeptierter Anforderung
    request->send(202,"text/plain","Dateisystem-Initialisierung gestartet. Seite spaeter neu laden.");
    noteWebCallback(t0);
  });

  web.on("/log.csv", HTTP_GET, [](AsyncWebServerRequest *request){
    const int64_t t0=esp_timer_get_time();
    if (!fsReady || !LittleFS.exists("/ocxo.csv")) {
      request->send(404,"text/plain","Noch kein Langzeitlog vorhanden.");
      noteWebCallback(t0); return;
    }
    AsyncWebServerResponse *response=request->beginResponse(
        LittleFS,"/ocxo.csv","text/csv; charset=utf-8");
    response->addHeader("Content-Disposition","attachment; filename=zeitkiste_ocxo.csv");
    request->send(response);
    noteWebCallback(t0);
  });

  // Lokales Wartungs-API: aktuell ohne Passwort! Nur in vertrauenswuerdigem
  // isoliertem Heimnetz betreiben; fuer fremde Netze HTTP-Auth/HTTPS ergaenzen.
  web.on("/save", HTTP_POST, [](AsyncWebServerRequest *request){
    const int64_t t0=esp_timer_get_time();
    if (!webMaintMutex || xSemaphoreTake(webMaintMutex,0)!=pdTRUE) {
      request->send(503,"text/plain","Wartungsaktion laeuft bereits. Spaeter erneut versuchen.");
      noteWebCallback(t0); return;
    }
    if (webMaintPending != WEB_MAINT_NONE || webMaintBusy) {
      xSemaphoreGive(webMaintMutex);
      request->send(503,"text/plain","Wartungsaktion laeuft bereits. Spaeter erneut versuchen.");
      noteWebCallback(t0); return;
    }
    webMaintSsid=asyncPostArg(request,"ssid");
    webMaintPass=asyncPostArg(request,"pass");
    webMaintUpdatePass=webMaintPass.length()>0;
    webMaintLocalNtp=asyncPostArg(request,"localntp");
    webMaintPoolNtp=asyncPostArg(request,"poolntp");
    webMaintPending=WEB_MAINT_SAVE_RESTART;
    webMaintNotBeforeMs=millis()+500U; // Antwort darf vor Neustart abfliessen
    xSemaphoreGive(webMaintMutex);
    request->send(202,"text/plain","Gespeichert wird gleich. Neustart folgt...");
    noteWebCallback(t0);
  });

  // Keine unsynchronisierte manuelle RTC-Setzung per HTTP-GET.
  web.on("/setrtc", HTTP_GET, [](AsyncWebServerRequest *request){
    const int64_t t0=esp_timer_get_time();
    request->send(410,"text/plain","RTC wird GPS/PPS-synchron automatisch gestellt.");
    noteWebCallback(t0);
  });

  web.onNotFound([](AsyncWebServerRequest *request){
    const int64_t t0=esp_timer_get_time();
    request->redirect("/");
    noteWebCallback(t0);
  });

  web.begin();
  Serial.printf("Async-Webserver gestartet, AsyncTCP Core=%d\n", CONFIG_ASYNC_TCP_RUNNING_CORE);
}

void webMaintenanceTick() {
  if (!webMaintMutex) return;
  WebMaintenanceAction action=WEB_MAINT_NONE;
  String ssid,pass,localHost,poolHost;
  bool updatePass=false;

  if (xSemaphoreTake(webMaintMutex,0)!=pdTRUE) return;
  if (webMaintPending!=WEB_MAINT_NONE &&
      (int32_t)(millis()-webMaintNotBeforeMs)>=0) {
    action=webMaintPending;
    if (action==WEB_MAINT_SAVE_RESTART) {
      ssid=webMaintSsid; pass=webMaintPass;
      localHost=webMaintLocalNtp; poolHost=webMaintPoolNtp;
      updatePass=webMaintUpdatePass;
    }
    webMaintPending=WEB_MAINT_NONE;
    webMaintBusy=true;
  }
  xSemaphoreGive(webMaintMutex);
  if (action==WEB_MAINT_NONE) return;

  if (action==WEB_MAINT_CLEAR_ALL) {
    const bool ok=clearAllMeasurements();
    if (ok) resetToken=(esp_random()|1UL);
    Serial.printf("Web-Wartung Messwerte loeschen: %s\n",ok?"OK":"FEHLER");
    if (xSemaphoreTake(webMaintMutex,portMAX_DELAY)==pdTRUE) {
      webMaintBusy=false; xSemaphoreGive(webMaintMutex);
    }
    return;
  }

  if (action==WEB_MAINT_INIT_FS) {
    markFlashActivity();
    const bool ok=LittleFS.format(); // betrifft NUR die Dateisystempartition
    markFlashActivity();
    if (ok) initFilesystem();
    if (fsReady) {
      fsReinitSucceeded=true;
      appendLongTermLog();
    }
    Serial.printf("Web-Wartung Dateisystem initialisieren: %s\n",fsReady?"OK":"FEHLER");
    if (xSemaphoreTake(webMaintMutex,portMAX_DELAY)==pdTRUE) {
      webMaintBusy=false; xSemaphoreGive(webMaintMutex);
    }
    return;
  }

  if (action==WEB_MAINT_SAVE_RESTART) {
    wifiSsid=ssid;
    if (updatePass) wifiPass=pass;
    if (ntpHostMutex) xSemaphoreTake(ntpHostMutex,portMAX_DELAY);
    localNtpHost=localHost;
    poolNtpHost=poolHost;
    if (ntpHostMutex) xSemaphoreGive(ntpHostMutex);
    markFlashActivity();
    prefs.begin("rtcdrift",false);
    prefs.putString("ssid",wifiSsid);
    prefs.putString("pass",wifiPass);
    prefs.putString("localntp",localNtpHost);
    prefs.putString("poolntp",poolNtpHost);
    prefs.end();
    markFlashActivity();
    Serial.println("Web-Wartung Konfiguration gespeichert, Neustart.");
    ESP.restart();
  }
}


// ------------------------------------------------------------
// LCD / Status
// ------------------------------------------------------------

// 1.2.8: Fuenf LCD-Seiten. Die Vergleichsseite ist als gedämpftes
// Nullinstrument ausgelegt und bleibt laenger sichtbar als die Detailseiten.
// Die vier Messgroessen haben bewusst eigene Skalen; die Balkenposition ist
// daher ein relativer Vergleich zur jeweils sinnvollen Toleranz.
constexpr double LCD_BAR_OCXO_PPM = 0.010;
constexpr double LCD_BAR_RTC_PPM  = 5.0;
constexpr double LCD_BAR_ESP_PPM  = 50.0;
constexpr double LCD_BAR_NTP_MS   = 20.0;
constexpr double LCD_DAMP_TAU_S   = 8.0;

struct LcdDampedValues {
  double ocxo=NAN, rtc=NAN, esp=NAN, ntp=NAN;
  uint32_t lastMs=0;
};
LcdDampedValues lcdDamped;

double lcdBestOcxoPpm(const MeasurementView &v) {
  if (!v.ocxoSignal) return NAN;
  if (v.validO1000 >= 980 && isfinite(v.ocxoPpm1000)) return v.ocxoPpm1000;
  if (v.validO300  >= 294 && isfinite(v.ocxoPpm300))  return v.ocxoPpm300;
  if (v.validO100  >= 98  && isfinite(v.ocxoPpm100))  return v.ocxoPpm100;
  return NAN;
}

void lcdDampOne(double input, double &state, double alpha) {
  if (!isfinite(input)) return;
  if (!isfinite(state)) state=input;
  else state += alpha*(input-state);
}

void updateLcdDamping(const MeasurementView &v, bool localNtpFresh) {
  const uint32_t now=millis();
  if (!lcdDamped.lastMs) lcdDamped.lastMs=now;
  double dt=(now-lcdDamped.lastMs)/1000.0;
  lcdDamped.lastMs=now;
  if (dt<0.001) dt=0.001;
  if (dt>2.0) dt=2.0;
  const double alpha=1.0-exp(-dt/LCD_DAMP_TAU_S);
  const double rtcAlpha=1.0-exp(-dt/20.0);
  lcdDampOne(lcdBestOcxoPpm(v),lcdDamped.ocxo,alpha);
  lcdDampOne(v.rtcLiveRatePpm,lcdDamped.rtc,rtcAlpha);
  lcdDampOne(v.espEstimatedPpm,lcdDamped.esp,alpha);
  if (localNtpFresh) lcdDampOne(v.localGpsMs,lcdDamped.ntp,alpha);
}

String lcdZeroBar(const char *label, double value, double fullScale, bool valid=true) {
  String s(label);
  while (s.length()<5) s += ' ';
  if (!valid || !isfinite(value) || !(fullScale>0.0)) {
    s += "-----?-----";
    return fit16(s);
  }
  double n=value/fullScale;
  if (n>1.0) n=1.0;
  if (n<-1.0) n=-1.0;
  int pos=(int)lround((n+1.0)*5.0); // 0..10, Nullpunkt exakt in der Mitte
  if (pos<0) pos=0; if (pos>10) pos=10;
  for (int i=0;i<11;i++) s += (i==pos) ? (char)0xFF : '|';
  return fit16(s);
}

String lcdPpm(double v, bool valid, int dec) {
  return valid && isfinite(v) ? signedVal(v,dec) : String("---");
}

void updateLcd() {
  static uint32_t last=0;
  if (!lcd || (uint32_t)(millis()-last)<500) return;
  last=millis();

  MeasurementView v; readMeasurement(v);
  const uint64_t nowUs=clockNowUs();
  const uint64_t rtcUs=v.haveRtc?v.rtcEpoch*1000000ULL:0;
  const bool localNtpFresh=ntpFresh(NTP_LOCAL) && isfinite(v.localGpsMs);
  const int64_t ppsAgeUs=v.ppsMono>0 ? esp_timer_get_time()-v.ppsMono : INT64_MAX;
  const bool ppsFresh=ppsAgeUs>=0 && ppsAgeUs<2000000LL;
  updateLcdDamping(v,localNtpFresh);

  // 40-s-Zyklus: Hauptseite 6 s, Vergleich 12 s, OCXO 8 s, RTC 8 s, System 6 s.
  const uint32_t phase=millis()%40000UL;
  uint8_t pageNo;
  if (phase<6000UL) pageNo=0;
  else if (phase<18000UL) pageNo=1;
  else if (phase<26000UL) pageNo=2;
  else if (phase<34000UL) pageNo=3;
  else pageNo=4;

  if (pageNo==0) {
    // Uhr und kompakter Gesamtstatus. GPS hat keine eigene Seite mehr.
    lcdDesiredLine(0,fmtDateShort(nowUs,true)+" "+fmtTime(nowUs,true,false));
    String gps=v.locked ? "GPS LOCK" : (v.gpsFix ? "GPS FIX " : "GPS --  ");
    lcdDesiredLine(1,gps+String(ppsFresh ? " PPS OK" : " PPS --"));
    lcdDesiredLine(2,"RTC "+fmtTime(rtcUs,true,false));
    String net = WiFi.status()==WL_CONNECTED ? "WLAN OK " : (apMode ? "WLAN AP " : "WLAN -- ");
    String ntpState="NTP --";
    if (ntpServerStarted && WiFi.status()==WL_CONNECTED) {
      const bool gpsNtp=!strcmp(v.clockMode,"GPS_LOCK") && v.locked;
      const bool holdNtp=!strcmp(v.clockMode,"HOLD_ESP") &&
                         v.uncertaintyMs<1000.0f &&
                         (uint32_t)(millis()-v.lastGpsLockMs)<86400000UL;
      if (gpsNtp) ntpState="NTP S2";
      else if (holdNtp) ntpState="NTP S3";
    }
    lcdDesiredLine(3,net+ntpState);

  } else if (pageNo==1) {
    // Vier gleiche Nullinstrumente, aber mit unterschiedlichen Vollskalen.
    lcdDesiredLine(0,lcdZeroBar("OFEN",lcdDamped.ocxo,LCD_BAR_OCXO_PPM,isfinite(lcdBestOcxoPpm(v))));
    lcdDesiredLine(1,lcdZeroBar("RTC", lcdDamped.rtc, LCD_BAR_RTC_PPM,isfinite(v.rtcLiveRatePpm)));
    lcdDesiredLine(2,lcdZeroBar("ESP", lcdDamped.esp, LCD_BAR_ESP_PPM,isfinite(v.espEstimatedPpm)));
    lcdDesiredLine(3,lcdZeroBar("NTP", lcdDamped.ntp, LCD_BAR_NTP_MS,localNtpFresh));

  } else if (pageNo==2) {
    const bool ok100=v.validO100>=98 && isfinite(v.ocxoPpm100);
    const bool ok300=v.validO300>=294 && isfinite(v.ocxoPpm300);
    const bool ok1000=v.validO1000>=980 && isfinite(v.ocxoPpm1000);
    lcdDesiredLine(0,v.ocxoSignal ? "OCXO 10MHz OK" : "OCXO KEIN SIGNAL");
    lcdDesiredLine(1,"100 "+lcdPpm(v.ocxoPpm100,ok100,5)+"ppm");
    lcdDesiredLine(2,"300 "+lcdPpm(v.ocxoPpm300,ok300,5)+"ppm");
    lcdDesiredLine(3,"1k  "+lcdPpm(v.ocxoPpm1000,ok1000,6)+"ppm");

  } else if (pageNo==3) {
    lcdDesiredLine(0,"RTC AG "+String(v.rtcAgingOffset)+" KAL"+String(v.rtcAgingAdjustments));
    lcdDesiredLine(1,"LIVE "+lcdPpm(v.rtcLiveRatePpm,isfinite(v.rtcLiveRatePpm),4)+"ppm");
    lcdDesiredLine(2,"PHASE "+(isfinite(v.rtcGpsPhaseMs)?signedVal(v.rtcGpsPhaseMs,2):String("---"))+"ms");
    double calHours=v.rtcCalStartMs ? (millis()-v.rtcCalStartMs)/3600000.0 : 0.0;
    const double targetHours=v.rtcCalTargetMs/3600000.0;
    lcdDesiredLine(3,"KAL "+String(calHours,2)+"/"+String(targetHours,2)+"h");

  } else {
    String wlan;
    if (WiFi.status()==WL_CONNECTED)
      wlan="W "+String(WiFi.RSSI())+" D"+String((unsigned long)wifiDisconnectEvents);
    else wlan=apMode ? "WLAN AP D"+String((unsigned long)wifiDisconnectEvents)
                     : "WLAN -- D"+String((unsigned long)wifiDisconnectEvents);
    lcdDesiredLine(0,wlan+String(fsReady?" FSOK":" FSER"));
    lcdDesiredLine(1,localNtpFresh ? "NTP L "+signedVal(v.localGpsMs,1)+"ms" :
             "NTP T"+String((unsigned long)ntpTimeouts[NTP_LOCAL])+" R"+String((unsigned long)ntpRejectedDelay[NTP_LOCAL]));
    lcdDesiredLine(2,"ESP "+String(v.chipTempC,1)+"C "+signedVal(v.espEstimatedPpm,1));
    lcdDesiredLine(3,"GPS "+String(v.gpsSats)+"/"+String(v.gpsVisible)+(ppsFresh?" PPS OK":" PPS --"));
    if (!fsReady && fsFormatCode) {
      // Der sechsstellige Code bleibt trotz neuer Seiten direkt am LCD erreichbar.
      lcdDesiredLine(1,"FS INIT CODE:");
      lcdDesiredLine(2,String(fsFormatCode));
      lcdDesiredLine(3,"NICHT AUTOFORMAT");
    }
  }
}

struct RtcEvent { uint8_t kind; uint64_t epoch; int16_t aging; double value; };
QueueHandle_t rtcEventQueue = nullptr;
volatile uint32_t rtcEventsDropped=0;
void enqueueRtcEvent(uint8_t kind,uint64_t epoch,int16_t aging,double value) {
  if (!rtcEventQueue) return;
  RtcEvent e={kind,epoch,aging,value};
  if (xQueueSend(rtcEventQueue,&e,0)!=pdTRUE) rtcEventsDropped++;
}
void drainRtcEvents() {
  if (!rtcEventQueue) return;
  RtcEvent e;
  // Queue kurz und bounded leeren, um Core0 nicht zu blockieren.
  for (int i=0;i<8 && xQueueReceive(rtcEventQueue,&e,0)==pdTRUE;i++) {
    if (!fsReady) continue;
    markFlashActivity(); rotateEventsIfNeeded();
    File f=LittleFS.open("/events.csv","a");
    if (f) {
      if (e.kind==1) f.printf("%llu,RTC_AGING,%d,%.6f\n",
           (unsigned long long)e.epoch,(int)e.aging,e.value);
      else if (e.kind==2) f.printf("%llu,RTC_SYNC,%.1f\n",
           (unsigned long long)e.epoch,e.value);
      f.close();
    }
    markFlashActivity();
  }
}

// ------------------------------------------------------------
// Setup / Loop
// ------------------------------------------------------------

// Arduino setup()/loop() fest auf Core0 (sdkconfig).
// Core1 ist der zeitkritische Messkern: GPS/PPS, PCNT, SQW, Zeitanker und
// reine Rechenlogik. Keine Web-/FS-/WLAN-Events und kein Temperatursensor-I/O.
// Core0: AsyncTCP/Web, LCD, LittleFS, WLAN, NTP, Temperatur-Hardware und Hintergrund-I/O.
TaskHandle_t measurementTaskHandle=nullptr;
void measurementTask(void *arg) {
  (void)arg;
  measurementStartedMs=millis();
  GPS.setRxBufferSize(2048);
  GPS.begin(9600,SERIAL_8N1,PIN_GPS_RX,PIN_GPS_TX);
  bool a=setupCounter(ocxoUnit,ocxoChan,PIN_OCXO);
  bool b=setupCounter(rtc32Unit,rtc32Chan,PIN_RTC_32K);
  counterOCOK=a; counter32OK=b;
  attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS),onGpsPps,RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RTC_SQW),onRtcSqw,FALLING);
  Serial.printf("PCNT initialisiert Core %d: OCXO=%s RTC32=%s, ISR-IRAM=%s\n",
      xPortGetCoreID(),a?"OK":"FEHLER",b?"OK":"FEHLER",
      CONFIG_PCNT_ISR_IRAM_SAFE?"JA":"NEIN");
  if (!CONFIG_PCNT_ISR_IRAM_SAFE)
    Serial.println("WARNUNG: PCNT nicht IRAM-SAFE. Flash kann Ueberlaeufe verlieren.");
  for (;;) {
    pollGps();
    updateRunPhase();
    processPps(); processSqw(); gpsLockDiagTick(); rtcCalTick();
    clockAnchorTick(); rtcSyncTick(); tempTick();
    publishNtpTimeState(); updateOffsets(); trendTick();
    const uint32_t pending=__atomic_load_n(&clearMeasureRequest,__ATOMIC_ACQUIRE);
    if (pending!=__atomic_load_n(&clearMeasureAck,__ATOMIC_RELAXED)) {
      resetMeasurementRam();
      __atomic_store_n(&clearMeasureAck,pending,__ATOMIC_RELEASE);
    }
    publishMeasurement();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

TaskHandle_t ntpWorkerHandle = nullptr;
void ntpWorker(void *arg) {
  (void)arg;
#if ZEITKISTE_WLAN_AN
  uint32_t lastLocal=0, lastPool=0;
  for (;;) {
    if (WiFi.status()==WL_CONNECTED) {
      uint32_t now=millis();
      if (!lastLocal || (uint32_t)(now-lastLocal)>=LOCAL_NTP_INTERVAL) {
        lastLocal=now;
        ntpWorkerLocalQueries++;
        if (!ntpGpsReady()) ntpWorkerSkippedNoGps[NTP_LOCAL]++;
        else {
          if (ntpHostMutex) xSemaphoreTake(ntpHostMutex,portMAX_DELAY);
          String host=localNtpHost;
          if (ntpHostMutex) xSemaphoreGive(ntpHostMutex);
          const int64_t q0=esp_timer_get_time();
          const bool ok=ntpQuery(host,NTP_LOCAL);
          const int64_t q1=esp_timer_get_time();
          const uint32_t qus=(uint32_t)(q1>q0 ? q1-q0 : 0);
          if (qus>ntpQueryMaxUs[NTP_LOCAL]) ntpQueryMaxUs[NTP_LOCAL]=qus;
          if (!ok) ntpWorkerFailedQueries[NTP_LOCAL]++;
        }
      }
      now=millis();
      if (!lastPool || (uint32_t)(now-lastPool)>=POOL_NTP_INTERVAL) {
        lastPool=now;
        ntpWorkerPoolQueries++;
        if (!ntpGpsReady()) ntpWorkerSkippedNoGps[NTP_POOL]++;
        else {
          if (ntpHostMutex) xSemaphoreTake(ntpHostMutex,portMAX_DELAY);
          String host=poolNtpHost;
          if (ntpHostMutex) xSemaphoreGive(ntpHostMutex);
          const int64_t q0=esp_timer_get_time();
          const bool ok=ntpQuery(host,NTP_POOL);
          const int64_t q1=esp_timer_get_time();
          const uint32_t qus=(uint32_t)(q1>q0 ? q1-q0 : 0);
          if (qus>ntpQueryMaxUs[NTP_POOL]) ntpQueryMaxUs[NTP_POOL]=qus;
          if (!ok) ntpWorkerFailedQueries[NTP_POOL]++;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
#else
  vTaskDelete(nullptr);
#endif
}

void setup() {
  Serial.begin(115200);
  delay(700);
  Serial.printf("Zeitkiste loopTask: Core=%d, Stack=%lu Bytes, frei(min)=%lu Bytes\n",
                xPortGetCoreID(), (unsigned long)getArduinoLoopTaskStackSize(),
                (unsigned long)uxTaskGetStackHighWaterMark(nullptr));
  Serial.printf("Coreplan: loop=%d event=%d udp=%d, GPIO-ISR-IRAM=%s, PCNT-IRAM=%s\n",
                ARDUINO_RUNNING_CORE, ARDUINO_EVENT_RUNNING_CORE, CONFIG_ARDUINO_UDP_RUNNING_CORE,
                CONFIG_ARDUINO_ISR_IRAM ? "JA" : "NEIN",
                CONFIG_PCNT_ISR_IRAM_SAFE ? "JA" : "NEIN");

  setenv("TZ", TZ_INFO, 1);
  tzset();

  fsFormatCode=100000UL+(esp_random()%900000UL);
  resetToken=(esp_random()|1UL);
  initFilesystem();
  if (!fsReady) {
    Serial.printf("FS-Formatcode: %06lu. Daten in der FS-Partition wuerden geloescht!\n",
                  (unsigned long)fsFormatCode);
  }


  i2cBusMutex=xSemaphoreCreateRecursiveMutex();
  if (!i2cBusMutex) Serial.println("FATAL: I2C Mutex kann nicht erzeugt werden");
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  for (uint8_t a=1; a<127; ++a) {
    if (!i2cPresent(a)) continue;
    Serial.printf("I2C 0x%02X\n",a);

    if (a == DS3231_ADDR) rtcPresent = true;

    if (!lcdAddr &&
        ((a>=0x20 && a<=0x27) || (a>=0x38 && a<=0x3F))) {
      lcdAddr = a;
    }
  }

  if (rtcPresent) {
    rtcConfigure();
    uint8_t ageRaw=0;
    rtcAgingReadable=rtcReadReg(0x10,ageRaw);
    if (rtcAgingReadable) rtcAgingOffset=(int8_t)ageRaw;
    uint64_t e = 0;
    if (!rtcOsf && rtcReadEpoch(e)) {
      rtcBootEpoch = e; rtcBootMonoUs = esp_timer_get_time();
      rtcBootValid = true;
    }
  }
  tempInit();

  if (lcdAddr) {
    lcd = new LiquidCrystal_I2C(lcdAddr,16,4);
    lcd->init();
    lcd->backlight();
    lcdLine(0,String("Zeitkiste v")+FW_VERSION);
    lcdLine(1,"GPS OCXO RTC");
    lcdLine(2,"OCXO GPIO9");
    lcdLine(3,"startet...");
  }

  pinMode(PIN_RTC_SQW, INPUT_PULLUP);
  pinMode(PIN_RTC_32K, INPUT_PULLUP); // externer Pullup fuer sauberes 32 kHz empfohlen
  pinMode(PIN_GPS_PPS, INPUT);
  pinMode(PIN_OCXO, INPUT);

  // GPS-UART wird erst im Messtask auf Core1 installiert.
  loadConfig();

#if ZEITKISTE_WLAN_AN
  webMaintMutex=xSemaphoreCreateMutex();
  if (!webMaintMutex) Serial.println("WARNUNG: Web-Wartungsmutex fehlt");
  WiFi.onEvent(onWiFiEvent);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  if (!connectWifi()) startAp();
  setupWeb();
  udp.begin(2390);
  ntpServerStarted = ntpServer.begin(123);
  Serial.printf("NTP-Server UDP123: %s\n", ntpServerStarted ? "OK" : "FEHLER");
#else
  // WLAN-Radio ausschalten, keinen STA- oder AP-Modus starten.
  // Kein Webserver, kein DNS, kein NTP. GPS, RTC, PCNT und Log laufen weiter.
  WiFi.mode(WIFI_OFF);
  ntpValid[NTP_LOCAL] = false;
  ntpValid[NTP_POOL] = false;
  Serial.println("*** WLAN RADIO AUS: kein AP, kein NTP, kein Webserver ***");
  Serial.println("*** GPS / BeiDou / OCXO / RTC LCD alle 5s; Serial 115200 Baud ***");
#endif

#if ZEITKISTE_WLAN_AN
  const int loopCore=xPortGetCoreID();
  ntpWorkerAssignedCore=0;
  BaseType_t tasked=xTaskCreatePinnedToCore(ntpWorker,"zk_ntp_worker",8192,
                                    nullptr,1,&ntpWorkerHandle,ntpWorkerAssignedCore);
  if (tasked!=pdPASS) { Serial.println("NTP Worker Start FEHLER"); ntpWorkerHandle=nullptr; }
  else Serial.printf("NTP Worker auf Core %d gestartet (Arduino auf %d)\n",ntpWorkerAssignedCore,loopCore);
#endif
  ntpHostMutex=xSemaphoreCreateMutex();
  if (!ntpHostMutex) Serial.println("WARNUNG: NTP-Hostmutex fehlt");
  rtcEventQueue=xQueueCreate(16,sizeof(RtcEvent));
  if (!rtcEventQueue) Serial.println("WARNUNG: RTC-Ereignisqueue nicht verfuegbar");
  BaseType_t mt=xTaskCreatePinnedToCore(measurementTask,"zk_measure",12288,
                                       nullptr,3,&measurementTaskHandle,1);
  if (mt!=pdPASS) {
    Serial.println("FATAL: Mess-Task auf Core1 konnte nicht gestartet werden");
    // Ohne Mess-Task keine verwertbare Frequenzmessung.
  }
  Serial.printf("Zeitkiste %s gestartet. GPIO GPS 1/2/4, I2C 5/6, RTC 7/8, OCXO 9\n",FW_VERSION);
  Serial.println("RTC: GPS/PPS-Autosync nach 30 stabilen UTC-Sekunden; RTC speichert UTC.");
  Serial.println("WICHTIG: GNSS PPS-Zuordnung und PCNT 10MHz Genauigkeit gegen Referenz testen.");
}

void loop() {
  static uint32_t lastWifiRetry=0;
  static int64_t lastLoopEnterUs=0;
  const int64_t loopEnterUs=esp_timer_get_time();
  if (lastLoopEnterUs) {
    const uint32_t gap=(uint32_t)(loopEnterUs>lastLoopEnterUs ? loopEnterUs-lastLoopEnterUs : 0);
    if (gap>loopMaxGapUs) loopMaxGapUs=gap;
    if (gap>50000U) loopGapOver50ms++;
    if (gap>200000U) loopGapOver200ms++;
  }
  lastLoopEnterUs=loopEnterUs;
#if ZEITKISTE_WLAN_AN
  // ESPAsyncWebServer braucht kein handleClient(); HTTP laeuft im auf Core0
  // gepinnten AsyncTCP-Task. Wartungsaktionen werden hier sicher abgearbeitet.
  webMaintenanceTick();
  if (apMode) dns.processNextRequest();
  if (WiFi.status()==WL_CONNECTED && !ntpServerStarted)
    ntpServerStarted=ntpServer.begin(123);
  ntpServerTick();
  if (WiFi.status()!=WL_CONNECTED && (uint32_t)(millis()-lastWifiRetry)>30000) {
    lastWifiRetry=millis();
    if (!apMode) startAp();
    if (!wifiSsid.isEmpty()) WiFi.begin(wifiSsid.c_str(),wifiPass.c_str());
  }
#endif
  // Flash-Zugriffe, LCD und Temperatursensor-I/O im loop auf Core0.
  // HTTP-Callbacks laufen ebenfalls auf Core0, aber im AsyncTCP-Task.
  temperatureIoTick();
  drainRtcEvents();
  int64_t tlog=esp_timer_get_time();
  loggerTick();
  uint32_t logUs=(uint32_t)((esp_timer_get_time()>tlog) ? esp_timer_get_time()-tlog : 0);
  if (logUs>loggerMaxUs) loggerMaxUs=logUs;
  if (lastTempSaveMs==0 || (uint32_t)(millis()-lastTempSaveMs)>3600000UL) {
    // Temperatureinsatz nicht kurz nach Start zwangsweise schreiben:
    MeasurementView v; readMeasurement(v);
    if (v.tempLearningCount && v.sampleMs) tempSave();
  }
  int64_t tlcd=esp_timer_get_time();
  updateLcd();   // nur Sollbild im RAM berechnen
  lcdService();  // maximal einen kleinen Dirty-Block ueber I2C schreiben
  uint32_t lcdUs=(uint32_t)((esp_timer_get_time()>tlcd) ? esp_timer_get_time()-tlcd : 0);
  if (lcdUs>lcdMaxUs) lcdMaxUs=lcdUs;
  const int64_t loopEndUs=esp_timer_get_time();
  const uint32_t workUs=(uint32_t)(loopEndUs>loopEnterUs ? loopEndUs-loopEnterUs : 0);
  if (workUs>loopMaxWorkUs) loopMaxWorkUs=workUs;
  // Stack-Reserve nach echten HTTP-Anfragen verfolgen, damit der Fehler
  // nicht nur durch eine grosse Reserve verdeckt wird.
  static uint32_t lastLoopStackLogMs = 0;
  if ((uint32_t)(millis()-lastLoopStackLogMs)>=30000U) {
    lastLoopStackLogMs=millis();
    Serial.printf("loopTask: Core=%d, Stackreserve min=%lu Bytes\n",
                  xPortGetCoreID(), (unsigned long)uxTaskGetStackHighWaterMark(nullptr));
  }
  delay(1);
}
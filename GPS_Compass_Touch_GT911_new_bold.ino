/*
  ============================================================
  Projekt: ESP32-S3 Touch LCD 4" GPS / RS485 / Kompass
  Autor:   Peter Schulte
  Plattform: Waveshare ESP32-S3-Touch-LCD-4
  Core:    2.0.17, GFX 1.3.7
  Touch:   GT911 (I2C 0x5D auf GPIO15/7), IO-Power/Reset ueber
           CH32V003-Mikrocontroller (I2C 0x24, SELBER Bus 15/7!)
  FW:      1.08 - ENDGUELTIGE KORREKTUR: Diese Board-Revision nutzt
           einen CH32V003-Mikrocontroller statt eines einfachen
           Register-Expanders (TCA9554/CH422G). Echter Treiber
           WS_CH32_IO.h/.cpp liegt als separate Datei im Sketch-
           Ordner. Kein separater Expander-Bus noetig - alles auf
           GPIO15/7, Adresse 0x24, mit Register 0x02 (Richtung)
           und 0x03 (Ausgabe).
  ============================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include <WiFi.h>
#include <Arduino_GFX_Library.h>
#include "FreeSansBold18pt7b.h"
#include "FreeSansBold12pt7b.h"
#include "WS_CH32_IO.h"
#include "TouchDrvGT911.hpp"
TouchDrvGT911 touchLib;
static bool touchLibReady = false;

// ===================== GFX (Pointer - in setup erstellt) =====================
Arduino_DataBus        *bus      = NULL;
Arduino_ESP32RGBPanel  *rgbpanel = NULL;
Arduino_RGB_Display    *gfx      = NULL;

// ===================== GT911 Touch =====================
#define GT911_ADDR       0x5D
#define GT911_INT_PIN    4

enum ViewMode { VIEW_NORMAL, VIEW_COMPASS_FULL, VIEW_WIND_FULL, VIEW_WIND_STATS };
static ViewMode viewMode = VIEW_NORMAL;
static bool     lastTouchState    = false;
static uint32_t lastTouchMs       = 0;
static const uint32_t TOUCH_DEBOUNCE_MS = 400;
bool uiStaticDrawn = false;   // Vorab deklariert, da handleTouch() weiter oben
                               // im Sketch steht als die urspruengliche Stelle

bool i2cDevicePresent(uint8_t addr){Wire.beginTransmission(addr);return Wire.endTransmission()==0;}

// Bringt Touch (und Display-Power) ueber den echten CH32V003-Treiber
// aus dem Reset und wartet aktiv, bis der GT911 auf I2C antwortet.
static bool gt911_init(uint32_t timeoutMs = 1000) {
  // Aktive INT-Pin-Steuerung waehrend des Resets (empirisch bestaetigt
  // notwendig): der INT-Pegel waehrend der TOUCH_RST-Freigabe (die
  // innerhalb von WS_CH32_IO::begin() passiert) entscheidet beim GT911
  // mit ueber die korrekte interne Kalibrierung. Ohne das antwortet der
  // Chip zwar auf I2C, erkennt aber keine echten Beruehrungen.
  pinMode(GT911_INT_PIN, OUTPUT);
  digitalWrite(GT911_INT_PIN, HIGH);
  delay(1);

  bool chipOK = WS_CH32_IO::begin(Wire, 15, 7, 400000, &Serial);
  Serial.println(chipOK ? "[TOUCH] CH32V003 IO-Chip initialisiert"
                         : "[TOUCH] WARNUNG: CH32V003 IO-Chip antwortet nicht!");

  pinMode(GT911_INT_PIN, INPUT);   // INT erst jetzt freigeben
  delay(50);

  uint32_t t0 = millis();
  bool found = false;
  while (millis() - t0 < timeoutMs) {
    if (i2cDevicePresent(GT911_ADDR)) { found = true; break; }
    delay(20);
  }

  Serial.println(found ? "[TOUCH] GT911 antwortet, ready"
                        : "[TOUCH] WARNUNG: GT911 antwortet nicht!");

  if (found) {
    touchLibReady = touchLib.begin(Wire, GT911_ADDR, 15, 7);
    Serial.println(touchLibReady ? "[TOUCH] SensorLib bereit"
                                  : "[TOUCH] WARNUNG: SensorLib begin() fehlgeschlagen");
  }

  return found;

}

static uint16_t lastTouchX = 0, lastTouchY = 0;

static bool touchRead() {
  if (!touchLibReady) return false;

  TouchPoints pts = touchLib.getTouchPoints();
  if (!pts.hasPoints()) return false;

  const auto &p = pts.getPoint(0);
  lastTouchX = p.x;
  lastTouchY = p.y;
  return true;
}

static void handleTouch() {
  bool touched = touchRead();
  if (touched && !lastTouchState) {
    uint32_t now = millis();
    if (now - lastTouchMs > TOUCH_DEBOUNCE_MS) {
      // Zyklisch durchschalten: Normal -> Kompass-Vollbild -> Wind-Vollbild -> Windstatistik -> Normal
      if (viewMode == VIEW_NORMAL) viewMode = VIEW_COMPASS_FULL;
      else if (viewMode == VIEW_COMPASS_FULL) viewMode = VIEW_WIND_FULL;
      else if (viewMode == VIEW_WIND_FULL) viewMode = VIEW_WIND_STATS;
      else viewMode = VIEW_NORMAL;

      lastTouchMs = now;
      gfx->fillScreen(0x0000);
      uiStaticDrawn = false;   // Bildschirm wurde geloescht - Rahmen der
                               // Normalansicht muessen beim naechsten
                               // Zurueckschalten neu gezeichnet werden
    }
  }
  lastTouchState = touched;
}

// ===================== GPS =====================
TinyGPSPlus gps;
HardwareSerial GPSRS485Serial(1);
volatile uint32_t gpsByteCount = 0;
uint32_t lastGpsByteMs = 0;
static const uint32_t GPS_SIGNAL_TIMEOUT_MS = 3000;
static const char* FW_VERSION = "1.09";
static uint32_t gpsActiveBaud = 115200;

// ===================== WLAN =====================
#include <WiFiUdp.h>
const char* WIFI_SSID = "Troll32_GPS";
// Kein WiFi Passwort - offener Access Point
WiFiServer nmeaServer(10110);
WiFiClient nmeaClient;

// Windsensor: separates ESP32-Projekt, sendet NMEA0183 MWV per UDP-Broadcast.
// Unser Board verbindet sich zusaetzlich als Client (WIFI_AP_STA), waehrend
// der eigene Access Point fuer das M5Tough-Display weiterlaeuft.
const char* WIND_WIFI_SSID = "Ylvi_wind";
// Offenes Netzwerk, kein Passwort
static const uint16_t WIND_UDP_PORT = 10110;
WiFiUDP windUdp;

static float    windAngleDeg = 0.0f;   // 0-359, wie vom Sensor gemeldet (meist relativ zum Bug)
static float    windSpeedKn  = 0.0f;
static bool     windDataValid = false;
static uint32_t lastWindMs    = 0;
static const uint32_t WIND_SIGNAL_TIMEOUT_MS = 5000;

// ---- Windlupe: zeigt die Pendelbreite (Min/Max) des Windwinkels ueber
// die letzten ca. 2 Minuten als farbigen Bogen auf der Windrose an ----
// Gleichzeitig Basis fuer die Windstatistik-Ansicht (Geschwindigkeitsverlauf)
#define WIND_LUPE_SAMPLES 120   // ca. 2 Minuten bei 1 Sample/Sekunde
static float    windLupeBuf[WIND_LUPE_SAMPLES];    // Winkel-Verlauf
static float    windSpeedBuf[WIND_LUPE_SAMPLES];   // Geschwindigkeits-Verlauf
static int      windLupeCount = 0;
static int      windLupeIdx = 0;

static void windSampleAdd(float angle, float speedKn) {
  windLupeBuf[windLupeIdx] = angle;
  windSpeedBuf[windLupeIdx] = speedKn;
  windLupeIdx = (windLupeIdx + 1) % WIND_LUPE_SAMPLES;
  if (windLupeCount < WIND_LUPE_SAMPLES) windLupeCount++;
}

// Ermittelt Min/Max relativ zum aktuellen Winkel (0/360-Wraparound-sicher,
// durch "Abwickeln" jedes Samples auf die kuerzeste Distanz zum aktuellen
// Winkel)
static void windLupeGetRange(float currentAngle, float &minA, float &maxA) {
  minA = currentAngle; maxA = currentAngle;
  for (int i = 0; i < windLupeCount; i++) {
    float d = windLupeBuf[i] - currentAngle;
    while (d > 180)  d -= 360;
    while (d < -180) d += 360;
    float unwrapped = currentAngle + d;
    if (unwrapped < minA) minA = unwrapped;
    if (unwrapped > maxA) maxA = unwrapped;
  }
}

// Ermittelt Min/Durchschnitt/Max der Geschwindigkeit aus dem Verlauf
static void windSpeedStats(float &minKn, float &avgKn, float &maxKn) {
  if (windLupeCount == 0) { minKn = avgKn = maxKn = 0; return; }
  minKn = windSpeedBuf[0]; maxKn = windSpeedBuf[0];
  float sum = 0;
  for (int i = 0; i < windLupeCount; i++) {
    float v = windSpeedBuf[i];
    if (v < minKn) minKn = v;
    if (v > maxKn) maxKn = v;
    sum += v;
  }
  avgKn = sum / windLupeCount;
}

// ---- TEMPORÄR zum Testen ohne echten Windsensor ----
// Auf 'false' setzen (oder die Zeile loeschen), sobald der echte
// Sensor angeschlossen/erreichbar ist - dann kommen echte UDP-Daten.
#define WIND_SIMULATE true

#if WIND_SIMULATE
static void simulateWindData() {
  // Windwinkel pendelt nur noch um +/-30 Grad um einen festen Mittelwert
  // (statt voller Umdrehung) - realistischer fuer die Windlupen-Anzeige
  const float centerAngle = 45.0f;   // Mittelwert, um den gependelt wird
  windAngleDeg  = wrap360(centerAngle + 30.0f * sinf(millis() / 4000.0f));
  windSpeedKn   = 8.0f + 4.0f * sinf(millis() / 3000.0f);
  windDataValid = true;
  lastWindMs    = millis();
}
#endif


// Parst einen NMEA0183 $--MWV-Satz: $WIMWV,Winkel,Referenz(R/T),Speed,Einheit(N/M/K/S),Status(A/V)*CS
static void parseMWV(const char* line) {
  const char* p = strchr(line, ',');
  if (!p) return;
  float angle = atof(p + 1);

  p = strchr(p + 1, ',');
  if (!p) return;
  // Referenz R/T - aktuell nicht unterschieden, beides als "Windwinkel" behandelt
  p = strchr(p + 1, ',');
  if (!p) return;
  float speed = atof(p + 1);

  p = strchr(p + 1, ',');
  if (!p) return;
  char unit = *(p + 1);

  p = strchr(p + 1, ',');
  bool valid = p && *(p + 1) == 'A';

  if (!valid) { windDataValid = false; return; }

  // Geschwindigkeit auf Knoten umrechnen
  float speedKn = speed;
  if (unit == 'K') speedKn = speed / 1.852f;        // km/h -> kn
  else if (unit == 'M') speedKn = speed * 1.94384f;  // m/s -> kn
  else if (unit == 'S') speedKn = speed * 0.868976f; // mph -> kn
  // 'N' ist bereits Knoten

  windAngleDeg  = wrap360(angle);
  windSpeedKn   = speedKn;
  windDataValid = true;
  lastWindMs    = millis();
}

// Prueft auf neue UDP-Pakete vom Windsensor und wertet MWV-Saetze aus.
// Setzt windDataValid=false, falls zu lange nichts mehr ankam.
static void updateWindFromUDP() {
#if WIND_SIMULATE
  simulateWindData();
  return;   // echten UDP-Empfang ueberspringen, solange simuliert wird
#endif
  int packetSize = windUdp.parsePacket();
  if (packetSize > 0) {
    char buf[128];
    int len = windUdp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';
      // Paket kann mehrere Zeilen/Saetze enthalten - alle mit MWV pruefen
      char* line = strtok(buf, "\r\n");
      while (line) {
        if (line[0] == '$' && strstr(line, "MWV")) {
          parseMWV(line);
        }
        line = strtok(NULL, "\r\n");
      }
    }
  }
  if (windDataValid && millis() - lastWindMs > WIND_SIGNAL_TIMEOUT_MS) {
    windDataValid = false;
  }
}

static const uint32_t GPS_TARGET_BAUD = 115200;
static const uint32_t NMEA_BAUD       = 115200;
static const uint32_t UPDATE_MS       = 1000;

#define RS485_RX_PIN   43
#define RS485_TX_PIN   44
#define I2C_SDA_PIN    15
#define I2C_SCL_PIN     7

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// ===================== Farben =====================
static const uint16_t UI_BG        = 0x0000;
static const uint16_t UI_CARD_EDGE = 0x2A69;
static const uint16_t UI_CYAN      = 0x05FF;
static const uint16_t UI_SOFT_CYAN = 0x867D;
static const uint16_t UI_WHITE     = 0xFFFF;
static const uint16_t UI_RED       = 0xF800;
static const uint16_t UI_YELLOW    = 0xFFE0;
static const uint16_t UI_ORANGE    = 0xFD20;
static const uint16_t UI_GREEN     = 0x07E0;
static const uint16_t UI_LIGHTGREY = 0xC618;
static const uint16_t UI_DARKGREY  = 0x528A;

static const bool   WP_ENABLED = false;
static const double WP_LAT = 52.520816;
static const double WP_LON = 13.409419;

enum CompassType { COMPASS_NONE, COMPASS_HMC5883L, COMPASS_QMC5883L };
CompassType compassType = COMPASS_NONE;
float declination_deg = 4.0f;
static const bool headUp = false;
static int bootY = 10;   // Vorab deklariert, damit Boot-Status-Funktionen (inkl. Kompass-Warteschleife) darauf zugreifen koennen

// ===================== NMEA-Fenster: Zeilen-Ringpuffer =====================
// Statt einem einzelnen Zeichenstrom (der Saetze mitten im Wort umbricht),
// wird hier satzweise gepuffert: jeder komplette NMEA-Satz landet als EINE
// Zeile im Ringpuffer. Ist ein Satz laenger als die Anzeigebreite, wird der
// Rest einfach abgeschnitten (nicht umgebrochen) - so bleibt sofort sichtbar,
// welcher Satztyp mit welchen Werten reinkommt.
#define NMEA_LINE_MAXCHARS 24
#define NMEA_DISPLAY_LINES 6
static char nmeaLineBuf[NMEA_DISPLAY_LINES][NMEA_LINE_MAXCHARS + 1];
static int  nmeaLineCount = 0;   // wie viele Zeilen aktuell befuellt sind
static int  nmeaLineHead  = 0;   // naechster zu beschreibender Ringpuffer-Slot

static char   nmeaCurLine[NMEA_LINE_MAXCHARS + 1];
static size_t nmeaCurLineLen = 0;

void appendGpsRawChar(char c) {
  if (c == '\r') return;
  if (c == '\n') {
    // Zeile abgeschlossen -> in den Ringpuffer uebernehmen
    nmeaCurLine[nmeaCurLineLen] = '\0';
    if (nmeaCurLineLen > 0) {   // leere Zeilen (nur \r\n) ignorieren
      strncpy(nmeaLineBuf[nmeaLineHead], nmeaCurLine, NMEA_LINE_MAXCHARS);
      nmeaLineBuf[nmeaLineHead][NMEA_LINE_MAXCHARS] = '\0';
      nmeaLineHead = (nmeaLineHead + 1) % NMEA_DISPLAY_LINES;
      if (nmeaLineCount < NMEA_DISPLAY_LINES) nmeaLineCount++;
    }
    nmeaCurLineLen = 0;
    return;
  }
  if ((unsigned char)c < 32) c = '.';   // sonstige Steuerzeichen ersetzen
  // Nur bis zur Zeilenbreite speichern - alles danach wird bis zum
  // naechsten Zeilenumbruch stillschweigend verworfen (= Abschneiden)
  if (nmeaCurLineLen < NMEA_LINE_MAXCHARS) {
    nmeaCurLine[nmeaCurLineLen++] = c;
  }
}

void drawRawRs485Box(int x, int y, int w, int h) {
  (void)w;(void)h;
  gfx->setTextColor(UI_CYAN,UI_BG);gfx->setTextSize(1);
  gfx->setCursor(x,y);gfx->print("NMEA Window");
  gfx->setTextColor(UI_WHITE,UI_BG);
  // Aelteste zuerst, neueste zuletzt (chronologisch von oben nach unten)
  int startIdx = (nmeaLineCount < NMEA_DISPLAY_LINES) ? 0 : nmeaLineHead;
  for (int i = 0; i < nmeaLineCount; i++) {
    int idx = (startIdx + i) % NMEA_DISPLAY_LINES;
    gfx->setCursor(x, y + 14 + i * 16);
    gfx->print(nmeaLineBuf[idx]);
  }
  gfx->setTextSize(2);
}

// ===================== I2C / Kompass =====================
void hmcWrite(uint8_t r,uint8_t v){Wire.beginTransmission(0x1E);Wire.write(r);Wire.write(v);Wire.endTransmission();}
int16_t hmcRead16(uint8_t r){Wire.beginTransmission(0x1E);Wire.write(r);Wire.endTransmission(false);Wire.requestFrom((uint8_t)0x1E,(uint8_t)2);if(Wire.available()<2)return 0;return(int16_t)((Wire.read()<<8)|Wire.read());}
void hmcInit(){hmcWrite(0x00,0x70);hmcWrite(0x01,0x20);hmcWrite(0x02,0x00);}
void hmcRead(int16_t &x,int16_t &y,int16_t &z){x=hmcRead16(0x03);z=hmcRead16(0x05);y=hmcRead16(0x07);}
void qmcWrite(uint8_t r,uint8_t v){Wire.beginTransmission(0x0D);Wire.write(r);Wire.write(v);Wire.endTransmission();}
int16_t qmcRead16LE(uint8_t r){Wire.beginTransmission(0x0D);Wire.write(r);Wire.endTransmission(false);Wire.requestFrom((uint8_t)0x0D,(uint8_t)2);if(Wire.available()<2)return 0;uint8_t l=Wire.read(),h=Wire.read();return(int16_t)((h<<8)|l);}
void qmcInit(){qmcWrite(0x0B,0x01);qmcWrite(0x09,0b00011101);}
void qmcRead(int16_t &x,int16_t &y,int16_t &z){x=qmcRead16LE(0x00);y=qmcRead16LE(0x02);z=qmcRead16LE(0x04);}

void detectCompass(){
  Wire.setClock(100000);
  if(i2cDevicePresent(0x1E)){compassType=COMPASS_HMC5883L;hmcInit();Serial.println("[COMPASS] HMC5883L");}
  else if(i2cDevicePresent(0x0D)){compassType=COMPASS_QMC5883L;qmcInit();Serial.println("[COMPASS] QMC5883L");}
  else{compassType=COMPASS_NONE;}
}

// I2C-Bus-Recovery: manche Sensor-Module blockieren den Bus (SDA haengt LOW
// fest), wenn sie mitten in einer Uebertragung ausfallen oder haengen
// bleiben. Reines Wiederholen von Wire.beginTransmission() hilft dann
// NICHT - der Bus muss per manuellem SCL-Toggle wieder freigegeben werden
// (das ist softwareseitig das Aequivalent zum Spannung-Aus/An).
void i2cBusRecovery() {
  Wire.end();
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT);
  for (int i = 0; i < 20; i++) {          // mehr Taktimpulse als vorher (war 9)
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
    if (digitalRead(I2C_SDA_PIN) == HIGH) break;  // SDA wieder frei
  }
  // Abschliessendes STOP-Signal erzeugen (SDA LOW->HIGH waehrend SCL HIGH)
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(I2C_SDA_PIN, HIGH);
  delayMicroseconds(5);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
}

// Wartet UNBEGRENZT, bis der GT911 (Touch) nach dem CH422G-Reset auf I2C
// antwortet - gibt nie auf. Versucht dabei periodisch erneut den Reset-
// Puls und zusaetzlich eine I2C-Bus-Recovery, falls der Bus haengt
// (z.B. direkt nach einem Firmware-Upload). Zeigt live den Fortschritt.
// Falls der Chip nach TOUCH_SELF_RESTART_MS immer noch nicht antwortet,
// fuehrt sich der ESP32 selbst per Software neu (ESP.restart()) - das
// entspricht dem manuellen Druecken der Reset-Taste, was nachweislich
// hilft, waehrend der automatische Reset beim Firmware-Upload (ueber
// USB DTR/RTS) das Problem in manchen Faellen nicht behebt.
static const uint32_t TOUCH_SELF_RESTART_MS = 8000;

void touchWaitForever(bool showOnDisplay) {
  uint32_t startMs = millis();
  uint32_t lastDisplayUpdate = 0;
  uint32_t lastRecovery = 0;
  bool found = false;

  while (!found) {
    found = gt911_init(500);   // fuehrt CH422G-Reset aus + kurzer Antwort-Check
    if (found) break;

    uint32_t elapsed = (millis() - startMs) / 1000;
    if (millis() - lastDisplayUpdate > 500) {
      lastDisplayUpdate = millis();
      char buf[40];
      snprintf(buf, 40, "[ .. ] Suche Touch GT911... (%lus)", (unsigned long)elapsed);
      Serial.println(buf);
      if (showOnDisplay) {
        gfx->fillRect(0, bootY, 480, 14, UI_BG);
        gfx->setTextSize(1);
        gfx->setTextColor(UI_YELLOW, UI_BG);
        gfx->setCursor(5, bootY);
        gfx->print(buf);
      }
    }

    if (millis() - lastRecovery > 3000) {
      lastRecovery = millis();
      Serial.println("[I2C] Bus-Recovery Versuch (Touch)...");
      i2cBusRecovery();
    }

    if (millis() - startMs > TOUCH_SELF_RESTART_MS) {
      Serial.println("[TOUCH] Kein Erfolg - fuehre Software-Reset durch (wie Reset-Taste)...");
      delay(50);   // Serial-Ausgabe noch rausschreiben lassen
      ESP.restart();
    }
  }
}

// Wartet UNBEGRENZT, bis ein Kompass gefunden wird - gibt nie auf.
// Zeigt dabei laufend Fortschritt (verstrichene Sekunden) auf dem Display,
// damit klar ist, dass das Board aktiv sucht und nicht haengt.
void compassWaitForever() {
  uint32_t startMs = millis();
  uint32_t lastDisplayUpdate = 0;
  uint32_t lastRecovery = millis() - 2000;  // sorgt dafuer, dass der erste
                                             // Recovery-Versuch quasi sofort
                                             // laeuft, nicht erst nach 1,5s
  while (compassType == COMPASS_NONE) {
    handleTouch();   // Touch parallel testbar machen, waehrend Kompass sucht
    detectCompass();
    if (compassType != COMPASS_NONE) break;

    uint32_t elapsed = (millis() - startMs) / 1000;
    if (millis() - lastDisplayUpdate > 500) {
      lastDisplayUpdate = millis();
      char buf[40];
      snprintf(buf, 40, "[ .. ] Suche Kompass... (%lus)", (unsigned long)elapsed);
      gfx->fillRect(0, bootY, 480, 14, UI_BG);
      gfx->setTextSize(1);
      gfx->setTextColor(UI_YELLOW, UI_BG);
      gfx->setCursor(5, bootY);
      gfx->print(buf);
      Serial.println(buf);
    }

    // Haeufiger als vorher (jetzt alle 1,5s statt 3s): I2C-Bus-Recovery
    // versuchen, falls der Bus haengt - reines Wiederholen von
    // detectCompass() alleine hilft dann nicht.
    if (millis() - lastRecovery > 1500) {
      lastRecovery = millis();
      Serial.println("[I2C] Bus-Recovery Versuch...");
      i2cBusRecovery();
    }

    delay(150);
  }
}

// ===================== Heading =====================
float wrap360(float a){while(a<0)a+=360;while(a>=360)a-=360;return a;}
float computeHeadingTrueishDeg(int16_t x,int16_t y){return wrap360(atan2((float)y,(float)x)*180.0f/PI+declination_deg);}
float computeHeadingMagDeg(int16_t x,int16_t y){return wrap360(atan2((float)y,(float)x)*180.0f/PI);}
static float headingFilt=NAN,needleFilt=NAN;
float filterHeadingDeg(float h){
  if(isnan(h))return h;if(isnan(headingFilt)){headingFilt=h;return h;}
  float d=h-headingFilt;if(d>180)d-=360;if(d<-180)d+=360;
  headingFilt=wrap360(headingFilt+d*0.15f);return headingFilt;
}
float filterNeedleDeg(float h){
  if(isnan(h))return h;if(isnan(needleFilt)){needleFilt=h;return h;}
  float d=h-needleFilt;if(d>180)d-=360;if(d<-180)d+=360;
  needleFilt=wrap360(needleFilt+d*0.22f);return needleFilt;
}

// ===================== LED =====================
void updateStatusLED(bool wifiOK,bool gpsFix,bool compassOK,bool dataWeak){
  static uint32_t lastBlink=0;static bool ledState=false;uint32_t now=millis();
  if(!wifiOK){if(now-lastBlink>=100){ledState=!ledState;digitalWrite(LED_BUILTIN,ledState);lastBlink=now;}return;}
  if(!gpsFix){if(now-lastBlink>=500){ledState=!ledState;digitalWrite(LED_BUILTIN,ledState);lastBlink=now;}return;}
  if(!compassOK){uint32_t t=now%1000;digitalWrite(LED_BUILTIN,(t<100||(t>=200&&t<300))?HIGH:LOW);return;}
  if(dataWeak){digitalWrite(LED_BUILTIN,(now%1000<30)?HIGH:LOW);return;}
  digitalWrite(LED_BUILTIN,HIGH);
}

// ===================== NMEA =====================
static uint8_t nmea_cs=0;
static inline void nmeaTrimLeft(char* s){while(*s==' ')memmove(s,s+1,strlen(s));}
static inline void outChar(char c){Serial.write(c);if(nmeaClient&&nmeaClient.connected())nmeaClient.write((uint8_t)c);}
static inline void outStr(const char* s){Serial.print(s);if(nmeaClient&&nmeaClient.connected())nmeaClient.print(s);}
void nmeaStart(){nmea_cs=0;outChar('$');}
void nmeaPutChar(char c){nmea_cs^=(uint8_t)c;outChar(c);}
void nmeaPutStr(const char* s){while(*s)nmeaPutChar(*s++);}
void nmeaComma(){nmeaPutChar(',');}
void nmeaEnd(){outChar('*');char h[3];sprintf(h,"%02X",nmea_cs);outStr(h);outStr("\r\n");}

void nmeaFormatLatLon(double lat,double lon,char* lf,char* lh,char* lnf,char* lnh){
  char lhc=(lat>=0)?'N':'S',lohc=(lon>=0)?'E':'W';
  lat=fabs(lat);lon=fabs(lon);
  int ld=(int)lat,lod=(int)lon;
  double lm=(lat-ld)*60.0,lom=(lon-lod)*60.0;
  char buf[16];
  dtostrf(lm,0,4,buf);nmeaTrimLeft(buf);if(buf[1]=='.'){char tmp[16];tmp[0]='0';strcpy(tmp+1,buf);strcpy(buf,tmp);}
  sprintf(lf,"%02d%s",ld,buf);
  dtostrf(lom,0,4,buf);nmeaTrimLeft(buf);if(buf[1]=='.'){char tmp[16];tmp[0]='0';strcpy(tmp+1,buf);strcpy(buf,tmp);}
  sprintf(lnf,"%03d%s",lod,buf);
  lh[0]=lhc;lh[1]='\0';lnh[0]=lohc;lnh[1]='\0';
}

void emitGNRMC(){
  bool lv=gps.location.isValid(),tv=gps.time.isValid(),dv=gps.date.isValid();
  nmeaStart();nmeaPutStr("GNRMC");nmeaComma();
  if(tv){char t[16];sprintf(t,"%02u%02u%02u.00",gps.time.hour(),gps.time.minute(),gps.time.second());nmeaPutStr(t);}
  nmeaComma();nmeaPutChar((lv&&tv)?'A':'V');nmeaComma();
  if(lv){char lf[20],lnf[20],lh[2],lnh[2];nmeaFormatLatLon(gps.location.lat(),gps.location.lng(),lf,lh,lnf,lnh);nmeaPutStr(lf);nmeaComma();nmeaPutStr(lh);nmeaComma();nmeaPutStr(lnf);nmeaComma();nmeaPutStr(lnh);nmeaComma();}
  else{nmeaComma();nmeaComma();nmeaComma();nmeaComma();}
  if(gps.speed.isValid()){char s[16];dtostrf(gps.speed.knots(),0,1,s);nmeaTrimLeft(s);nmeaPutStr(s);}nmeaComma();
  if(gps.course.isValid()){char c[16];dtostrf(gps.course.deg(),0,1,c);nmeaTrimLeft(c);nmeaPutStr(c);}nmeaComma();
  if(dv){char d[16];sprintf(d,"%02u%02u%02u",gps.date.day(),gps.date.month(),(uint8_t)(gps.date.year()%100));nmeaPutStr(d);}
  nmeaComma();nmeaComma();nmeaEnd();
}

void emitGNGGA(){
  bool lv=gps.location.isValid(),tv=gps.time.isValid();
  nmeaStart();nmeaPutStr("GNGGA");nmeaComma();
  if(tv){char t[16];sprintf(t,"%02u%02u%02u.00",gps.time.hour(),gps.time.minute(),gps.time.second());nmeaPutStr(t);}
  nmeaComma();
  if(lv){char lf[20],lnf[20],lh[2],lnh[2];nmeaFormatLatLon(gps.location.lat(),gps.location.lng(),lf,lh,lnf,lnh);nmeaPutStr(lf);nmeaComma();nmeaPutStr(lh);nmeaComma();nmeaPutStr(lnf);nmeaComma();nmeaPutStr(lnh);nmeaComma();}
  else{nmeaComma();nmeaComma();nmeaComma();nmeaComma();}
  nmeaPutChar(lv?'1':'0');nmeaComma();
  if(gps.satellites.isValid()){char s[6];sprintf(s,"%02u",gps.satellites.value());nmeaPutStr(s);}else nmeaPutStr("00");
  nmeaComma();
  if(gps.hdop.isValid()){char h[16];dtostrf(gps.hdop.hdop(),0,1,h);nmeaTrimLeft(h);nmeaPutStr(h);}else nmeaPutStr("99.9");
  nmeaComma();
  if(gps.altitude.isValid()){char a[16];dtostrf(gps.altitude.meters(),0,1,a);nmeaTrimLeft(a);nmeaPutStr(a);}else nmeaPutStr("0.0");
  nmeaComma();nmeaPutChar('M');nmeaComma();nmeaComma();nmeaPutChar('M');nmeaComma();nmeaComma();nmeaEnd();
}

void emitGNVTG(){
  nmeaStart();nmeaPutStr("GNVTG");nmeaComma();
  if(gps.course.isValid()){char c[16];dtostrf(gps.course.deg(),0,1,c);nmeaTrimLeft(c);nmeaPutStr(c);}
  nmeaComma();nmeaPutChar('T');nmeaComma();nmeaComma();nmeaPutChar('M');nmeaComma();
  if(gps.speed.isValid()){char s[16];dtostrf(gps.speed.knots(),0,1,s);nmeaTrimLeft(s);nmeaPutStr(s);}
  nmeaComma();nmeaPutChar('N');nmeaComma();
  if(gps.speed.isValid()){char k[16];dtostrf(gps.speed.kmph(),0,1,k);nmeaTrimLeft(k);nmeaPutStr(k);}
  nmeaComma();nmeaPutChar('K');nmeaEnd();
}

void emitHCHDM(float hdgMag,bool haveCompass){
  if(!haveCompass||isnan(hdgMag))return;
  nmeaStart();nmeaPutStr("HCHDM");nmeaComma();
  char h[16];dtostrf(hdgMag,0,1,h);nmeaTrimLeft(h);nmeaPutStr(h);
  nmeaComma();nmeaPutChar('M');nmeaEnd();
}

// Gibt die vom Windsensor empfangenen Daten als $WIMWV weiter aus (Serial +
// TCP-NMEA-Client) - so bekommen auch andere Empfaenger (z.B. M5Tough) die
// Winddaten, nicht nur unser eigenes Display.
void emitWIMWV(bool windValid,float windAngle,float windSpeedKn){
  nmeaStart();nmeaPutStr("WIMWV");nmeaComma();
  char a[16];dtostrf(windAngle,0,1,a);nmeaTrimLeft(a);nmeaPutStr(a);
  nmeaComma();nmeaPutChar('R');nmeaComma();   // R = relativ zum Bug (wie empfangen)
  char s[16];dtostrf(windSpeedKn,0,1,s);nmeaTrimLeft(s);nmeaPutStr(s);
  nmeaComma();nmeaPutChar('N');nmeaComma();   // N = Knoten
  nmeaPutChar(windValid?'A':'V');
  nmeaEnd();
}

// ===================== DMS =====================
void formatDMS(double v,bool isLat,char* out,size_t sz){
  char h=isLat?(v>=0?'N':'S'):(v>=0?'E':'W');
  double a=fabs(v);int d=(int)a;double mf=(a-d)*60.0;int m=(int)mf;
  int s10=(int)lround((mf-m)*60.0*10.0);
  if(s10>=600){s10=0;m++;}if(m>=60){m=0;d++;}
  if(isLat)snprintf(out,sz,"%02d %02d'%02d.%1d\" %c",d,m,s10/10,s10%10,h);
  else     snprintf(out,sz,"%03d %02d'%02d.%1d\" %c",d,m,s10/10,s10%10,h);
}

// Gibt eine DMS-Position ab einer festen linken Startposition aus, mit
// einem separat gezeichneten Gradsymbol nach der Gradzahl - noetig, weil
// unsere Bold-Schriftarten nur ASCII bis 0x7E abdecken (kein "°"-Zeichen).
// gfx->setFont()/Farbe muessen VOR dem Aufruf bereits gesetzt sein.
// Vorab-Deklaration: die automatische Prototyp-Erstellung von Arduino
// erkennt Funktionen mit Standardwerten (hier r=2) manchmal nicht
// zuverlaessig, wenn sie erst spaeter im Sketch definiert werden.
void drawDegreeSymbol(int x,int y,uint16_t c,int r=2);

void printDMSWithDegree(int xLeft, int yBaseline, double v, bool isLat, uint16_t color){
  char h=isLat?(v>=0?'N':'S'):(v>=0?'E':'W');
  double a=fabs(v);int d=(int)a;double mf=(a-d)*60.0;int m=(int)mf;
  int s10=(int)lround((mf-m)*60.0*10.0);
  if(s10>=600){s10=0;m++;}if(m>=60){m=0;d++;}
  char degStr[8],restStr[24];
  if(isLat) snprintf(degStr,8,"%02d",d); else snprintf(degStr,8,"%03d",d);
  snprintf(restStr,24," %02d'%02d.%1d\" %c",m,s10/10,s10%10,h);

  gfx->setCursor(xLeft,yBaseline);
  gfx->print(degStr);
  int16_t tbx,tby; uint16_t tbw,tbh;
  gfx->getTextBounds(degStr,0,0,&tbx,&tby,&tbw,&tbh);
  int symX = xLeft+(int)tbw+6;
  int symY = yBaseline-(int)tbh+3;   // mittig auf Zahlenhoehe, nicht unten haengend
  drawDegreeSymbol(symX,symY,color,2);
  gfx->setCursor(symX+8,yBaseline);
  gfx->print(restStr);
}

// ===================== Grafik =====================
static inline float deg2rad_f(float d){return d*(float)PI/180.0f;}
void drawDegreeSymbol(int x,int y,uint16_t c,int r){gfx->drawCircle(x,y,r,c);}

void drawArrow(int x0,int y0,float angleDeg,int len,int wing,uint16_t color){
  float a=deg2rad_f(angleDeg-90.0f);
  int x1=x0+(int)lroundf(cosf(a)*len),y1=y0+(int)lroundf(sinf(a)*len);
  gfx->drawLine(x0,y0,x1,y1,color);
  float pr=a+(float)PI/2.0f;
  int bx=x0+(int)lroundf(cosf(a)*(len-10)),by=y0+(int)lroundf(sinf(a)*(len-10));
  int lx=bx+(int)lroundf(cosf(pr)*wing),ly=by+(int)lroundf(sinf(pr)*wing);
  int rx=bx-(int)lroundf(cosf(pr)*wing),ry=by-(int)lroundf(sinf(pr)*wing);
  gfx->drawLine(x1,y1,lx,ly,color);gfx->drawLine(x1,y1,rx,ry,color);gfx->drawLine(lx,ly,rx,ry,color);
}

void drawCompassNeedleStyled(int cx,int cy,float angleDeg,int len){
  float a=deg2rad_f(angleDeg-90.0f);int hb=10,tl=len/3;
  int tipX=cx+(int)lroundf(cosf(a)*len),tipY=cy+(int)lroundf(sinf(a)*len);
  int tailX=cx-(int)lroundf(cosf(a)*tl),tailY=cy-(int)lroundf(sinf(a)*tl);
  int lx=cx+(int)lroundf(cosf(a+PI/2)*hb),ly=cy+(int)lroundf(sinf(a+PI/2)*hb);
  int rx=cx+(int)lroundf(cosf(a-PI/2)*hb),ry=cy+(int)lroundf(sinf(a-PI/2)*hb);
  gfx->fillTriangle(tipX,tipY,lx,ly,rx,ry,UI_RED);
  gfx->fillTriangle(tailX,tailY,lx,ly,rx,ry,UI_LIGHTGREY);
  gfx->drawLine(tipX,tipY,lx,ly,UI_WHITE);gfx->drawLine(tipX,tipY,rx,ry,UI_WHITE);
  gfx->drawLine(tailX,tailY,lx,ly,UI_DARKGREY);gfx->drawLine(tailX,tailY,rx,ry,UI_DARKGREY);
  gfx->fillCircle(cx,cy,7,UI_LIGHTGREY);gfx->drawCircle(cx,cy,7,UI_WHITE);gfx->fillCircle(cx,cy,2,UI_WHITE);
}

void drawCompassNeedleStyledLarge(int cx,int cy,float angleDeg,int len){
  float a=deg2rad_f(angleDeg-90.0f);int hb=18,tl=len/3;
  int tipX=cx+(int)lroundf(cosf(a)*len),tipY=cy+(int)lroundf(sinf(a)*len);
  int tailX=cx-(int)lroundf(cosf(a)*tl),tailY=cy-(int)lroundf(sinf(a)*tl);
  int lx=cx+(int)lroundf(cosf(a+PI/2)*hb),ly=cy+(int)lroundf(sinf(a+PI/2)*hb);
  int rx=cx+(int)lroundf(cosf(a-PI/2)*hb),ry=cy+(int)lroundf(sinf(a-PI/2)*hb);
  gfx->fillTriangle(tipX,tipY,lx,ly,rx,ry,UI_RED);
  gfx->fillTriangle(tailX,tailY,lx,ly,rx,ry,UI_LIGHTGREY);
  gfx->drawLine(tipX,tipY,lx,ly,UI_WHITE);gfx->drawLine(tipX,tipY,rx,ry,UI_WHITE);
  gfx->drawLine(tailX,tailY,lx,ly,UI_DARKGREY);gfx->drawLine(tailX,tailY,rx,ry,UI_DARKGREY);
  gfx->fillCircle(cx,cy,12,UI_LIGHTGREY);gfx->drawCircle(cx,cy,12,UI_WHITE);gfx->fillCircle(cx,cy,4,UI_WHITE);
}

void drawCompassRose(int cx,int cy,int r,float headingDeg,bool cogValid,float cogDeg,bool wpValid,float wpBearDeg){
  float rot=headUp?headingDeg:0.0f;
  gfx->drawCircle(cx,cy,r,UI_CYAN);gfx->drawCircle(cx,cy,r-1,UI_CYAN);
  gfx->drawCircle(cx,cy,r-18,UI_SOFT_CYAN);gfx->drawCircle(cx,cy,r-19,UI_SOFT_CYAN);
  for(int a=0;a<360;a+=10){
    float ar=deg2rad_f(((float)a-rot)-90.0f);
    int x1=cx+(int)lroundf(cosf(ar)*(r-1)),y1=cy+(int)lroundf(sinf(ar)*(r-1));
    int inner=(a%30==0)?(r-16):(r-8);
    int x2=cx+(int)lroundf(cosf(ar)*inner),y2=cy+(int)lroundf(sinf(ar)*inner);
    gfx->drawLine(x1,y1,x2,y2,(a%30==0)?UI_WHITE:UI_SOFT_CYAN);
  }
  gfx->setTextSize(1);
  for(int a=0;a<360;a+=20){
    char num[8];snprintf(num,8,"%d",a);
    float ar=deg2rad_f(((float)a-rot)-90.0f);
    gfx->setTextColor(UI_WHITE,UI_BG);
    gfx->setCursor(cx+(int)lroundf(cosf(ar)*(r+10))-6,cy+(int)lroundf(sinf(ar)*(r+10))+4);
    gfx->print(num);
  }
  gfx->setFont(&FreeSansBold18pt7b);
  gfx->setTextSize(1);
  auto lbl=[&](float angle,const char* txt,uint16_t color){
    float ar=deg2rad_f((angle-rot)-90.0f);
    int16_t tbx,tby; uint16_t tbw,tbh;
    gfx->getTextBounds(txt,0,0,&tbx,&tby,&tbw,&tbh);
    gfx->setTextColor(color,UI_BG);
    gfx->setCursor(cx+(int)lroundf(cosf(ar)*(r-30))-(int)tbw/2,cy+(int)lroundf(sinf(ar)*(r-30))+(int)tbh/2);
    gfx->print(txt);
  };
  lbl(0,"N",UI_RED);lbl(90,"E",UI_WHITE);lbl(180,"S",UI_WHITE);lbl(270,"W",UI_WHITE);
  gfx->setFont();
  drawCompassNeedleStyled(cx,cy,headUp?0.0f:headingDeg,r-20);
  if(cogValid)drawArrow(cx,cy,headUp?wrap360(cogDeg-headingDeg):cogDeg,r-40,7,UI_GREEN);
  if(wpValid)drawArrow(cx,cy,headUp?wrap360(wpBearDeg-headingDeg):wpBearDeg,r-56,7,UI_YELLOW);
  char buf[16];snprintf(buf,16,"%03d",((int)lroundf(headingDeg))%360);
  gfx->setFont(&FreeSansBold12pt7b);
  gfx->setTextSize(1);
  int16_t tbx,tby; uint16_t tbw,tbh;
  gfx->getTextBounds(buf,0,0,&tbx,&tby,&tbw,&tbh);
  gfx->setTextColor(UI_WHITE,UI_BG);gfx->setCursor(cx-(int)tbw/2,cy+r+18+(int)tbh);gfx->print(buf);
  gfx->setFont();
  drawDegreeSymbol(cx+(int)tbw/2+8,cy+r+18+(int)tbh-8,UI_WHITE,2);
}

void drawCompassRoseFullscreen(int cx,int cy,int r,float headingDeg,
    bool cogValid,float cogDeg,bool wpValid,float wpBearDeg,
    bool locValid,double lat,double lon,bool timeValid,uint32_t hh,uint32_t mm,uint32_t ss,
    int labelTopY=6){
  float rot=headUp?headingDeg:0.0f;
  gfx->drawCircle(cx,cy,r,UI_CYAN);gfx->drawCircle(cx,cy,r-1,UI_CYAN);
  gfx->drawCircle(cx,cy,r-22,UI_SOFT_CYAN);gfx->drawCircle(cx,cy,r-23,UI_SOFT_CYAN);
  for(int a=0;a<360;a+=5){
    float ar=deg2rad_f(((float)a-rot)-90.0f);
    int x1=cx+(int)lroundf(cosf(ar)*(r-1)),y1=cy+(int)lroundf(sinf(ar)*(r-1));
    int inner=(a%30==0)?(r-22):(a%10==0)?(r-12):(r-7);
    int x2=cx+(int)lroundf(cosf(ar)*inner),y2=cy+(int)lroundf(sinf(ar)*inner);
    gfx->drawLine(x1,y1,x2,y2,(a%30==0)?UI_WHITE:UI_SOFT_CYAN);
  }
  gfx->setTextSize(1);
  for(int a=0;a<360;a+=10){
    char num[8];snprintf(num,8,"%d",a);
    float ar=deg2rad_f(((float)a-rot)-90.0f);
    gfx->setTextColor(UI_WHITE,UI_BG);
    gfx->setCursor(cx+(int)lroundf(cosf(ar)*(r+12))-7,cy+(int)lroundf(sinf(ar)*(r+12))-5);
    gfx->print(num);
  }
  gfx->setFont(&FreeSansBold18pt7b);
  gfx->setTextSize(1);
  auto lbl=[&](float angle,const char* txt,uint16_t color){
    float ar=deg2rad_f((angle-rot)-90.0f);
    int16_t tbx,tby; uint16_t tbw,tbh;
    gfx->getTextBounds(txt,0,0,&tbx,&tby,&tbw,&tbh);
    gfx->setTextColor(color,UI_BG);
    gfx->setCursor(cx+(int)lroundf(cosf(ar)*(r-38))-(int)tbw/2,cy+(int)lroundf(sinf(ar)*(r-38))+(int)tbh/2);
    gfx->print(txt);
  };
  lbl(0,"N",UI_RED);lbl(90,"E",UI_WHITE);lbl(180,"S",UI_WHITE);lbl(270,"W",UI_WHITE);
  gfx->setFont();
  drawCompassNeedleStyledLarge(cx,cy,headUp?0.0f:headingDeg,r-26);
  if(cogValid)drawArrow(cx,cy,headUp?wrap360(cogDeg-headingDeg):cogDeg,r-50,10,UI_GREEN);
  if(wpValid)drawArrow(cx,cy,headUp?wrap360(wpBearDeg-headingDeg):wpBearDeg,r-72,10,UI_YELLOW);
  char buf[16];snprintf(buf,16,"%03d",((int)lroundf(headingDeg))%360);
  gfx->setFont(&FreeSansBold18pt7b);
  gfx->setTextSize(1);
  gfx->setTextColor(UI_WHITE,UI_BG);
  int16_t tbx,tby; uint16_t tbw,tbh;
  gfx->getTextBounds(buf,0,0,&tbx,&tby,&tbw,&tbh);
  gfx->setCursor(cx-(int)tbw/2, cy+8+(int)tbh);
  gfx->print(buf);
  gfx->setFont();   // zurueck auf Standard-Pixelschrift fuer den Rest
  drawDegreeSymbol(cx+(int)tbw/2+10,cy+8,UI_WHITE,3);
  gfx->setFont(&FreeSansBold12pt7b);
  gfx->setTextSize(1);
  gfx->setTextColor(UI_RED,UI_BG);gfx->setCursor(6,labelTopY+10);gfx->print("RED=HDG");
  gfx->setTextColor(UI_GREEN,UI_BG);gfx->setCursor(6,labelTopY+32);gfx->print("GRN=COG");
  if(locValid){
    char latStr[32],lonStr[32];
    formatDMS(lat,true,latStr,32);formatDMS(lon,false,lonStr,32);
    gfx->setTextColor(UI_CYAN,UI_BG);
    int16_t tbx,tby; uint16_t tbw,tbh;
    gfx->getTextBounds(latStr,0,0,&tbx,&tby,&tbw,&tbh);
    printDMSWithDegree(cx - (int)tbw/2, cy + 58, lat, true, UI_CYAN);
    gfx->getTextBounds(lonStr,0,0,&tbx,&tby,&tbw,&tbh);
    printDMSWithDegree(cx - (int)tbw/2, cy + 78, lon, false, UI_CYAN);
    if(timeValid){
      char tb[20];snprintf(tb,20,"UTC %02lu:%02lu:%02lu",(unsigned long)hh,(unsigned long)mm,(unsigned long)ss);
      gfx->getTextBounds(tb,0,0,&tbx,&tby,&tbw,&tbh);
      gfx->setTextColor(UI_SOFT_CYAN,UI_BG);gfx->setCursor(cx - (int)tbw/2, cy + 98);gfx->print(tb);
    }
  } else {
    gfx->setTextColor(UI_DARKGREY,UI_BG);gfx->setCursor(cx-56,cy+50);gfx->print("kein Fix");
  }
  gfx->setFont();
}

// Kompakte Windrose fuer den 1/4-Ausschnitt oben in der Kompass-Vollbildansicht.
// Zeigt nur den Windwinkel als Nadel plus Geschwindigkeit, keine Gradzahlen.
void drawWindRoseMini(int cx,int cy,int r,bool windValid,float windAngle,float windSpeedKn){
  // Backbord/Steuerbord-Faerbung wie im Vollbild: gruen 0-120 (Steuerbord),
  // rot 240-360 (Backbord), Heckbereich dazwischen neutral
  for (int a = 0; a < 360; a++) {
    uint16_t bandColor = (a > 0 && a <= 120) ? UI_GREEN : (a >= 240 && a < 360) ? UI_RED : UI_BG;
    float ar = deg2rad_f((float)a - 90.0f);
    int x1 = cx + (int)lroundf(cosf(ar) * (r - 1));
    int y1 = cy + (int)lroundf(sinf(ar) * (r - 1));
    int x2 = cx + (int)lroundf(cosf(ar) * (r - 10));
    int y2 = cy + (int)lroundf(sinf(ar) * (r - 10));
    gfx->drawLine(x1, y1, x2, y2, bandColor);
  }
  gfx->drawCircle(cx,cy,r,UI_YELLOW);
  gfx->drawCircle(cx,cy,r-1,UI_YELLOW);
  gfx->setTextSize(1);
  gfx->setTextColor(UI_WHITE,UI_BG);
  gfx->setCursor(cx-4,cy-r-2);gfx->print("0");
  gfx->setCursor(cx+r+2,cy-4);gfx->print("90");
  gfx->setCursor(cx-4,cy+r+2);gfx->print("180");
  gfx->setCursor(cx-r-14,cy-4);gfx->print("270");
  if(windValid){
    drawArrow(cx,cy,windAngle,r-8,5,UI_YELLOW);
    gfx->fillCircle(cx,cy,4,UI_YELLOW);
    gfx->setFont(&FreeSansBold12pt7b);
    gfx->setTextSize(1);
    gfx->setTextColor(UI_YELLOW,UI_BG);
    char aDeg[8]; snprintf(aDeg,8,"%03d",((int)lroundf(windAngle))%360);
    gfx->setCursor(230,cy+r+30);
    gfx->print("WA ");
    gfx->print(aDeg);
    int16_t tbx,tby; uint16_t tbw,tbh;
    gfx->getTextBounds(aDeg,0,0,&tbx,&tby,&tbw,&tbh);
    int16_t wax,way; uint16_t waw,wah;
    gfx->getTextBounds("WA ",0,0,&wax,&way,&waw,&wah);
    int symX = 230+(int)waw+(int)tbw+6;
    int symY = (cy+r+30)-(int)tbh+3;
    drawDegreeSymbol(symX,symY,UI_YELLOW,2);
    char sBuf[16];snprintf(sBuf,16,"WS %.1fkn",windSpeedKn);
    gfx->setCursor(cx+4,cy+r+30);gfx->print(sBuf);
    gfx->setFont();
  } else {
    gfx->setFont(&FreeSansBold12pt7b);
    gfx->setTextSize(1);
    gfx->setTextColor(UI_DARKGREY,UI_BG);
    gfx->setCursor(cx-55,cy+r+20);gfx->print("kein Wind-Signal");
    gfx->setFont();
  }
}

// Grosse, eigenstaendige Windrose als komplette Vollbildansicht.
// Windstatistik-Ansicht: aktuelle Geschwindigkeit gross, Min/Durchschnitt/Max,
// darunter ein Balkendiagramm des Verlaufs (letzte ~60 Samples). Boeen
// (deutlich ueber Durchschnitt) werden rot hervorgehoben.
void drawWindStatsFullscreen(bool windValid, float currentSpeed, float currentAngle) {
  gfx->setFont(&FreeSansBold12pt7b);
  gfx->setTextSize(1);
  gfx->setTextColor(UI_YELLOW, UI_BG);
  gfx->setCursor(6, 20); gfx->print("WINDSTATISTIK");

  if (!windValid) {
    gfx->setTextColor(UI_DARKGREY, UI_BG);
    gfx->setCursor(140, 224); gfx->print("KEIN WIND-SIGNAL");
    gfx->setFont();
    return;
  }
  gfx->setFont();

  float minKn, avgKn, maxKn;
  windSpeedStats(minKn, avgKn, maxKn);
  static const float GUST_MARGIN_KN = 2.5f;   // ab wieviel ueber Durchschnitt gilt's als Boee
  bool isGust = (currentSpeed - avgKn) > GUST_MARGIN_KN;

  // Aktuelle Geschwindigkeit gross, mittig oben - neue Schriftart
  gfx->setFont(&FreeSansBold18pt7b);
  gfx->setTextSize(2);
  char buf[16]; snprintf(buf, 16, "%.1f", currentSpeed);
  int16_t tbx,tby; uint16_t tbw,tbh;
  gfx->getTextBounds(buf,0,0,&tbx,&tby,&tbw,&tbh);
  gfx->setTextColor(isGust ? UI_RED : UI_WHITE, UI_BG);
  gfx->setCursor(240 - (int)tbw/2, 40 + (int)tbh); gfx->print(buf);
  gfx->setFont();
  gfx->setFont(&FreeSansBold12pt7b);
  gfx->setTextSize(1);
  gfx->setTextColor(UI_SOFT_CYAN, UI_BG);
  gfx->setCursor(205, 140); gfx->print("Knoten");
  if (isGust) {
    gfx->setTextColor(UI_RED, UI_BG);
    gfx->setCursor(190, 160); gfx->print("BOE!");
  }
  gfx->setFont();

  // MIN / AVG / MAX Reihe - Werte in der neuen Schriftart
  gfx->setFont(&FreeSansBold12pt7b);
  gfx->setTextSize(1);
  auto stat = [&](int x, const char* label, float val, uint16_t color){
    gfx->setTextColor(UI_SOFT_CYAN, UI_BG);
    gfx->setCursor(x, 200); gfx->print(label);
    char b[16]; snprintf(b,16,"%.1f",val);
    gfx->setTextColor(color, UI_BG);
    gfx->setCursor(x, 235); gfx->print(b);
  };
  stat(50,  "MIN", minKn, UI_WHITE);
  stat(210, "AVG", avgKn, UI_WHITE);
  stat(370, "MAX", maxKn, UI_RED);
  gfx->setFont();

  // Verlaufsgraph: letzte bis zu 60 Samples (~1 Minute), Boeen rot
  const int graphX = 20, graphY = 260, graphW = 440, graphH = 190;
  gfx->drawRect(graphX, graphY, graphW, graphH, UI_CARD_EDGE);
  int nShow = windLupeCount < 60 ? windLupeCount : 60;
  if (nShow > 1) {
    float scaleMax = maxKn * 1.2f;
    if (scaleMax < 5.0f) scaleMax = 5.0f;
    int barW = graphW / nShow;
    for (int i = 0; i < nShow; i++) {
      int idx = (windLupeIdx - nShow + i + WIND_LUPE_SAMPLES * 2) % WIND_LUPE_SAMPLES;
      float v = windSpeedBuf[idx];
      int barH = (int)((v / scaleMax) * (graphH - 4));
      if (barH < 1) barH = 1;
      uint16_t barColor = ((v - avgKn) > GUST_MARGIN_KN) ? UI_RED : UI_YELLOW;
      int bx = graphX + i * barW;
      int by = graphY + graphH - barH - 2;
      gfx->fillRect(bx, by, barW - 1, barH, barColor);
    }
  } else {
    gfx->setFont(&FreeSansBold12pt7b);
    gfx->setTextSize(1);
    gfx->setTextColor(UI_DARKGREY, UI_BG);
    gfx->setCursor(graphX + 100, graphY + graphH/2); gfx->print("Sammle Verlaufsdaten...");
    gfx->setFont();
  }
}

void drawWindRoseFullscreen(int cx,int cy,int r,bool windValid,float windAngle,float windSpeedKn){
  gfx->drawCircle(cx,cy,r,UI_YELLOW);gfx->drawCircle(cx,cy,r-1,UI_YELLOW);
  gfx->drawCircle(cx,cy,r-22,UI_SOFT_CYAN);gfx->drawCircle(cx,cy,r-23,UI_SOFT_CYAN);

  // Backbord/Steuerbord-Faerbung: rechte Haelfte (0-180, Steuerbord) gruen,
  // linke Haelfte (180-360, Backbord) rot - als Hintergrundband, bevor
  // Windlupe und Teilstriche darueber gezeichnet werden
  for (int a = 0; a < 360; a++) {
    uint16_t bandColor = (a > 0 && a <= 120) ? UI_GREEN : (a >= 240 && a < 360) ? UI_RED : UI_BG;
    float ar = deg2rad_f((float)a - 90.0f);
    int x1 = cx + (int)lroundf(cosf(ar) * (r - 1));
    int y1 = cy + (int)lroundf(sinf(ar) * (r - 1));
    int x2 = cx + (int)lroundf(cosf(ar) * (r - 22));
    int y2 = cy + (int)lroundf(sinf(ar) * (r - 22));
    gfx->drawLine(x1, y1, x2, y2, bandColor);
  }

  // Windlupe: farbiger Bogen zeigt die Pendelbreite (Min/Max) des
  // Windwinkels der letzten ca. 2 Minuten, gezeichnet als breites Band
  // im aeusseren Ring - danach ueberzeichnen die normalen Teilstriche
  if (windValid && windLupeCount > 1) {
    float lupeMin, lupeMax;
    windLupeGetRange(windAngle, lupeMin, lupeMax);
    for (float a = lupeMin; a <= lupeMax; a += 1.0f) {
      float ar = deg2rad_f(wrap360(a) - 90.0f);
      int x1 = cx + (int)lroundf(cosf(ar) * (r - 1));
      int y1 = cy + (int)lroundf(sinf(ar) * (r - 1));
      int x2 = cx + (int)lroundf(cosf(ar) * (r - 22));
      int y2 = cy + (int)lroundf(sinf(ar) * (r - 22));
      gfx->drawLine(x1, y1, x2, y2, UI_WHITE);
    }
  }

  for(int a=0;a<360;a+=5){
    float ar=deg2rad_f((float)a-90.0f);
    int x1=cx+(int)lroundf(cosf(ar)*(r-1)),y1=cy+(int)lroundf(sinf(ar)*(r-1));
    int inner=(a%30==0)?(r-22):(a%10==0)?(r-12):(r-7);
    int x2=cx+(int)lroundf(cosf(ar)*inner),y2=cy+(int)lroundf(sinf(ar)*inner);
    gfx->drawLine(x1,y1,x2,y2,(a%30==0)?UI_WHITE:UI_SOFT_CYAN);
  }
  gfx->setTextSize(1);
  for(int a=0;a<360;a+=10){
    char num[8];snprintf(num,8,"%d",a);
    float ar=deg2rad_f((float)a-90.0f);
    gfx->setTextColor(UI_WHITE,UI_BG);
    gfx->setCursor(cx+(int)lroundf(cosf(ar)*(r+12))-7,cy+(int)lroundf(sinf(ar)*(r+12))-5);
    gfx->print(num);
  }
  // Grosse 0/90/180/270-Beschriftung - neue Schriftart, automatisch zentriert
  gfx->setFont(&FreeSansBold18pt7b);
  gfx->setTextSize(1);
  auto lbl=[&](float angle,const char* txt){
    float ar=deg2rad_f(angle-90.0f);
    int16_t tbx,tby; uint16_t tbw,tbh;
    gfx->getTextBounds(txt,0,0,&tbx,&tby,&tbw,&tbh);
    gfx->setTextColor(UI_WHITE,UI_BG);
    gfx->setCursor(cx+(int)lroundf(cosf(ar)*(r-38))-(int)tbw/2, cy+(int)lroundf(sinf(ar)*(r-38))+(int)tbh/2);
    gfx->print(txt);
  };
  lbl(0,"0");lbl(90,"90");lbl(180,"180");lbl(270,"270");
  gfx->setFont();

  gfx->setFont(&FreeSansBold12pt7b);
  gfx->setTextSize(1);
  gfx->setTextColor(UI_YELLOW,UI_BG);gfx->setCursor(6,20);gfx->print("WIND");
  gfx->setFont();

  if(windValid){
    drawArrow(cx,cy,windAngle,r-26,10,UI_YELLOW);
    gfx->fillCircle(cx,cy,8,UI_LIGHTGREY);gfx->drawCircle(cx,cy,8,UI_WHITE);

    // WA oben, WS unten, zentriert ueber/unter dem Mittelpunkt, neue Schriftart
    gfx->setFont(&FreeSansBold12pt7b);
    gfx->setTextSize(1);
    char buf[16];snprintf(buf,16,"WA %03d",((int)lroundf(windAngle))%360);
    int16_t tbx,tby; uint16_t tbw,tbh;
    gfx->getTextBounds(buf,0,0,&tbx,&tby,&tbw,&tbh);
    gfx->setTextColor(UI_WHITE,UI_BG);
    gfx->setCursor(cx-(int)tbw/2,cy-30);gfx->print(buf);
    drawDegreeSymbol(cx+(int)tbw/2+8,cy-30-(int)tbh+3,UI_WHITE,2);

    char spdBuf[16];snprintf(spdBuf,16,"WS %.1fkn",windSpeedKn);
    gfx->getTextBounds(spdBuf,0,0,&tbx,&tby,&tbw,&tbh);
    gfx->setTextColor(UI_YELLOW,UI_BG);
    gfx->setCursor(cx-(int)tbw/2,cy+35);gfx->print(spdBuf);
    gfx->setFont();
  } else {
    gfx->setFont(&FreeSansBold12pt7b);
    gfx->setTextSize(1);
    gfx->setTextColor(UI_DARKGREY,UI_BG);
    gfx->setCursor(cx-90,cy);gfx->print("KEIN WIND-SIGNAL");
    gfx->setFont();
  }
}

void drawStaticUI(){
  gfx->fillScreen(UI_BG);
  gfx->drawRoundRect(8,8,464,48,10,UI_CARD_EDGE);
  gfx->drawRoundRect(8,64,210,50,10,UI_CARD_EDGE);
  gfx->drawRoundRect(8,122,210,44,10,UI_CARD_EDGE);
  gfx->drawRoundRect(8,174,210,82,10,UI_CARD_EDGE);
  gfx->drawRoundRect(8,264,210,70,10,UI_CARD_EDGE);
  gfx->drawRoundRect(8,344,210,128,10,UI_CARD_EDGE);
  gfx->drawRoundRect(226,64,246,408,10,UI_CARD_EDGE);
  gfx->setFont(&FreeSansBold12pt7b);
  gfx->setTextSize(1);gfx->setTextColor(UI_CYAN,UI_BG);
  gfx->setCursor(18,38);gfx->print("MARINE GPS / COMPASS");
  gfx->setFont();
  uiStaticDrawn=true;
}

void drawLCD(bool haveCompass,float headingTrue,float headingMag,
             bool gpsSignalPresent,bool locValid,double lat,double lon,
             int sats,double spdKmh,bool timeValid,
             uint32_t hh,uint32_t mm,uint32_t ss,
             bool cogValid,float cogDeg,bool wpValid,float wpBearDeg){
  (void)headingMag;

  if(viewMode==VIEW_COMPASS_FULL){
    gfx->fillRect(0,0,480,480,UI_BG);
    if(haveCompass&&!isnan(headingTrue)){
      drawCompassRoseFullscreen(240,230,210,headingTrue,cogValid,cogDeg,wpValid,wpBearDeg,locValid,lat,lon,timeValid,hh,mm,ss);
    } else {
      gfx->drawCircle(240,222,210,UI_WHITE);
      gfx->setFont(&FreeSansBold12pt7b);
      gfx->setTextSize(1);
      gfx->setTextColor(UI_WHITE,UI_BG);gfx->setCursor(160,218);gfx->print("KEIN KOMPASS");
      gfx->setFont();
    }
    return;
  }

  if(viewMode==VIEW_WIND_FULL){
    gfx->fillRect(0,0,480,480,UI_BG);
    drawWindRoseFullscreen(240,230,210,windDataValid,windAngleDeg,windSpeedKn);
    return;
  }

  if(viewMode==VIEW_WIND_STATS){
    gfx->fillRect(0,0,480,480,UI_BG);
    drawWindStatsFullscreen(windDataValid,windSpeedKn,windAngleDeg);
    return;
  }

  if(!uiStaticDrawn)drawStaticUI();
  gfx->fillRect(10,10,460,44,UI_BG);
  gfx->setFont(&FreeSansBold12pt7b);
  gfx->setTextSize(1);
  gfx->setTextColor(UI_WHITE,UI_BG);gfx->setCursor(18,38);
  if(locValid)gfx->print("FIX");else if(gpsSignalPresent)gfx->print("GPS");else gfx->print("NODATA");
  if(timeValid){char tb[20];sprintf(tb," %02lu:%02lu:%02lu UTC",(unsigned long)hh,(unsigned long)mm,(unsigned long)ss);gfx->print(tb);}
  else gfx->print(" --:--:-- UTC");
  gfx->setFont();
  gfx->setTextSize(2);
  gfx->fillRect(10,66,206,46,UI_BG);
  {
    char satBuf[16]; snprintf(satBuf,16,"SAT %d",sats);
    gfx->setFont(&FreeSansBold12pt7b);
    gfx->setTextSize(1);
    gfx->setTextColor(UI_WHITE,UI_BG);
    gfx->setCursor(18,90); gfx->print(satBuf);
    gfx->setFont();
  }
  gfx->fillRect(10,124,206,40,UI_BG);
  {
    char spdBuf[24]; snprintf(spdBuf,24,"SPD %.1f km/h",spdKmh);
    gfx->setFont(&FreeSansBold12pt7b);
    gfx->setTextSize(1);
    gfx->setTextColor(UI_WHITE,UI_BG);
    gfx->setCursor(18,148); gfx->print(spdBuf);
    gfx->setFont();
  }
  gfx->fillRect(10,176,206,78,UI_BG);
  gfx->setFont(&FreeSansBold12pt7b);
  gfx->setTextSize(1);
  gfx->setTextColor(UI_WHITE,UI_BG);
  if(locValid){
    printDMSWithDegree(18,200,lat,true,UI_WHITE);
    printDMSWithDegree(18,224,lon,false,UI_WHITE);
  } else {gfx->setCursor(18,212);gfx->print("POS ----");}
  gfx->setFont();
  gfx->fillRect(10,266,206,66,UI_BG);
  gfx->setFont(&FreeSansBold12pt7b);
  gfx->setTextSize(1);
  gfx->setCursor(18,290);gfx->setTextColor(UI_RED,UI_BG);gfx->print("ROT");gfx->setTextColor(UI_WHITE,UI_BG);gfx->print(": HDG");
  gfx->setCursor(18,314);gfx->setTextColor(UI_GREEN,UI_BG);gfx->print("GRN");gfx->setTextColor(UI_WHITE,UI_BG);gfx->print(": COG");
  gfx->setFont();
  gfx->fillRect(10,346,206,124,UI_BG);drawRawRs485Box(18,360,190,104);
  gfx->fillRect(228,66,242,404,UI_BG);
  drawWindRoseMini(340,142,52,windDataValid,windAngleDeg,windSpeedKn);
  if(haveCompass&&!isnan(headingTrue)){drawCompassRose(340,340,82,headingTrue,cogValid,cogDeg,wpValid,wpBearDeg);}
  else{
    gfx->drawCircle(340,340,82,UI_WHITE);
    gfx->setFont(&FreeSansBold12pt7b);
    gfx->setTextSize(1);
    gfx->setTextColor(UI_WHITE,UI_BG);gfx->setCursor(300,340);gfx->print("NO HDG");
    gfx->setFont();
  }
}

// ===================== BE880 GPS =====================
void gpsFlushInput(uint32_t d=200){uint32_t t0=millis();while(millis()-t0<d){while(GPSRS485Serial.available())GPSRS485Serial.read();delay(1);}}

void ubxSend(const uint8_t* p,uint16_t len,bool fl=true){
  uint8_t a=0,b=0;GPSRS485Serial.write(0xB5);GPSRS485Serial.write(0x62);
  for(uint16_t i=0;i<len;i++){a+=p[i];b+=a;GPSRS485Serial.write(p[i]);}
  GPSRS485Serial.write(a);GPSRS485Serial.write(b);GPSRS485Serial.flush();if(fl)delay(80);
}
void ubxCfgMsg(uint8_t mc,uint8_t mi,uint8_t r){uint8_t p[12]={0x06,0x01,0x08,0x00,mc,mi,0,r,0,0,0,0};ubxSend(p,12);}
void ubxCfgRate(uint16_t ms){uint8_t p[10]={0x06,0x08,0x06,0x00,(uint8_t)(ms&0xFF),(uint8_t)((ms>>8)&0xFF),0x01,0x00,0x00,0x00};ubxSend(p,10);}
void ubxCfgPortBaud(uint32_t baud){
  uint8_t p[24]={0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xC0,0x08,0x00,0x00,
    (uint8_t)(baud&0xFF),(uint8_t)((baud>>8)&0xFF),(uint8_t)((baud>>16)&0xFF),(uint8_t)((baud>>24)&0xFF),
    0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00};ubxSend(p,24);
}

bool detectNmeaAtBaud(uint32_t ms,char* sl,size_t ss){
  size_t idx=0;uint32_t t0=millis();
  while(millis()-t0<ms){
    while(GPSRS485Serial.available()){
      char c=(char)GPSRS485Serial.read();
      if(c=='\r')continue;
      if(c=='\n'){sl[idx]='\0';if(sl[0]=='$'&&(strstr(sl,"RMC")||strstr(sl,"GGA")))return true;idx=0;continue;}
      if(idx<ss-1)sl[idx++]=c;else idx=0;
    }
  }sl[0]='\0';return false;
}

// Aktives Auto-Baud: probiert die Kandidaten in bis zu zwei Durchlaeufen
// durch (GPS-Empfaenger nach Kaltstart braucht manchmal etwas laenger,
// bis er ueberhaupt sauber sendet). Gesamtbudget bleibt begrenzt.
bool be880AutoDetect(uint32_t& found, uint32_t overallTimeoutMs = 6000){
  const uint32_t cands[]={115200,38400,9600,57600,19200};
  const size_t nCands = 5;
  char s[128];
  uint32_t t0 = millis();
  int pass = 0;
  while (millis() - t0 < overallTimeoutMs) {
    pass++;
    for(size_t i=0;i<nCands;i++){
      if (millis() - t0 >= overallTimeoutMs) break;
      GPSRS485Serial.end();delay(60);
      GPSRS485Serial.begin(cands[i],SERIAL_8N1,RS485_RX_PIN,RS485_TX_PIN);
      gpsFlushInput(120);
      if(detectNmeaAtBaud(1000,s,128)){found=cands[i];Serial.print("[GPS] ");Serial.println(found);return true;}
    }
    Serial.printf("[GPS] Durchlauf %d ohne Treffer, versuche erneut...\n", pass);
  }
  return false;
}

void be880Configure(uint32_t curBaud){
  GPSRS485Serial.end();delay(60);
  GPSRS485Serial.begin(curBaud,SERIAL_8N1,RS485_RX_PIN,RS485_TX_PIN);
  gpsFlushInput(150);
  ubxCfgMsg(0xF0,0x00,1);ubxCfgMsg(0xF0,0x01,0);ubxCfgMsg(0xF0,0x02,0);
  ubxCfgMsg(0xF0,0x03,0);ubxCfgMsg(0xF0,0x04,1);ubxCfgMsg(0xF0,0x05,0);
  ubxCfgRate(200);
  if(curBaud!=GPS_TARGET_BAUD){
    ubxCfgPortBaud(GPS_TARGET_BAUD);delay(200);
    GPSRS485Serial.end();delay(80);
    GPSRS485Serial.begin(GPS_TARGET_BAUD,SERIAL_8N1,RS485_RX_PIN,RS485_TX_PIN);
    gpsFlushInput(300);gpsActiveBaud=GPS_TARGET_BAUD;
  }else gpsActiveBaud=curBaud;
  Serial.printf("[GPS] Baud %lu\n",(unsigned long)gpsActiveBaud);
}

// ===================== Waypoint =====================
double deg2rad_d(double d){return d*M_PI/180.0;}
double bearingDeg(double lat1,double lon1,double lat2,double lon2){
  double p1=deg2rad_d(lat1),p2=deg2rad_d(lat2),dl=deg2rad_d(lon2-lon1);
  double y=sin(dl)*cos(p2),x=cos(p1)*sin(p2)-sin(p1)*cos(p2)*cos(dl);
  double b=atan2(y,x)*180.0/M_PI;while(b<0)b+=360;while(b>=360)b-=360;return b;
}
double haversineMeters(double lat1,double lon1,double lat2,double lon2){
  const double R=6371000.0;double p1=deg2rad_d(lat1),p2=deg2rad_d(lat2);
  double dp=deg2rad_d(lat2-lat1),dl=deg2rad_d(lon2-lon1);
  double a=sin(dp/2)*sin(dp/2)+cos(p1)*cos(p2)*sin(dl/2)*sin(dl/2);
  return R*2*atan2(sqrt(a),sqrt(1-a));
}

// ===================== WiFi AP =====================
// Startet den Access Point und wartet aktiv, bis er wirklich eine
// gueltige IP vergeben hat, statt eine feste Zeit zu raten.
bool wifiApWaitReady(uint32_t timeoutMs = 3000){
  // WIFI_AP_STA statt WIFI_AP: Access Point (fuer M5Tough) UND
  // gleichzeitig Client-Verbindung zum Windsensor-WLAN moeglich.
  WiFi.mode(WIFI_AP_STA);
  bool started = WiFi.softAP(WIFI_SSID);  // offenes Netzwerk, kein Passwort
  if (!started) {
    Serial.println("[WIFI] softAP() meldete Fehler beim Start");
    return false;
  }
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    IPAddress ip = WiFi.softAPIP();
    if (ip[0] != 0 || ip[1] != 0 || ip[2] != 0 || ip[3] != 0) return true;
    delay(50);
  }
  return false;
}

// Verbindet sich (nicht-blockierend) mit dem WLAN des Windsensors.
// Laeuft im Hintergrund weiter, blockiert den Bootvorgang NICHT - falls
// der Windsensor gerade aus ist, bootet das Board trotzdem normal durch
// und die Windanzeige bleibt einfach "kein Signal", bis er verfuegbar ist.
void windWifiConnect() {
  WiFi.begin(WIND_WIFI_SSID);   // offenes Netzwerk, kein Passwort
  Serial.printf("[WIND-WIFI] Verbinde mit '%s'...\n", WIND_WIFI_SSID);
}

// ===================== Boot Screen =====================
static void bootMsg(const char* msg, uint16_t color = 0xFFFF) {
  Serial.println(msg);
  gfx->setTextSize(1);
  gfx->setTextColor(color, UI_BG);
  gfx->setCursor(5, bootY);
  gfx->print(msg);
  bootY += 14;
  if (bootY > 460) {  // Scroll
    gfx->fillScreen(UI_BG);
    bootY = 10;
  }
}
static void bootMsgf(uint16_t color, const char* fmt, ...) {
  char buf[80];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  bootMsg(buf, color);
}

void setup(){
  Serial.begin(NMEA_BAUD);delay(300);
  pinMode(LED_BUILTIN,OUTPUT);digitalWrite(LED_BUILTIN,LOW);

  // Grosszuegige Kaltstart-Wartezeit: bei einem echten Spannungs-Neustart
  // (Stecker rein) ist die Einschwingzeit der Versorgungsspannung fuer
  // Touch/Expander kuerzer verfuegbar als direkt nach einem Firmware-
  // Upload (dort laeuft ueber USB schon laenger Spannung an). Ohne
  // diese Wartezeit werden Touch UND Expander bei einem Kaltstart
  // manchmal gar nicht gefunden.
  delay(3000);

  // WS_CH32_IO::begin() (in touchWaitForever/gt911_init aufgerufen) macht
  // Bus-Recovery, I2C-Start und die komplette Reset-Sequenz selbststaendig -
  // auf dem GLEICHEN Bus wie Touch (GPIO15/7), kein separater Expander-Bus
  // mehr noetig.
  Serial.println("[BOOT] Touch GT911 (inkl. CH32V003 IO-Chip) Reset...");
  touchWaitForever(false);
  Serial.println("[BOOT] Touch GT911 OK");

  // GFX erst jetzt erstellen
  bus = new Arduino_SWSPI(GFX_NOT_DEFINED,42,2,1,GFX_NOT_DEFINED);
  rgbpanel = new Arduino_ESP32RGBPanel(
    40,39,38,41, 46,3,8,18,17, 14,13,12,11,10,9, 5,45,48,47,21,
    1,10,8,50,1,10,8,20);
  gfx = new Arduino_RGB_Display(
    480,480,rgbpanel,2,true,bus,GFX_NOT_DEFINED,
    st7701_type1_init_operations,sizeof(st7701_type1_init_operations));
  gfx->begin();
  gfx->fillScreen(UI_BG);

  // Boot Header
  gfx->setTextSize(2);
  gfx->setTextColor(UI_CYAN, UI_BG);
  gfx->setCursor(5, bootY);
  gfx->print("Marine GPS v");
  gfx->print(FW_VERSION);
  bootY += 22;
  gfx->setTextSize(1);
  gfx->setTextColor(UI_SOFT_CYAN, UI_BG);
  gfx->setCursor(5, bootY);
  gfx->print("(C) Peter Schulte");
  bootY += 18;
  gfx->drawLine(0, bootY, 480, bootY, UI_CARD_EDGE);
  bootY += 8;

  bootMsg("[ OK ] Display 480x480", UI_GREEN);
  bootMsg("[ OK ] Touch GT911 (CH32V003)", UI_GREEN);

  // GPS: aktives Auto-Baud mit Gesamt-Timeout (mehrere Durchlaeufe moeglich)
  bootMsg("[ .. ] GPS Auto-Baud...", UI_YELLOW);
  uint32_t detBaud=0;
  if(be880AutoDetect(detBaud, 6000)){
    be880Configure(detBaud);
    char buf[40];snprintf(buf,40,"[ OK ] GPS BE880 @ %lu Bd",(unsigned long)gpsActiveBaud);
    bootMsg(buf, UI_GREEN);
  } else {
    gpsActiveBaud=GPS_TARGET_BAUD;
    GPSRS485Serial.end();delay(60);
    GPSRS485Serial.begin(GPS_TARGET_BAUD,SERIAL_8N1,RS485_RX_PIN,RS485_TX_PIN);
    bootMsg("[WARN] GPS Fallback 115200", UI_YELLOW);
  }

  // WiFi AP: aktiv warten bis IP tatsaechlich vergeben ist
  bootMsg("[ .. ] WiFi Access Point...", UI_YELLOW);
  if (wifiApWaitReady(3000)) {
    bootMsg("[ OK ] AP: Troll32_GPS", UI_GREEN);
    char buf[40];snprintf(buf,40,"[ OK ] IP: %s",WiFi.softAPIP().toString().c_str());bootMsg(buf,UI_GREEN);
  } else {
    bootMsg("[WARN] WiFi AP evtl. nicht bereit", UI_YELLOW);
  }
  nmeaServer.begin();nmeaServer.setNoDelay(true);
  bootMsg("[ OK ] NMEA TCP :10110", UI_GREEN);

  // Windsensor: Verbindung im Hintergrund starten (blockiert nicht),
  // UDP-Listener sofort bereitstellen
  windWifiConnect();
  windUdp.begin(WIND_UDP_PORT);
  bootMsg("[ .. ] Windsensor: Verbinde im Hintergrund...", UI_SOFT_CYAN);

  // Kompass: unbegrenzt warten, bis er gefunden wird (gibt nie auf)
  compassWaitForever();
  bootMsg(compassType==COMPASS_HMC5883L ? "[ OK ] HMC5883L" : "[ OK ] QMC5883L", UI_GREEN);

  gfx->drawLine(0, bootY+2, 480, bootY+2, UI_CARD_EDGE);
  bootY += 10;
  bootMsg("[ READY ] Starte in...", UI_CYAN);

  for(int i=3; i>0; i--) {
    gfx->fillRect(250, bootY, 50, 16, UI_BG);
    gfx->setTextSize(1);
    gfx->setTextColor(UI_CYAN, UI_BG);
    gfx->setCursor(250, bootY);
    gfx->print(i);
    gfx->print(" ...");
    delay(1000);
  }
}

uint32_t lastUpdate=0;

// ===================== Loop =====================
void loop(){
  handleTouch();
  updateWindFromUDP();

  if(!nmeaClient||!nmeaClient.connected()){
    WiFiClient nc=nmeaServer.available();
    if(nc){nmeaClient=nc;nmeaClient.setNoDelay(true);}
  }

  while(GPSRS485Serial.available()){
    int c=GPSRS485Serial.read();
    gps.encode((char)c);gpsByteCount++;lastGpsByteMs=millis();appendGpsRawChar((char)c);
  }

  int16_t mx=0,my=0,mz=0;
  bool haveCompass=(compassType!=COMPASS_NONE);
  float headingTrue=NAN,headingMag=NAN;

  // Kompass Auto-Recovery (falls er waehrend des Betriebs ausfaellt)
  static uint32_t lastCompassCheck=0;
  if(!haveCompass&&millis()-lastCompassCheck>2000){
    lastCompassCheck=millis();
    i2cBusRecovery();
    detectCompass();haveCompass=(compassType!=COMPASS_NONE);
  }

  if(haveCompass){
    if(compassType==COMPASS_HMC5883L)hmcRead(mx,my,mz);else qmcRead(mx,my,mz);
    headingTrue=filterNeedleDeg(filterHeadingDeg(computeHeadingTrueishDeg(mx,my)));
    headingMag=computeHeadingMagDeg(mx,my);
  }

  bool wifiOK = (WiFi.softAPgetStationNum() > 0);  // AP: Client verbunden?
  bool gpsSignalPresent=(millis()-lastGpsByteMs)<GPS_SIGNAL_TIMEOUT_MS;
  updateStatusLED(wifiOK,gps.location.isValid(),haveCompass,
    !gps.location.isValid()||!gps.satellites.isValid()||gps.satellites.value()<4);

  if(millis()-lastUpdate>=UPDATE_MS){
    lastUpdate=millis();
    bool lv=gps.location.isValid();
    double lat=lv?gps.location.lat():0.0,lon=lv?gps.location.lng():0.0;
    int sats=gps.satellites.isValid()?(int)gps.satellites.value():0;
    double spd=gps.speed.isValid()?gps.speed.kmph():0.0;
    bool tv=gps.time.isValid();
    uint32_t hh=tv?gps.time.hour():0,mm=tv?gps.time.minute():0,ss=tv?gps.time.second():0;
    bool cv=gps.course.isValid();float cog=cv?(float)gps.course.deg():0.0f;
    bool wpValid=false;float wpBear=0;
    if(WP_ENABLED&&lv){wpValid=true;wpBear=(float)bearingDeg(lat,lon,WP_LAT,WP_LON);}
    drawLCD(haveCompass,headingTrue,headingMag,gpsSignalPresent,lv,lat,lon,sats,spd,tv,hh,mm,ss,cv,cog,wpValid,wpBear);
    if (windDataValid) windSampleAdd(windAngleDeg, windSpeedKn);
    emitGNRMC();emitGNGGA();emitGNVTG();emitHCHDM(headingMag,haveCompass);emitWIMWV(windDataValid,windAngleDeg,windSpeedKn);
  }
}

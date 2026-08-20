# Projektbeschreibung: Marine GPS/Kompass/Wind-Display "Ylvi"

**Autor:** Peter Schulte
**Firmware-Version:** 1.09
**Plattform:** Waveshare ESP32-S3-Touch-LCD-4

---

## 1. Projektübersicht

Ein eigenständiges Marine-Multifunktionsdisplay für das Boot *Ylvi* (Troll 32), das GPS-Position, Magnetkompass-Heading und Winddaten (von einem separaten Windsensor-Projekt) auf einem 480×480-Touchscreen darstellt und gleichzeitig als NMEA0183-Datenquelle für andere Geräte (z. B. M5Tough-BBN-Display) über WLAN und USB/Serial dient.

**Vier durchschaltbare Ansichten** (per Touch, zyklisch):
1. **Normalansicht** – Übersicht mit SAT/SPD/Position, Mini-Kompass, Mini-Windrose, NMEA-Fenster
2. **Kompass-Vollbild** – große Kompassrose mit Heading, COG, Position
3. **Wind-Vollbild** – große Windrose mit Backbord/Steuerbord-Färbung, Windlupe (Pendelbreite-Anzeige)
4. **Windstatistik** – aktuelle Geschwindigkeit, Min/Durchschnitt/Max, Böen-Erkennung (rot markiert), Verlaufsgraph

---

## 2. Hardware

| Komponente | Modell / Typ | Anbindung |
|---|---|---|
| Hauptboard | Waveshare ESP32-S3-Touch-LCD-4 | – |
| Display | 480×480 RGB-Panel (ST7701-Treiber) | Arduino_ESP32RGBPanel |
| Touch-Controller | GT911 | I2C, Adresse `0x5D` |
| IO-Controller (Touch-Reset, Backlight etc.) | **CH32V003**-Mikrocontroller | I2C, Adresse `0x24` |
| GPS-Empfänger | u-blox BE880 | RS485 / UBX-Protokoll |
| Kompass | HMC5883L **oder** QMC5883L (automatische Erkennung) | I2C |
| Windsensor | separates DIY-ESP32-Projekt | WLAN/UDP, NMEA0183 `$--MWV` |

### Pin-Belegung

| Funktion | GPIO |
|---|---|
| RS485 GPS RX | 43 |
| RS485 GPS TX | 44 |
| I2C SDA (Touch/Kompass/CH32V003) | 15 |
| I2C SCL (Touch/Kompass/CH32V003) | 7 |
| GT911 Touch-Interrupt (INT) | 4 |
| RGB-Panel (DE/VSYNC/HSYNC/PCLK/R/G/B) | siehe `Arduino_ESP32RGBPanel`-Konstruktor im Sketch |

> **Wichtiger Hinweis zur Board-Revision:** Ältere Waveshare-Boards dieser Familie (4.3", 5", 7") nutzen einfache Register-IO-Expander (CH422G oder TCA9554PWR). **Diese** Board-Revision verwendet stattdessen einen eigenständigen **CH32V003**-Mikrocontroller für Touch-Reset, Backlight & Co. – mit einem eigenen Registerprotokoll (`0x02`=Richtung, `0x03`=Ausgabe, statt einfacher GPIO-Register). Der Treiber dafür liegt als eigene Datei `WS_CH32_IO.h`/`.cpp` im Sketch-Ordner (Quelle: offizielles Waveshare-Demo-Paket, angepasst).

---

## 3. Netzwerk-Konfiguration

Das Board läuft im **`WIFI_AP_STA`**-Modus – Access Point **und** WLAN-Client gleichzeitig:

| Rolle | SSID | Passwort | Zweck |
|---|---|---|---|
| Access Point (eigenes Netz) | `Troll32_GPS` | offen (kein Passwort) | M5Tough-Display & andere NMEA-Clients verbinden sich hier |
| WLAN-Client | `Ylvi_wind` | offen (kein Passwort) | Empfang der Winddaten vom separaten Windsensor-Projekt |

**NMEA0183-Ausgabe:**
- **Serial/USB:** 115200 Baud
- **TCP-Server:** Port `10110` (IP des Access Points: `192.168.4.1`)
- **UDP-Empfang Windsensor:** Port `10110`

**Ausgegebene Sätze** (1× pro Sekunde): `$GNRMC`, `$GNGGA`, `$GNVTG`, `$HCHDM`, `$WIMWV`

---

## 4. Arduino-IDE-Werkzeugeinstellungen

| Einstellung | Wert |
|---|---|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | **Enabled** |
| CPU Frequency | 240MHz (WiFi/BT) |
| Flash Mode | QIO |
| Flash Size | **16MB (128Mb)** |
| PSRAM | **OPI PSRAM** |
| Partition Scheme | ein zu 16 MB passendes Schema (**nicht** "Default 4MB") |
| Upload Speed | 921600 |
| JTAG Adapter | Disabled |
| Core Debug Level | None |
| Erase All Flash Before Sketch Upload | Disabled (nur bei Bedarf zur Fehlersuche aktivieren) |

### Board-Paket-Version
- **esp32 (by Espressif Systems): Version 2.0.17** – bewusst gepinnt, nicht die neueste Version

---

## 5. Benötigte Bibliotheken

| Bibliothek | Version | Bemerkung |
|---|---|---|
| **GFX Library for Arduino** (Arduino_GFX, moononournation) | **exakt 1.3.7** | ⚠️ Neuere Versionen sind mit ESP32-Core 2.0.17 **inkompatibel** (ESP-IDF-API-Änderungen) – Downgrade über Library Manager falls nötig |
| **TinyGPSPlus** | aktuell | GPS-NMEA-Parsing |
| **SensorLib** (lewisxhe) | aktuell (0.4.x) | Touch-Punkt-Auslesung (`TouchDrvGT911`) |
| WiFi, WiFiUdp | im ESP32-Core enthalten | – |

---

## 6. Dateien im Sketch-Ordner

Alle folgenden Dateien müssen **im selben Ordner** liegen:

| Datei | Zweck |
|---|---|
| `GPS_Compass_Touch_GT911_robust.ino` | Hauptsketch |
| `WS_CH32_IO.h` / `WS_CH32_IO.cpp` | Treiber für den CH32V003-IO-Controller (Touch-Reset etc.) |
| `FreeSansBold18pt7b.h` | Große Schriftart (Heading-Zahl, Windgeschwindigkeit) |
| `FreeSansBold12pt7b.h` | Mittlere Schriftart (Labels, Position, Legenden) |

---

## 7. Wichtige technische Besonderheiten

- **I2C-Bus-Recovery:** Beim Boot und im laufenden Betrieb wird der I2C-Bus per manuellem SCL-Toggle aktiv freigeräumt, falls er durch einen hängenden Sensor blockiert ist.
- **Selbst-Neustart:** Falls der Touch-Chip nach 8 Sekunden nicht antwortet, führt sich der ESP32 per `ESP.restart()` selbst neu – entspricht softwareseitig einem manuellen Reset-Tastendruck.
- **Aktive INT-Pin-Steuerung:** Der GT911-Touch-Reset erfordert, dass der INT-Pin während der TOUCH_RST-Freigabe aktiv auf HIGH gehalten wird (nicht nur als Eingang belassen) – sonst kalibriert der Chip falsch.
- **Gradsymbole:** Die verwendeten Bold-Schriftarten decken nur ASCII bis `0x7E` ab (kein „°"-Zeichen). Gradsymbole werden deshalb separat als kleine Kreise gezeichnet (`drawDegreeSymbol()`).
- **Kaltstart-Wartezeit:** 3 Sekunden Verzögerung ganz am Anfang von `setup()`, da ein echter Spannungs-Neustart eine längere Einschwingzeit braucht als ein Neustart direkt nach einem Firmware-Upload.
- **Windsimulation:** Für Tests ohne angeschlossenen Windsensor gibt es einen Simulationsmodus (`#define WIND_SIMULATE true/false` im Sketch).

---

## 8. Bekannte Grenzen / offene Punkte

- Windlupe und Böen-Erkennung basieren auf einem 2-Minuten-Verlaufspuffer (120 Samples à 1 Sekunde) – bei Neustart des Boards ist dieser Verlauf leer und füllt sich erst nach und nach.
- Referenz R/T (relativ zum Bug / theoretisch) wird aus dem `$--MWV`-Satz aktuell nicht unterschieden, beides wird gleich behandelt.

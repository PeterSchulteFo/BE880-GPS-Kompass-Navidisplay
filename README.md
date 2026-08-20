[README.md](https://github.com/user-attachments/files/31265736/README.md)
# BE880-GPS-Kompass-Navidisplay# BE880 GPS / Kompass Navigationsdisplay – ESP32-S3

Marine-Navigationsdisplay für mein Boot **Ylvi** (Troll 32), gebaut auf einem
**Waveshare ESP32-S3-Touch-LCD-4"** Board (480×480, ST7701 RGB-Panel).

Der Sketch liest GPS-Daten über RS485 (BE880-Empfänger), ermittelt die
Kompassrichtung über einen HMC5883L- oder QMC5883L-Sensor, zeigt beides
auf dem 4"-Display an und gibt die Daten gleichzeitig als NMEA 0183 über
USB-Seriell **und** über WLAN (TCP Port 10110) aus – kompatibel zu
OpenCPN, SignalK & Co.

## Funktionen

- 5 s Startbildschirm, danach 2 s WiFi/IP/Kompass-Info, dann Normalbetrieb
- GPS-Empfang über RS485 mit automatischer Baudraten-Erkennung (BE880)
- Automatische Erkennung von HMC5883L **oder** QMC5883L Kompass-IC
- Geglättete Kompassnadel (Heading-Filter, kein Zittern der Anzeige)
- NMEA-Ausgabe (GNRMC etc.) parallel über USB und WLAN/TCP (Port 10110)
- RS485-Rohdaten-Anzeige auf dem Display (Debug)
- Status-LED mit eindeutiger Blink-Codierung (siehe unten)
- Deklinationskorrektur einstellbar im Code

## Hardware

| Komponente | Beschreibung |
|---|---|
| Board | Waveshare ESP32-S3-Touch-LCD-4" (ST7701, 480×480 RGB-Panel) |
| GPS | BE880-Modul über RS485 |
| Kompass | HMC5883L oder QMC5883L (I²C, automatische Erkennung) |
| Display | 480×480 RGB TFT (integriert) |

### Pinbelegung

| Funktion | Pin |
|---|---|
| RS485 RX | GPIO 43 |
| RS485 TX | GPIO 44 |
| I²C SDA | GPIO 15 |
| I²C SCL | GPIO 7 |

Die Display-/RGB-Panel-Pins sind fest im Sketch für das Waveshare-Board
hinterlegt (siehe `bus`/`rgbpanel`-Definition im Code).

> Im Ordner `docs/images/` liegt zusätzlich ein Pin-Referenzblatt für ein
> ESP32-S3-Zero-Board (z. B. für ein separates Zusatzmodul / den
> Windsensor) – nicht identisch mit dem hier verbauten Touch-LCD-4-Board.

## Status-LED

| Verhalten | Bedeutung |
|---|---|
| 🔴 schnelles Blinken | WLAN-Problem |
| 🟡 langsames Blinken | GPS sucht Satelliten |
| 🔵 Doppelblinken | Kompass nicht erkannt (Kabel prüfen!) |
| ⚪ kurzes Blitzen | GPS hat Fix, aber keine Bewegung |
| 🟢 Dauerlicht | Alles perfekt – Daten stabil |

## Benötigte Arduino-Bibliotheken

Installierbar über den Arduino Library Manager:

- [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)
- [Arduino_GFX_Library](https://github.com/moononournation/Arduino_GFX) (moononournation)
- `Wire` und `WiFi` (im ESP32-Board-Package enthalten)

**Board-Package:** ESP32 (Espressif) – Board "ESP32S3 Dev Module" bzw. die
passende Waveshare-Variante, PSRAM je nach Board-Ausstattung aktivieren.

## Einrichtung

1. Repository klonen bzw. Ordner öffnen.
2. Im Sketch-Ordner `config.h.example` zu `config.h` kopieren:
   ```
   cp config.h.example config.h
   ```
3. In `config.h` die eigenen WLAN-Zugangsdaten eintragen:
   ```cpp
   const char* WIFI_SSID = "DEIN_WLAN_NAME";
   const char* WIFI_PASS = "DEIN_WLAN_PASSWORT";
   ```
   `config.h` ist in `.gitignore` eingetragen und wird **nicht** mit
   eingecheckt.
4. Benötigte Bibliotheken installieren (siehe oben).
5. Sketch auf das ESP32-S3-Touch-LCD-4 Board flashen.
6. NMEA-Daten stehen danach über USB-Seriell **und** über
   `tcp://<Board-IP>:10110` zur Verfügung (z. B. in OpenCPN als
   Netzwerkverbindung einrichten).

## Projektstruktur

```
.
├── ESP32-S3-Touch-LCD-4_BE880_NEW/
│   ├── ESP32-S3-Touch-LCD-4_BE880_NEW.ino
│   └── config.h.example
├── docs/
│   └── images/
│       └── pin_definition_esp32-s3-zero.png
├── .gitignore
├── LICENSE.md
└── README.md
```

## Status

Aktuelle Firmware-Version: **V2.2** – Marine UI, gestylte Kompassnadel,
Status-Ausgabe. Touch-Funktion wurde bewusst entfernt (reine
Anzeige-/Datenlogik).

## Lizenz

Privates Projekt – siehe [LICENSE.md](LICENSE.md). Nicht für kommerzielle
Nutzung oder Weiterverbreitung freigegeben.

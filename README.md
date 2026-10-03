# AHRS – STM32F446 mit GNSS-gestütztem Error-State-EKF

Firmware für ein selbstgebautes AHRS (WeAct STM32F446RE Core Board) als Datenquelle für
[OpenEFIS](https://github.com/dansparker/open-efis): Lage, Kurs, Drehraten, Druckhöhe, Steigrate,
GNSS-Daten und UTC-Zeit über CAN-Bus (CANaerospace, ADR 0002).

| Sensor | Baustein | Bus |
|---|---|---|
| Beschleunigung + Drehrate | LSM6DSOX (208 Hz, ±8 g, 500 °/s) | I2C1 PB6/PB7 |
| Magnetometer | LIS3MDL (80 Hz, ±4 G, UHP) | I2C1 |
| Barometer | MS5611 (GY-63, OSR 4096) | SPI1 PA4–PA7 |
| Differenzdruck (nur Auslesen) | MS4525DO | I2C2 PB10/PC12 |
| GNSS | u-blox, UBX-NAV-PVT 10 Hz, 115200 Bd | USART1 PA9/PA10 |
| CAN | ATA6561, 500 kbit/s | CAN1 PB8/PB9, STB PB4 |

Pinbelegung und Einbaulage: [`firmware/src/board.h`](firmware/src/board.h).

## Aufbau

```
core/      hardwareunabhängig, auf dem PC getestet
  eskf.*      17-Zustands-Error-State-EKF (Position, Geschwindigkeit, Lage, Accel-/Gyro-Bias,
              Baro-Bias, Missweisung), nur skalare Updates, keine Matrixinversion
  ahrs.*      Ausrichtung, GNSS-Überwachung und -Ausfall, Pseudo-Messungen, Magnetometer-Gating,
              Windschätzung, Ausgabe
  magcal.*    Online-Kalibrierung des Magnetometers
  ubx.*       UBX-Parser (NAV-PVT) und Empfängerkonfiguration (M8 und M9/M10)
  canas.*     CANaerospace-Kodierung, Identifikationsdienst (ID 128/129)
  can_out.*   Abbildung auf die Identifier von OpenEFIS
  baro.*      MS5611-Kompensation (inkl. 2. Ordnung, CRC), ISA-Druckhöhe
firmware/  STM32-HAL: Takt 180 MHz, Treiber, CAN-Sendepuffer, GNSS-UART, Flash-Ablage, Watchdog
tests/     Unit-Tests und Flugsimulation (läuft in der CI mit AddressSanitizer/UBSan)
```

## Sensorfusion

* **Prädiktion** mit 208 Hz (Strapdown, Quaternion), Kovarianz voll 17×17.
* **GNSS** (10 Hz, `GNSS_RATE_HZ`): Position und Geschwindigkeit NED als skalare Updates mit χ²-Gate; Genauigkeit aus
  `hAcc/vAcc/sAcc`. Erste Position setzt den Ursprung; ab 20 km wird er nachgeführt.
* **Barometer**: Druckhöhe mit Bias-Zustand (Baro − GNSS-Höhe); daraus Höhe und Steigrate.
* **Magnetometer**: nur als Kurs-Messung (verändert Roll/Pitch nicht), mit Plausibilitätsprüfung
  von Feldstärke und Inklination gegen Langzeitmittel und χ²-Gate (Störungen durch Funkgerät,
  Pitot-Heizung usw. werden verworfen). Die **Missweisung** ist ein Filterzustand und wird mit GNSS
  in Kurven geschätzt (Startwert `MAG_DECLINATION_DEG`); gesendet wird der **missweisende Kurs**.
* **Stillstand** (ZUPT): ruhige Drehrate, ruhige und *richtungsstabile* Beschleunigung – eine
  Richtungsänderung ohne Drehung ist Linearbeschleunigung (Startlauf) und beendet den Stillstand.

### GNSS-Ausfall

| Zustand | Bedingung | Verhalten |
|---|---|---|
| `GNSS_OK` | 3D-Fix, ≥ 5 Satelliten, hAcc < 15 m, sAcc < 2 m/s | volle Stützung |
| `GNSS_COAST` | > 1,5 s keine gültige Lösung | reine Trägheitsnavigation (5 s) |
| `GNSS_LOST` | danach | Pseudo-Messungen: Luftgeschwindigkeit liegt in Rumpflängsachse (kein Schiebewinkel, gelernter Anstellwinkel), Betrag = Fahrtmesser bzw. geschätzte Fahrt |
| Wiederkehr | nach > 30 s Ausfall | Position/Geschwindigkeit neu gesetzt, Lageunsicherheit erhöht |

Solange GNSS da ist, schätzt ein kleines RLS aus dem **Windreieck** in Kurven Wind und Fahrt
(`v_gnss = V_a·(cos ψ, sin ψ) + w`) sowie den mittleren Anstellwinkel. Im Ausfall bekommt die
Pseudo-Messung so die richtige Bezugsgeschwindigkeit und kann im Kurvenflug Zentripetal- und
Schwerebeschleunigung trennen. Die Sensor-Biase sind dabei „Consider“-Zustände (Schmidt-Kalman):
das Näherungsmodell darf sie nicht verstellen.

Ergebnis der Simulation (600 s, Startlauf, Steigflug, Kurven mit 20–30° Querneigung, Wind 8 m/s,
GNSS-Ausfall 120 s mit Kurven):

| Szenario | Lagefehler max (RMS) | Kursfehler max |
|---|---|---|
| mit GNSS, 120 s Ausfall | 1,5° (0,3°), im Ausfall 0,9° | 1,2° |
| nie GNSS, mit Fahrtmesser | 2,9° | 6,7° |
| nie GNSS, ohne Fahrtmesser (Notbetrieb) | 7,0° | 17° |

## Magnetometer-Kalibrierung

Zur Frage, ob es bessere Varianten als das Paper gibt (Cao/Xu/Xu, *Sensors* 2020, 20, 535):
Das RLS-Verfahren dort braucht nur Magnetometerdaten, setzt aber voraus, dass das Feld aus **allen
Richtungen** gemessen wird. Im Flugzeug liegt der Feldvektor (Inklination ~64°) im normalen Flug
nur in einem engen Kegel – der volle Ellipsoid-Fit ist dann schlecht konditioniert und liefert
Unsinn. Deshalb zwei Verfahren nebeneinander:

1. **Ellipsoid-RLS** (wie im Paper, Hart- und Weicheisen): normiert auf den x²-Koeffizienten (auch
   dann stabil, wenn der Offset so groß wie das Feld ist), Abtastwerte nach Richtung in 26 Fächer
   mit Kontingent einsortiert, Lösung nur bei ausreichender räumlicher Verteilung, plausiblem
   Ellipsoid (Achsverhältnis < 2) und Residuum < 3 %. Gedacht für das Drehen der Einheit/des
   Flugzeugs am Boden; die Lösung wird im Flash gespeichert (nur im Stillstand, wegen des
   blockierenden Löschens).
2. **Lagegestützte Hart-Eisen-Nachführung** (besser für den Flugbetrieb): Mit der GNSS-gestützten
   Lage gilt `m = Rᵀ·B + Δo` – linear in Erdfeld und Rest-Offset. Ein 6-Zustands-RLS mit Vergessen
   schätzt Δo im Kurvenflug, wo der Ellipsoid-Fit nichts liefert.

Getestet: Ellipsoid-Offset auf 0,02 µT genau, Feldbetrag danach auf 0,3 % konstant; ebener Flug
liefert erwartungsgemäß keine Lösung; die Nachführung findet einen Rest-Offset von (6, −4) µT auf
0,15 µT.

## CAN-Bus (CANaerospace, Node-ID 7)

| Rate | Identifier |
|---|---|
| 50 Hz | 311 Pitch, 312 Roll, 1069 Kurs (missweisend), 301 Querbeschleunigung, 303 Nickrate, 305 Gierrate, 322 Druckhöhe (1013,25), 314 Steigrate |
| 10 Hz | 1036/1037 Lat/Lon, 1038 Höhe (Ellipsoid), 1039 Grundgeschwindigkeit, 1040 Track, 1048 Fix, 1800 Satelliten, 1121 Missweisung |
| 10 Hz | **1200 UTC** (UCHAR4: h, min, s, 0), **1201 Datum** (UCHAR4: Tag, Monat, Jahr % 100, Jahr / 100) – auch ohne Positionslösung, sobald der Empfänger eine gültige Zeit hat |

Werte ohne Gültigkeit werden nicht gesendet (OpenEFIS zeigt dann rotes X/Striche). Anfragen des
Identifikationsdienstes (ID 128) werden auf ID 129 beantwortet.

## Bauen und Testen

```sh
make -C tests                     # Unit-Tests + Flugsimulation (gcc)
sh firmware/fetch_drivers.sh      # ST CMSIS + HAL holen
make -C firmware                  # arm-none-eabi-gcc -> firmware/build/ahrs.{elf,bin,hex}
```

Die CI (GitHub Actions) macht beides und stellt die Firmware als Artefakt bereit. Flashen mit
**`ahrs.hex` oder `ahrs.elf`** (z. B. `STM32_Programmer_CLI -c port=SWD -w ahrs.hex -rst` oder
`st-flash --format ihex write ahrs.hex`). Nicht die `.bin` verwenden: sie füllt die Lücke ab
0x08004000 mit Nullen und löscht damit die gespeicherte Magnetometer-Kalibrierung (Sektor 1).

## Offene Punkte / Annahmen

* **Auf echter Hardware noch nicht gelaufen** – nur Simulation und Build.
* **I2C1 auf PB6/PB7** angenommen (aus dem Schaltplan nicht eindeutig; CAN1 belegt PB8/PB9).
* **Einbaulage** `IMU_MOUNT`/`MAG_MOUNT` in `board.h` an die Achspfeile des Breakouts anpassen.
* **PB11 (J2)**: Beim STM32F446RE im 64-Pin-Gehäuse ist dieser Pin VCAP_1 – kein GPIO.
* **Zeit-Identifier 1200/1201**: Belegung nach CANaerospace-Tabelle, bitte mit der Spezifikation
  abgleichen. OpenEFIS-Seite: [open-efis#27](https://github.com/dansparker/open-efis/pull/27).
* **MS4525DO** an I2C2 (J1, 0x28, ±1 psi Typ A in `board.h`): wird mit 20 Hz ausgelesen
  (`airspeed_read()`, Werte in `g_diff_pressure_pa`/`g_pitot_temp_c`), fließt aber bewusst **nicht**
  in die Fusion ein und wird nicht gesendet. `ahrs_airspeed()` wäre die Schnittstelle dafür.

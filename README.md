# 3D LIDAR Scanner - Installations- und Nutzungsanleitung

## Hardware-Voraussetzungen

- Arduino Micro mit Servo an Pin 9
- RPLIDAR A1M8
- Servo-Winkelbereich: 45° - 135° (90° = horizontal)
- Windows PC mit PyCharm

## Software-Installation

### 1. Python-Pakete installieren

```bash
pip install pyserial
pip install rplidar-roboticia
pip install numpy
pip install open3d
```

### 2. COM-Ports herausfinden

**Arduino:**
- Arduino IDE öffnen → Tools → Port → COM-Port notieren (z.B. COM3)

**LIDAR:**
- Geräte-Manager öffnen → Anschlüsse (COM & LPT)
- Suche nach "Silicon Labs CP210x" oder ähnlich
- COM-Port notieren (z.B. COM4)

### 3. Konfiguration anpassen

Datei `config.json` öffnen und Ports anpassen:

```json
{
  "arduino": {
    "port": "COM3",      ← Dein Arduino-Port
    "baudrate": 115200
  },
  "lidar": {
    "port": "COM4"       ← Dein LIDAR-Port
  },
  "scan": {
    "angle_start": 45,   ← Startwinkel
    "angle_end": 135,    ← Endwinkel
    "angle_step": 5,     ← Schrittweite in Grad
    "duration_per_position": 2.0  ← Scan-Dauer pro Position
  }
}
```

## Arduino-Code hochladen

1. Öffne `arduino_servo_3d_scan.ino` in Arduino IDE
2. Board auswählen: Tools → Board → Arduino Micro
3. Port auswählen: Tools → Port → (dein COM-Port)
4. Upload (Strg+U)

## Programme einzeln testen

### Test 1: Arduino-Verbindung

```bash
python arduino_controller.py
```

**Erwartete Ausgabe:**
```
=== Arduino Servo Controller Test ===

✅ Arduino verbunden auf COM3
   Position: 90°
   Bereich: 45° - 135°

--- Test 1: Home Position ---
🏠 Fahre zu Home-Position...
✅ Position erreicht: 90°
...
```

### Test 2: LIDAR-Verbindung

```bash
python lidar_scanner.py
```

**Erwartete Ausgabe:**
```
=== RPLIDAR Scanner Test ===

✅ LIDAR verbunden auf COM4
   Modell: 24
   Firmware: 1.29
   Hardware: 7
   Gesundheit: Good

📡 Scanne für 3.0 Sekunden...
✅ Scan abgeschlossen: 1243 Messpunkte
...
```

## Kompletten 3D-Scan durchführen

```bash
python scan_3d_coordinator.py
```

**Ablauf:**
1. Programm verbindet mit Arduino und LIDAR
2. Warte auf ENTER-Taste zum Start
3. Servo fährt zu Startposition (45°)
4. Für jede Position:
   - Servo bewegt sich
   - LIDAR scannt für 2 Sekunden
   - Daten werden gespeichert
5. Nach Abschluss: Dateien werden gespeichert
6. Servo kehrt zu Home-Position (90°) zurück

**Ausgabe-Dateien:**
- `scan_3d_YYYYMMDD_HHMMSS.txt` - Punktwolke als CSV (X,Y,Z)
- `scan_3d_YYYYMMDD_HHMMSS.xyz` - Punktwolke im XYZ-Format
- `scan_3d_YYYYMMDD_HHMMSS_stats.json` - Scan-Statistiken

## Scan-Parameter anpassen

In `config.json`:

**Feinerer Scan (mehr Details, länger):**
```json
"scan": {
  "angle_start": 45,
  "angle_end": 135,
  "angle_step": 3,     ← Kleinere Schritte
  "duration_per_position": 3.0  ← Länger scannen
}
```

**Schneller Scan (weniger Details, schneller):**
```json
"scan": {
  "angle_start": 60,
  "angle_end": 120,
  "angle_step": 10,    ← Größere Schritte
  "duration_per_position": 1.5  ← Kürzer scannen
}
```

## Punktwolke visualisieren

Die erzeugten `.xyz` Dateien können mit verschiedenen Programmen geöffnet werden:

- **CloudCompare** (kostenlos): https://www.cloudcompare.org/
- **MeshLab** (kostenlos): https://www.meshlab.net/
- **Open3D Viewer** (Python):

```python
import open3d as o3d

# Punktwolke laden
pcd = o3d.io.read_point_cloud("scan_3d_20250101_120000.xyz", format='xyz')

# Visualisieren
o3d.visualization.draw_geometries([pcd])
```

## Fehlerbehandlung

### "Arduino nicht verbunden"
- Prüfe COM-Port in `config.json`
- Prüfe USB-Kabel
- Prüfe ob Arduino-Code hochgeladen ist

### "LIDAR nicht verbunden"
- Prüfe COM-Port in `config.json`
- Prüfe USB-Kabel
- LIDAR sollte sich drehen (Motor läuft)

### "Position out of range"
- Servo-Bereich: 45° - 135°
- Prüfe `angle_start` und `angle_end` in config.json

### "Keine gültigen Messpunkte"
- LIDAR-Sicht ist blockiert
- LIDAR zu nah an Wand/Objekt
- Scan-Dauer erhöhen

## Koordinatensystem

```
        Z (oben)
        ↑
        |
        |
        +----→ X (vorwärts/LIDAR-Richtung)
       /
      /
     ↙ Y (links)
```

**Servo-Orientierung:**
- 45° = LIDAR nach oben geneigt
- 90° = LIDAR horizontal (Standard 2D-Scan)
- 135° = LIDAR nach unten geneigt

## Tipps für gute Scans

1. **Raum vorbereiten:**
   - Gute Beleuchtung (für spätere Fotos/Referenz)
   - Statische Objekte (keine Bewegung während Scan)

2. **LIDAR-Position:**
   - Erhöhte Position (z.B. auf Tisch) für bessere Abdeckung
   - Freie Sicht in alle Richtungen
   - Mindestens 50cm Abstand zu Wänden

3. **Scan-Parameter:**
   - Für Übersicht: 10° Schritte
   - Für Details: 3-5° Schritte
   - Pro Position: 2-3 Sekunden

4. **Nach dem Scan:**
   - Punktwolke in CloudCompare öffnen
   - Outlier entfernen (Cleanup)
   - Verschiedene Ansichten prüfen

## Nächste Schritte (Migration zu ROS2)

Dieses System ist vorbereitet für Migration zu ROS2:

- `arduino_controller.py` → `arduino_bridge_node` (ROS2)
- `lidar_scanner.py` → `rplidar_ros` Package verwenden
- `scan_3d_coordinator.py` → `scan_3d_node` (ROS2)

Die Logik bleibt gleich, nur die Kommunikation läuft über ROS2 Topics.

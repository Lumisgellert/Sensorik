"""
3D LIDAR Scanner - Hauptprogramm
Koordiniert Arduino-Servo und LIDAR für 3D-Raumscans
"""

import numpy as np
import time
import json
from datetime import datetime
from arduino_controller_JSON_NO_STARTUP import ArduinoServoController
from lidar_scanner import LidarScanner


class Scanner3D:
    def __init__(self, config):
        """
        Initialisiert 3D-Scanner
        
        Args:
            config: Dictionary mit Konfiguration
        """
        self.config = config
        self.arduino = None
        self.lidar = None
        self.scan_data_3d = []
        
    def connect_devices(self):
        """Verbindet Arduino und LIDAR"""
        print("=== 3D LIDAR Scanner ===\n")
        print("🔌 Verbinde Geräte...\n")
        
        # Arduino verbinden
        self.arduino = ArduinoServoController(
            port=self.config['arduino']['port'],
            baudrate=self.config['arduino']['baudrate']
        )
        
        if not self.arduino.connect():
            print("❌ Arduino-Verbindung fehlgeschlagen!")
            return False
        
        print()
        
        # LIDAR verbinden
        self.lidar = LidarScanner(port=self.config['lidar']['port'])
        
        if not self.lidar.connect():
            print("❌ LIDAR-Verbindung fehlgeschlagen!")
            self.arduino.disconnect()
            return False
        
        print("\n✅ Alle Geräte verbunden!\n")
        return True
    
    def perform_3d_scan(self, manual_start=True):
        """
        Führt kompletten 3D-Scan durch
        
        Args:
            manual_start: Wenn True, wartet auf Benutzereingabe vor Start
        """
        if manual_start:
            input("📋 Drücke ENTER um 3D-Scan zu starten...")
            print()
        
        # Scan-Parameter
        angle_start = self.config['scan']['angle_start']
        angle_end = self.config['scan']['angle_end']
        angle_step = self.config['scan']['angle_step']
        scan_duration = self.config['scan']['duration_per_position']
        
        # Berechne Anzahl der Positionen
        angles = list(range(angle_start, angle_end + 1, angle_step))
        num_positions = len(angles)
        
        print(f"📐 Scan-Konfiguration:")
        print(f"   Winkelbereich: {angle_start}° bis {angle_end}°")
        print(f"   Schrittweite: {angle_step}°")
        print(f"   Anzahl Positionen: {num_positions}")
        print(f"   Scan-Dauer pro Position: {scan_duration}s")
        print(f"   Geschätzte Gesamtdauer: ~{num_positions * (scan_duration + 2)} Sekunden")
        print()
        
        # Zur Startposition fahren
        print(f"🏠 Fahre zu Startposition ({angle_start}°)...")
        self.arduino.move_to_position(angle_start, wait=True)
        time.sleep(1)
        
        print(f"\n{'='*60}")
        print(f"🚀 3D-Scan startet jetzt!")
        print(f"{'='*60}\n")
        
        self.scan_data_3d = []
        start_time = time.time()
        
        for i, angle in enumerate(angles):
            print(f"📍 Position {i+1}/{num_positions}: {angle}°")
            
            # Servo zur Position bewegen
            if i > 0:  # Erste Position bereits erreicht
                self.arduino.move_to_position(angle, wait=True)
                time.sleep(0.5)  # Kurz warten bis Schwingungen abgeklungen
            
            # 2D-Scan durchführen
            scan_2d = self.lidar.get_single_scan(duration=scan_duration)
            
            if scan_2d is not None:
                # 2D-Scan + Servo-Winkel speichern
                self.scan_data_3d.append({
                    'servo_angle': angle,
                    'scan_2d': scan_2d,
                    'timestamp': time.time()
                })
                
                stats = self.lidar.get_scan_statistics(scan_2d)
                print(f"   ✅ {stats['num_points']} Punkte erfasst")
            else:
                print(f"   ⚠️ Scan fehlgeschlagen!")
            
            print()
        
        total_time = time.time() - start_time
        
        print(f"{'='*60}")
        print(f"✅ 3D-Scan abgeschlossen!")
        print(f"   Dauer: {total_time:.1f} Sekunden")
        print(f"   Positionen: {len(self.scan_data_3d)}/{num_positions}")
        print(f"{'='*60}\n")
        
        # Zurück zu Home-Position
        print("🏠 Fahre zurück zu Home-Position (90°)...")
        self.arduino.go_home(wait=True)
    
    def convert_to_3d_pointcloud(self):
        """
        Konvertiert 2D-Scans in 3D-Punktwolke
        
        Returns:
            numpy array mit [x, y, z] Koordinaten
        """
        print("🔄 Konvertiere zu 3D-Punktwolke...")
        
        points_3d = []
        
        for scan_data in self.scan_data_3d:
            servo_angle = scan_data['servo_angle']
            scan_2d = scan_data['scan_2d']
            
            # Servo-Winkel von Grad zu Radiant (90° = horizontal)
            # Wenn Servo bei 90° ist: LIDAR horizontal
            # Wenn Servo bei 45° ist: LIDAR nach oben geneigt
            # Wenn Servo bei 135° ist: LIDAR nach unten geneigt
            servo_rad = np.radians(servo_angle - 90)  # 0° = horizontal
            
            for point in scan_2d:
                lidar_angle_deg, distance_mm, quality = point
                
                # LIDAR-Winkel zu Radiant
                lidar_rad = np.radians(lidar_angle_deg)
                
                # Distanz von mm zu m
                distance_m = distance_mm / 1000.0
                
                # 3D-Koordinaten berechnen
                # X-Achse: nach vorne
                # Y-Achse: nach links
                # Z-Achse: nach oben
                
                x = distance_m * np.cos(servo_rad) * np.cos(lidar_rad)
                y = distance_m * np.sin(lidar_rad)
                z = distance_m * np.sin(servo_rad) * np.cos(lidar_rad)
                
                points_3d.append([x, y, z])
        
        points_3d = np.array(points_3d)
        
        print(f"✅ {len(points_3d)} 3D-Punkte erstellt\n")
        
        return points_3d
    
    def save_pointcloud(self, filename=None):
        """
        Speichert 3D-Punktwolke als .txt und .xyz Datei
        
        Args:
            filename: Dateiname (ohne Endung)
        """
        if len(self.scan_data_3d) == 0:
            print("⚠️ Keine Scan-Daten vorhanden!")
            return
        
        # 3D-Punktwolke erstellen
        points_3d = self.convert_to_3d_pointcloud()
        
        # Dateiname generieren
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"scan_3d_{timestamp}"
        
        # Als TXT speichern (einfaches Format)
        txt_file = f"{filename}.txt"
        np.savetxt(txt_file, points_3d, fmt='%.6f', delimiter=',', 
                   header='X,Y,Z', comments='')
        print(f"💾 Gespeichert: {txt_file}")
        
        # Als XYZ speichern (Standard-Format für Punktwolken)
        xyz_file = f"{filename}.xyz"
        np.savetxt(xyz_file, points_3d, fmt='%.6f')
        print(f"💾 Gespeichert: {xyz_file}")
        
        # Statistiken speichern
        stats_file = f"{filename}_stats.json"
        stats = {
            'timestamp': datetime.now().isoformat(),
            'num_positions': len(self.scan_data_3d),
            'num_points_3d': len(points_3d),
            'config': self.config['scan']
        }
        
        with open(stats_file, 'w') as f:
            json.dump(stats, f, indent=2)
        print(f"💾 Gespeichert: {stats_file}")
        
        print(f"\n📊 Punktwolken-Statistiken:")
        print(f"   Punkte: {len(points_3d)}")
        print(f"   X-Bereich: [{np.min(points_3d[:, 0]):.2f}, {np.max(points_3d[:, 0]):.2f}] m")
        print(f"   Y-Bereich: [{np.min(points_3d[:, 1]):.2f}, {np.max(points_3d[:, 1]):.2f}] m")
        print(f"   Z-Bereich: [{np.min(points_3d[:, 2]):.2f}, {np.max(points_3d[:, 2]):.2f}] m")
    
    def disconnect_devices(self):
        """Trennt alle Verbindungen"""
        print("\n🔌 Trenne Verbindungen...")
        
        if self.arduino:
            self.arduino.disconnect()
        
        if self.lidar:
            self.lidar.disconnect()
        
        print("✅ Fertig!")


def load_config(config_file='config.json'):
    """Lädt Konfiguration aus JSON-Datei"""
    try:
        with open(config_file, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"⚠️ Konfigurationsdatei '{config_file}' nicht gefunden.")
        print("   Verwende Standard-Konfiguration.\n")
        return get_default_config()


def get_default_config():
    """Standard-Konfiguration"""
    return {
        'arduino': {
            'port': 'COM3',
            'baudrate': 115200
        },
        'lidar': {
            'port': 'COM4'
        },
        'scan': {
            'angle_start': 45,
            'angle_end': 135,
            'angle_step': 5,
            'duration_per_position': 2.0
        }
    }


# Hauptprogramm
if __name__ == "__main__":
    # Konfiguration laden
    config = load_config('config.json')
    
    # Scanner erstellen
    scanner = Scanner3D(config)
    
    try:
        # Geräte verbinden
        if scanner.connect_devices():
            
            # 3D-Scan durchführen
            scanner.perform_3d_scan(manual_start=True)
            
            # Punktwolke speichern
            scanner.save_pointcloud()
            
    except KeyboardInterrupt:
        print("\n\n⚠️ Abbruch durch Benutzer (Ctrl+C)")
        
    finally:
        # Aufräumen
        scanner.disconnect_devices()

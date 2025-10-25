"""
RPLIDAR A1M8 Scanner
Liest 2D-Scans vom LIDAR
"""

from rplidar import RPLidar
import numpy as np
import time


class LidarScanner:
    def __init__(self, port='COM4'):
        """
        Initialisiert RPLIDAR
        
        Args:
            port: COM-Port des LIDAR (Windows: 'COM4', 'COM5', etc.)
        """
        self.port = port
        self.lidar = None
        self.is_connected = False
        
    def connect(self):
        """Verbindung zum LIDAR herstellen"""
        try:
            self.lidar = RPLidar(self.port)
            
            # LIDAR Info abrufen
            info = self.lidar.get_info()
            health = self.lidar.get_health()
            
            print(f"✅ LIDAR verbunden auf {self.port}")
            print(f"   Modell: {info['model']}")
            print(f"   Firmware: {info['firmware'][0]}.{info['firmware'][1]}")
            print(f"   Hardware: {info['hardware']}")
            print(f"   Gesundheit: {health[0]}")
            
            self.is_connected = True
            return True
            
        except Exception as e:
            print(f"❌ LIDAR Verbindungsfehler: {e}")
            return False
    
    def get_single_scan(self, duration=2.0, min_quality=10):
        """
        Führt einen einzelnen 2D-Scan durch
        
        Args:
            duration: Scan-Dauer in Sekunden
            min_quality: Mindest-Qualität der Messungen (0-15)
        
        Returns:
            numpy array mit [angle, distance, quality] für jeden Messpunkt
            oder None bei Fehler
        """
        if not self.is_connected:
            print("❌ LIDAR nicht verbunden!")
            return None
        
        print(f"📡 Scanne für {duration} Sekunden...")
        
        scan_data = []
        start_time = time.time()
        
        try:
            # LIDAR starten
            iterator = self.lidar.iter_scans(max_buf_meas=500)
            
            while time.time() - start_time < duration:
                scan = next(iterator)
                
                for measurement in scan:
                    quality, angle, distance = measurement
                    
                    # Nur gültige Messungen (Qualität und Distanz > 0)
                    if quality >= min_quality and distance > 0:
                        scan_data.append([angle, distance, quality])
            
            # LIDAR stoppen
            self.lidar.stop()
            self.lidar.stop_motor()
            
            if len(scan_data) > 0:
                scan_array = np.array(scan_data)
                print(f"✅ Scan abgeschlossen: {len(scan_data)} Messpunkte")
                return scan_array
            else:
                print("⚠️ Keine gültigen Messpunkte!")
                return None
                
        except Exception as e:
            print(f"❌ Scan-Fehler: {e}")
            self.stop()
            return None
    
    def stop(self):
        """LIDAR stoppen"""
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
            except:
                pass
    
    def disconnect(self):
        """Verbindung trennen"""
        if self.lidar:
            try:
                self.stop()
                self.lidar.disconnect()
                print("🔌 LIDAR getrennt")
            except:
                pass
            self.is_connected = False
    
    def get_scan_statistics(self, scan_data):
        """
        Berechnet Statistiken eines Scans
        
        Args:
            scan_data: numpy array vom Scan
        
        Returns:
            Dictionary mit Statistiken
        """
        if scan_data is None or len(scan_data) == 0:
            return None
        
        distances = scan_data[:, 1]
        
        stats = {
            'num_points': len(scan_data),
            'min_distance': np.min(distances),
            'max_distance': np.max(distances),
            'mean_distance': np.mean(distances),
            'median_distance': np.median(distances)
        }
        
        return stats


# Test-Code
if __name__ == "__main__":
    print("=== RPLIDAR Scanner Test ===\n")
    
    # Port anpassen! (Windows: COM4, COM5, etc.)
    scanner = LidarScanner(port='COM4')
    
    if scanner.connect():
        print("\n--- Test: Einzelner Scan ---")
        
        scan_data = scanner.get_single_scan(duration=3.0)
        
        if scan_data is not None:
            stats = scanner.get_scan_statistics(scan_data)
            print(f"\n📊 Scan-Statistiken:")
            print(f"   Messpunkte: {stats['num_points']}")
            print(f"   Min Distanz: {stats['min_distance']:.2f} mm")
            print(f"   Max Distanz: {stats['max_distance']:.2f} mm")
            print(f"   Durchschnitt: {stats['mean_distance']:.2f} mm")
            
            # Erste 5 Messpunkte anzeigen
            print(f"\n📍 Erste 5 Messpunkte:")
            print("   Winkel | Distanz | Qualität")
            for i in range(min(5, len(scan_data))):
                angle, dist, quality = scan_data[i]
                print(f"   {angle:6.2f}° | {dist:7.1f} mm | {quality:2.0f}")
        
        scanner.disconnect()
    else:
        print("❌ Verbindung fehlgeschlagen!")

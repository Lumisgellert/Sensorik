"""
Arduino Servo Controller
Kommuniziert mit Arduino über Serial (JSON)
"""

import serial
import json
import time
import threading


class ArduinoServoController:
    def __init__(self, port='COM3', baudrate=115200, timeout=2):
        """
        Initialisiert Verbindung zum Arduino
        
        Args:
            port: COM-Port (Windows: 'COM3', 'COM4', etc.)
            baudrate: 115200
            timeout: Timeout in Sekunden
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.position_reached = False
        self.current_position = 90
        self.is_connected = False
        self.read_thread = None
        self.running = False
        
    def connect(self):
        """Verbindung zum Arduino herstellen"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=2)
            time.sleep(2)  # Arduino Reset nach Verbindung abwarten
            
            # Startup-Nachricht lesen
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8').strip()
                data = json.loads(line)
                if data.get('status') == 'ready':
                    print(f"✅ Arduino verbunden auf {self.port}")
                    print(f"   Position: {data.get('position')}°")
                    print(f"   Bereich: {data.get('range_min')}° - {data.get('range_max')}°")
                    self.current_position = data.get('position', 90)
                    self.is_connected = True
                    
                    # Lese-Thread starten
                    self.running = True
                    self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
                    self.read_thread.start()
                    
                    return True
        except Exception as e:
            print(f"❌ Fehler beim Verbinden: {e}")
            return False
    
    def _read_loop(self):
        """Liest kontinuierlich Daten vom Arduino (in separatem Thread)"""
        while self.running:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line:
                        self._process_message(line)
            except Exception as e:
                print(f"⚠️ Read error: {e}")
            time.sleep(0.01)
    
    def _process_message(self, line):
        """Verarbeitet eingehende Nachrichten vom Arduino"""
        try:
            data = json.loads(line)
            status = data.get('status')
            
            if status == 'reached':
                self.position_reached = True
                self.current_position = data.get('position')
                print(f"✅ Position erreicht: {self.current_position}°")
                
            elif status == 'moving':
                print(f"🔄 Servo bewegt sich zu {data.get('target')}°...")
                self.position_reached = False
                
            elif status == 'error':
                print(f"❌ Arduino Fehler: {data.get('message')}")
                
            elif status == 'homing':
                print(f"🏠 Fahre zu Home-Position...")
                self.position_reached = False
                
        except json.JSONDecodeError:
            print(f"⚠️ Ungültiges JSON: {line}")
    
    def move_to_position(self, angle, wait=True):
        """
        Bewegt Servo zu bestimmter Position
        
        Args:
            angle: Zielwinkel (45-135°)
            wait: Warten bis Position erreicht (True/False)
        
        Returns:
            True wenn erfolgreich
        """
        if not self.is_connected:
            print("❌ Nicht verbunden!")
            return False
        
        if angle < 45 or angle > 135:
            print(f"❌ Winkel {angle}° außerhalb des Bereichs (45-135°)")
            return False
        
        command = {"position": angle}
        self.position_reached = False
        
        try:
            self.serial.write((json.dumps(command) + '\n').encode('utf-8'))
            
            if wait:
                # Warten bis Position erreicht
                timeout = 5  # 5 Sekunden Timeout
                start_time = time.time()
                
                while not self.position_reached:
                    if time.time() - start_time > timeout:
                        print("⚠️ Timeout beim Warten auf Position")
                        return False
                    time.sleep(0.1)
            
            return True
            
        except Exception as e:
            print(f"❌ Fehler beim Senden: {e}")
            return False
    
    def go_home(self, wait=True):
        """Fährt zu Home-Position (90°)"""
        command = {"home": True}
        self.position_reached = False
        
        try:
            self.serial.write((json.dumps(command) + '\n').encode('utf-8'))
            
            if wait:
                timeout = 5
                start_time = time.time()
                
                while not self.position_reached:
                    if time.time() - start_time > timeout:
                        print("⚠️ Timeout beim Homing")
                        return False
                    time.sleep(0.1)
            
            return True
            
        except Exception as e:
            print(f"❌ Fehler beim Homing: {e}")
            return False
    
    def get_current_position(self):
        """Gibt aktuelle Position zurück"""
        return self.current_position
    
    def disconnect(self):
        """Verbindung trennen"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1)
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("🔌 Arduino getrennt")


# Test-Code
if __name__ == "__main__":
    # Test: Arduino-Verbindung und Servo-Bewegung
    print("=== Arduino Servo Controller Test ===\n")
    
    # Port anpassen! (Windows: COM3, COM4, etc.)
    controller = ArduinoServoController(port='COM3')
    
    if controller.connect():
        print("\n--- Test 1: Home Position ---")
        controller.go_home()
        time.sleep(1)
        
        print("\n--- Test 2: Bewegungen ---")
        test_positions = [90, 60, 90, 120, 90]
        
        for pos in test_positions:
            print(f"\nFahre zu {pos}°...")
            controller.move_to_position(pos, wait=True)
            time.sleep(0.5)
        
        print("\n--- Test abgeschlossen ---")
        controller.disconnect()
    else:
        print("❌ Verbindung fehlgeschlagen!")

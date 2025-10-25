"""
Arduino Servo Controller - OHNE Startup-Warten
Sendet direkt Befehle, wartet nicht auf Startup-Nachricht
"""

import serial
import time
import threading


class ArduinoServoController:
    def __init__(self, port='COM14', baudrate=115200):
        """
        Initialisiert Verbindung zum Arduino
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
            print(f"🔌 Verbinde mit {self.port}...")
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            
            # Kurz warten damit Arduino bereit ist
            time.sleep(0.5)
            
            # Buffer leeren (falls alte Daten)
            self.serial.reset_input_buffer()
            
            print(f"✅ Verbindung hergestellt!")
            self.is_connected = True
            
            # Lese-Thread starten
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            # Kurze Pause
            time.sleep(0.2)
            
            return True
            
        except Exception as e:
            print(f"❌ Fehler beim Verbinden: {e}")
            return False
    
    def _read_loop(self):
        """Liest kontinuierlich Daten vom Arduino"""
        while self.running:
            try:
                if self.serial and self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self._process_message(line)
            except Exception as e:
                print(f"⚠️ Read error: {e}")
            time.sleep(0.01)
    
    def _process_message(self, line):
        """Verarbeitet eingehende Nachrichten vom Arduino"""
        
        # REACHED:90
        if line.startswith("REACHED:"):
            try:
                pos = int(line.split(":")[1])
                self.position_reached = True
                self.current_position = pos
                print(f"✅ Position erreicht: {pos}°")
            except:
                pass
        
        # MOVING:90
        elif line.startswith("MOVING:"):
            try:
                target = int(line.split(":")[1])
                print(f"🔄 Servo bewegt sich zu {target}°...")
                self.position_reached = False
            except:
                pass
        
        # HOMING
        elif line == "HOMING":
            print(f"🏠 Fahre zu Home-Position...")
            self.position_reached = False
        
        # READY (ignorieren, nur Info)
        elif line.startswith("READY:"):
            print(f"📌 Arduino meldet: {line}")
        
        # ERROR:...
        elif line.startswith("ERROR:"):
            error_msg = line.split(":", 1)[1] if ":" in line else line
            print(f"❌ Arduino Fehler: {error_msg}")
        
        # Andere Nachrichten
        else:
            # Nur anzeigen wenn interessant
            if len(line) > 0 and not line.startswith("Test"):
                print(f"📨 Arduino: {line}")
    
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
        
        # Befehl senden: "P90\n"
        command = f"P{angle}\n"
        self.position_reached = False
        
        try:
            self.serial.write(command.encode('utf-8'))
            self.serial.flush()  # Sicherstellen dass gesendet
            
            if wait:
                # Warten bis Position erreicht
                timeout = 5
                start_time = time.time()
                
                while not self.position_reached:
                    if time.time() - start_time > timeout:
                        print("⚠️ Timeout beim Warten auf Position")
                        return False
                    time.sleep(0.05)
            
            return True
            
        except Exception as e:
            print(f"❌ Fehler beim Senden: {e}")
            return False
    
    def go_home(self, wait=True):
        """Fährt zu Home-Position (90°)"""
        command = "H\n"
        self.position_reached = False
        
        try:
            self.serial.write(command.encode('utf-8'))
            self.serial.flush()
            
            if wait:
                timeout = 5
                start_time = time.time()
                
                while not self.position_reached:
                    if time.time() - start_time > timeout:
                        print("⚠️ Timeout beim Homing")
                        return False
                    time.sleep(0.05)
            
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
    print("="*60)
    print("Arduino Servo Controller Test (NO STARTUP)")
    print("="*60)
    print()
    
    controller = ArduinoServoController(port='COM14')
    
    if controller.connect():
        print()
        print("="*60)
        print("Verbindung OK! Starte Test...")
        print("="*60)
        print()
        
        time.sleep(1)
        
        print("--- Test 1: Home Position ---")
        controller.go_home()
        time.sleep(1)
        
        print("\n--- Test 2: Bewegungen ---")
        test_positions = [60, 90, 120, 90, 75, 90]
        
        for pos in test_positions:
            print(f"\n→ Fahre zu {pos}°...")
            controller.move_to_position(pos, wait=True)
            time.sleep(0.3)
        
        print("\n" + "="*60)
        print("✅ Test abgeschlossen!")
        print("="*60)
        
        controller.disconnect()
    else:
        print("\n❌ Verbindung fehlgeschlagen!")
        print("\nPrüfe:")
        print("• Arduino IDE geschlossen?")
        print("• COM14 korrekt?")
        print("• USB-Kabel OK?")

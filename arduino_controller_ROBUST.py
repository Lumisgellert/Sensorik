"""
Arduino Servo Controller - ROBUST Version
Mit Debug-Ausgaben und besserem Fehlerhandling
"""

import serial
import time
import threading


class ArduinoServoController:
    def __init__(self, port='COM14', baudrate=115200, debug=True):
        """
        Initialisiert Verbindung zum Arduino
        
        Args:
            port: COM-Port
            baudrate: Baudrate (115200)
            debug: Debug-Ausgaben aktivieren
        """
        self.port = port
        self.baudrate = baudrate
        self.debug = debug
        self.serial = None
        self.position_reached = False
        self.current_position = 90
        self.is_connected = False
        self.read_thread = None
        self.running = False
        
    def connect(self):
        """Verbindung zum Arduino herstellen"""
        try:
            if self.debug:
                print(f"🔌 Verbinde mit {self.port} @ {self.baudrate} baud...")
            
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            
            # Warten und Buffer leeren
            time.sleep(0.5)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            if self.debug:
                print(f"✅ Port geöffnet!")
            
            self.is_connected = True
            
            # Lese-Thread starten
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            time.sleep(0.3)
            
            if self.debug:
                print(f"✅ Arduino bereit!")
            
            return True
            
        except Exception as e:
            print(f"❌ Verbindungsfehler: {e}")
            return False
    
    def _read_loop(self):
        """Liest kontinuierlich vom Arduino"""
        while self.running:
            try:
                if self.serial and self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self._process_message(line)
            except Exception as e:
                if self.debug:
                    print(f"⚠️ Read error: {e}")
            time.sleep(0.01)
    
    def _process_message(self, line):
        """Verarbeitet Arduino-Nachrichten"""
        
        if self.debug:
            print(f"📨 Arduino: {line}")
        
        # REACHED:90
        if line.startswith("REACHED:"):
            try:
                pos = int(line.split(":")[1])
                self.position_reached = True
                self.current_position = pos
                if not self.debug:
                    print(f"✅ Position erreicht: {pos}°")
            except:
                pass
        
        # MOVING:90
        elif line.startswith("MOVING:"):
            try:
                target = int(line.split(":")[1])
                self.position_reached = False
                if not self.debug:
                    print(f"🔄 Bewege zu {target}°...")
            except:
                pass
        
        # ERROR
        elif line.startswith("ERROR:"):
            print(f"❌ {line}")
        
        # READY (ignorieren)
        elif line.startswith("READY:"):
            pass
    
    def move_to_position(self, angle, wait=True, timeout=3):
        """
        Bewegt Servo zu Position
        
        Args:
            angle: Zielwinkel (45-135°)
            wait: Warten bis erreicht
            timeout: Timeout in Sekunden
        
        Returns:
            True wenn erfolgreich
        """
        if not self.is_connected:
            print("❌ Nicht verbunden!")
            return False
        
        if angle < 45 or angle > 135:
            print(f"❌ Winkel {angle}° außerhalb 45-135°")
            return False
        
        # Befehl senden
        command = f"P{angle}\n"
        self.position_reached = False
        
        try:
            if self.debug:
                print(f"📤 Sende: {command.strip()}")
            
            self.serial.write(command.encode('utf-8'))
            self.serial.flush()
            
            if wait:
                start_time = time.time()
                
                while not self.position_reached:
                    if time.time() - start_time > timeout:
                        print(f"⚠️ Timeout nach {timeout}s")
                        return False
                    time.sleep(0.05)
                
                # Kurze Pause nach Erreichen
                time.sleep(0.1)
            
            return True
            
        except Exception as e:
            print(f"❌ Sendefehler: {e}")
            return False
    
    def go_home(self, wait=True):
        """Fährt zu Home (90°)"""
        command = "H\n"
        self.position_reached = False
        
        try:
            if self.debug:
                print(f"📤 Sende: {command.strip()}")
            
            self.serial.write(command.encode('utf-8'))
            self.serial.flush()
            
            if wait:
                start_time = time.time()
                
                while not self.position_reached:
                    if time.time() - start_time > 3:
                        print("⚠️ Timeout")
                        return False
                    time.sleep(0.05)
                
                time.sleep(0.1)
            
            return True
            
        except Exception as e:
            print(f"❌ Fehler: {e}")
            return False
    
    def query_status(self):
        """Fragt aktuellen Status ab"""
        if self.is_connected:
            self.serial.write(b"?\n")
            time.sleep(0.1)
    
    def get_current_position(self):
        """Gibt aktuelle Position zurück"""
        return self.current_position
    
    def disconnect(self):
        """Trennt Verbindung"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1)
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("🔌 Getrennt")


# Test
if __name__ == "__main__":
    print("="*70)
    print("Arduino Servo Controller - ROBUST Test")
    print("="*70)
    print()
    
    # Debug=True zeigt alle Nachrichten
    controller = ArduinoServoController(port='COM14', debug=True)
    
    if controller.connect():
        print()
        print("="*70)
        print("Test startet...")
        print("="*70)
        print()
        
        time.sleep(1)
        
        # Test 1: Home
        print("\n--- Test 1: Home Position ---")
        controller.go_home()
        time.sleep(0.5)
        
        # Test 2: Bewegungen
        print("\n--- Test 2: Verschiedene Positionen ---")
        test_positions = [60, 75, 90, 105, 120, 90]
        
        for i, pos in enumerate(test_positions):
            print(f"\n[{i+1}/{len(test_positions)}] Fahre zu {pos}°")
            success = controller.move_to_position(pos, wait=True, timeout=3)
            
            if not success:
                print(f"⚠️ Position {pos}° nicht erreicht!")
            
            time.sleep(0.3)
        
        # Test 3: Status abfragen
        print("\n--- Test 3: Status abfragen ---")
        controller.query_status()
        time.sleep(0.5)
        
        print("\n" + "="*70)
        print("✅ Test abgeschlossen!")
        print("="*70)
        
        controller.disconnect()
    else:
        print("\n❌ Verbindung fehlgeschlagen!")
        print("\nChecke:")
        print("• Arduino IDE geschlossen?")
        print("• Richtiger Port (COM14)?")
        print("• Arduino-Code hochgeladen?")

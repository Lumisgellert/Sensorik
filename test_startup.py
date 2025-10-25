"""
Testet ob Arduino beim Öffnen der Verbindung sendet
"""

import serial
import time

PORT = 'COM14'
BAUDRATE = 115200

print("="*60)
print("TEST: Arduino-Startup beim Verbinden")
print("="*60)
print()
print("WICHTIG: Arduino IDE muss GESCHLOSSEN sein!")
print()
input("Drücke ENTER um zu starten...")
print()

try:
    print(f"Öffne {PORT} mit {BAUDRATE} Baud...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print("✅ Verbindung hergestellt!")
    print()
    
    print("=" * 60)
    print("JETZT passiert folgendes:")
    print("1. Arduino macht Reset (weil Serial verbunden wurde)")
    print("2. Arduino sollte seine Startup-Nachricht senden")
    print("3. Wir lesen 10 Sekunden lang alles")
    print("=" * 60)
    print()
    
    print("Warte 1 Sekunde auf Arduino-Reset...")
    time.sleep(1)
    print()
    
    print("Lese jetzt 10 Sekunden:")
    print("-" * 60)
    
    received_anything = False
    start_time = time.time()
    
    while time.time() - start_time < 10:
        # Prüfe ob Daten da sind
        waiting = ser.in_waiting
        if waiting > 0:
            print(f"[{waiting} bytes warten]", end=" ")
            
            # Lese Zeile
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"→ '{line}'")
                    received_anything = True
                else:
                    print("(leer)")
            except Exception as e:
                print(f"Fehler beim Lesen: {e}")
        
        time.sleep(0.2)
    
    print("-" * 60)
    print()
    
    if received_anything:
        print("✅✅✅ SUCCESS! Arduino sendet an Python! ✅✅✅")
        print()
        print("Das Problem liegt woanders.")
        print("Wahrscheinlich: Timing oder Code-Logik.")
        
    else:
        print("❌❌❌ PROBLEM GEFUNDEN! ❌❌❌")
        print()
        print("Arduino sendet NICHTS wenn Python verbindet!")
        print()
        print("Mögliche Ursachen:")
        print("1. Arduino macht keinen Reset beim Verbinden")
        print("2. Arduino-Code wartet nicht auf Serial")
        print("3. Arduino-Code hat Fehler")
        print()
        print("LÖSUNG:")
        print("→ Drücke manuell RESET am Arduino NACHDEM Python verbunden ist")
        print()
        print("Teste das:")
        print("1. Starte dieses Script nochmal")
        print("2. Wenn 'Lese jetzt 10 Sekunden' erscheint:")
        print("3. DRÜCKE RESET-BUTTON AM ARDUINO")
        print("4. Arduino sollte dann senden")
    
    ser.close()
    print()
    print("Verbindung geschlossen.")
    
except serial.SerialException as e:
    print(f"❌ Fehler beim Öffnen: {e}")
    print()
    print("→ Ist Arduino IDE Serial Monitor noch offen?")
    print("→ Schließe Arduino IDE komplett!")
    
except KeyboardInterrupt:
    print("\n\nAbgebrochen durch Benutzer.")
    if 'ser' in locals() and ser.is_open:
        ser.close()

except Exception as e:
    print(f"❌ Fehler: {e}")

print()
print("="*60)

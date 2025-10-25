"""
Super einfacher Test - nur lesen vom Arduino
"""

import serial
import time

PORT = 'COM14'
BAUDRATE = 115200

print("="*50)
print("MINIMALER ARDUINO TEST")
print("="*50)
print()

try:
    print(f"1. Öffne {PORT}...")
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print("   ✅ Port geöffnet")
    print()
    
    print("2. Warte 3 Sekunden...")
    time.sleep(3)
    print()
    
    print("3. Lese 10 Sekunden lang alles was kommt:")
    print("-" * 50)
    
    start = time.time()
    line_count = 0
    
    while time.time() - start < 10:
        if ser.in_waiting > 0:
            data = ser.readline()
            try:
                line = data.decode('utf-8').strip()
                if line:
                    line_count += 1
                    print(f"[{line_count}] {line}")
            except:
                print(f"[{line_count}] (unlesbar)")
        time.sleep(0.1)
    
    print("-" * 50)
    print()
    
    if line_count > 0:
        print(f"✅ SUCCESS! {line_count} Zeilen empfangen!")
        print()
        print("Arduino funktioniert!")
        
    else:
        print("❌ KEINE Daten empfangen!")
        print()
        print("Mach das:")
        print("1. Drücke RESET am Arduino")
        print("2. Starte dieses Script nochmal")
    
    ser.close()
    print()
    print("Port geschlossen.")
    
except serial.SerialException as e:
    print(f"❌ FEHLER: {e}")
    print()
    print("Ist der Serial Monitor noch offen?")
    print("→ Arduino IDE komplett schließen!")
    
except Exception as e:
    print(f"❌ Unerwarteter Fehler: {e}")

print()
print("="*50)

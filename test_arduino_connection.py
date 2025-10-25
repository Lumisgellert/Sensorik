"""
Arduino Debug - Erweiterte Diagnose
"""

import serial
import serial.tools.list_ports
import time

def check_ports():
    """Zeigt alle verfügbaren Ports"""
    print("📌 Verfügbare COM-Ports:")
    ports = serial.tools.list_ports.comports()
    
    if len(ports) == 0:
        print("   ❌ Keine Ports gefunden!")
        return []
    
    for port in ports:
        print(f"   • {port.device}")
        print(f"     Name: {port.description}")
        print(f"     Hardware: {port.hwid}")
        print()
    
    return [p.device for p in ports]

def test_connection(port='COM14'):
    """Testet Verbindung zu Arduino"""
    print(f"\n{'='*50}")
    print(f"Teste Verbindung zu {port}...")
    print(f"{'='*50}\n")
    
    # Prüfen ob Port existiert
    available = check_ports()
    if port not in available:
        print(f"❌ Port {port} nicht in der Liste!")
        print(f"   Verfügbare Ports: {available}")
        return False
    
    try:
        print(f"🔌 Öffne {port}...")
        ser = serial.Serial(port, 115200, timeout=2)
        print(f"✅ Port erfolgreich geöffnet!\n")
        
        print("⏳ Warte 2 Sekunden auf Arduino-Reset...")
        time.sleep(2)
        
        # Prüfen ob Daten kommen
        print("\n📡 Lese Daten (5 Sekunden)...\n")
        start_time = time.time()
        
        received_data = False
        while time.time() - start_time < 5:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"✅ Empfangen: {line}")
                    received_data = True
            time.sleep(0.1)
        
        if not received_data:
            print("\n⚠️ Keine Daten empfangen!")
            print("\n🔧 Mögliche Ursachen:")
            print("   1. Arduino-Code nicht hochgeladen")
            print("   2. Falscher COM-Port")
            print("   3. Arduino hängt/gecrasht")
            print("\n💡 Lösung:")
            print("   • Reset-Button am Arduino drücken")
            print("   • Arduino-Code neu hochladen")
            print("   • Anderen Port testen")
        
        ser.close()
        print("\n🔌 Port geschlossen")
        return received_data
        
    except serial.SerialException as e:
        print(f"\n❌ SerialException: {e}\n")
        
        if "PermissionError" in str(e) or "Access is denied" in str(e):
            print("🔧 Port ist bereits belegt!")
            print("   Schließe:")
            print("   • Arduino IDE Serial Monitor")
            print("   • Andere Python-Programme")
            print("   • Andere Serial-Programme (PuTTY, etc.)")
            
        elif "FileNotFoundError" in str(e) or "could not open port" in str(e):
            print("🔧 Port existiert nicht!")
            print(f"   {port} ist nicht verfügbar")
            print(f"   Verfügbare Ports: {available}")
            
        return False
        
    except Exception as e:
        print(f"\n❌ Unbekannter Fehler: {e}")
        print(f"   Typ: {type(e).__name__}")
        return False

if __name__ == "__main__":
    print("╔════════════════════════════════════════╗")
    print("║   Arduino Verbindungs-Diagnose         ║")
    print("╚════════════════════════════════════════╝\n")
    
    # WICHTIG: Passe hier deinen COM-Port an!
    PORT = 'COM14'  # ← Ändere falls nötig
    
    success = test_connection(PORT)
    
    if success:
        print("\n" + "="*50)
        print("✅ VERBINDUNG ERFOLGREICH!")
        print("="*50)
        print("\nDu kannst jetzt arduino_controller.py nutzen:")
        print("  python arduino_controller.py")
    else:
        print("\n" + "="*50)
        print("❌ VERBINDUNG FEHLGESCHLAGEN")
        print("="*50)
        print("\nBehebe die oben genannten Probleme und versuche es erneut.")

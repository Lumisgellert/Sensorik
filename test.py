import serial
import time

# Arduino-Verbindung herstellen
# Ändere 'COM3' zu deinem Port (z.B. COM4, COM5 unter Windows)
# Unter Mac/Linux: '/dev/ttyUSB0' oder '/dev/ttyACM0'
arduino = serial.Serial('COM3', 9600)
time.sleep(2)  # Warte, bis die Verbindung steht

while True:
    # Winkel vom Benutzer abfragen
    winkel = input("Gib einen Winkel ein (0-180): ")

    # Winkel an Arduino senden
    arduino.write(winkel.encode())
    print(f"Winkel {winkel} wurde gesendet!")

# Verbindung schließen
arduino.close()
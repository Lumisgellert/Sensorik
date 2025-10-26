/*
 * Servo Controller - Basierend auf funktionierendem Labor-Code
 * 
 * Befehle:
 * P45  = Fahre zu 45°
 * P90  = Fahre zu 90°
 * P135 = Fahre zu 135°
 * H    = Home (90°)
 * 
 * Einfach und bewährt!
 */

#include <Servo.h>

Servo meinServo;
int aktuellePosition = 90;

void setup() {
  Serial.begin(115200);  // Gleiche Baudrate wie Python
  meinServo.attach(9);
  meinServo.write(90);   // Startposition
  
  delay(500);
  
  Serial.println("READY:90:45:135");
}

void loop() {
  if (Serial.available() > 0) {
    String befehl = Serial.readStringUntil('\n');
    befehl.trim();
    
    // Position setzen: "P90"
    if (befehl.startsWith("P") || befehl.startsWith("p")) {
      String winkelStr = befehl.substring(1);
      int winkel = winkelStr.toInt();
      
      if (winkel >= 45 && winkel <= 135) {
        Serial.print("MOVING:");
        Serial.println(winkel);
        
        meinServo.write(winkel);
        aktuellePosition = winkel;
        
        delay(1000);  // 1 Sekunde warten bis Servo Position erreicht
        
        Serial.print("REACHED:");
        Serial.println(winkel);
      } else {
        Serial.print("ERROR:Range 45-135, got ");
        Serial.println(winkel);
      }
    }
    
    // Home Position: "H"
    else if (befehl == "H" || befehl == "h") {
      Serial.println("MOVING:90");
      
      meinServo.write(90);
      aktuellePosition = 90;
      
      delay(1000);
      
      Serial.println("REACHED:90");
    }
    
    // Status abfragen: "?"
    else if (befehl == "?") {
      Serial.print("STATUS:");
      Serial.print(aktuellePosition);
      Serial.println(":IDLE");
    }
  }
}

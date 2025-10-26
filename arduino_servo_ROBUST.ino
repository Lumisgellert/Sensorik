/*
 * 3D LIDAR Scanner - Arduino Servo Controller (ROBUST)
 * 
 * Einfacher und robuster Code ohne JSON
 * 
 * Befehle:
 * P45  = Fahre zu 45°
 * P90  = Fahre zu 90°
 * P135 = Fahre zu 135°
 * H    = Home (90°)
 * 
 * Antworten:
 * MOVING:90
 * REACHED:90
 * ERROR:...
 * 
 * Verkabelung:
 * - Servo Signal → Pin 9
 * - Servo VCC → 5V
 * - Servo GND → GND
 */

#include <Servo.h>

#define SERVO_PIN 9
#define LED_PIN 13  // Eingebaute LED für visuelles Feedback

Servo lidarServo;

int current_position = 90;
bool is_moving = false;
unsigned long move_start_time = 0;
const int MOVE_TIME = 800;  // 800ms für Servo-Bewegung + Settling

void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  
  // Servo initialisieren
  lidarServo.attach(SERVO_PIN);
  lidarServo.write(current_position);
  
  delay(500);
  
  // LED blinken = Bereit
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  // Startup-Nachricht (optional, wird ignoriert wenn Python nicht wartet)
  Serial.println("READY:90:45:135");
}

void loop() {
  // Befehle empfangen
  if(Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if(command.length() > 0) {
      processCommand(command);
    }
  }
  
  // Prüfen ob Servo Position erreicht hat
  if(is_moving) {
    unsigned long elapsed = millis() - move_start_time;
    
    if(elapsed >= MOVE_TIME) {
      is_moving = false;
      
      // Position erreicht Nachricht
      Serial.print("REACHED:");
      Serial.println(current_position);
      
      // LED kurz blinken
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
    }
  }
}

void processCommand(String cmd) {
  
  // Debug: Zeige empfangenen Befehl
  // Serial.print("DEBUG:Received:");
  // Serial.println(cmd);
  
  // Position setzen: "P90"
  if(cmd.startsWith("P") || cmd.startsWith("p")) {
    String pos_str = cmd.substring(1);
    int target_pos = pos_str.toInt();
    
    // Sicherheit: Bereich prüfen
    if(target_pos < 45 || target_pos > 135) {
      Serial.print("ERROR:Range 45-135, got ");
      Serial.println(target_pos);
      return;
    }
    
    // Nur bewegen wenn anders als aktuelle Position
    if(target_pos != current_position) {
      current_position = target_pos;
      
      // Servo bewegen
      lidarServo.write(current_position);
      
      is_moving = true;
      move_start_time = millis();
      
      // Bestätigung
      Serial.print("MOVING:");
      Serial.println(current_position);
      
      // LED an während Bewegung
      digitalWrite(LED_PIN, HIGH);
    } else {
      // Schon an Position
      Serial.print("REACHED:");
      Serial.println(current_position);
    }
  }
  
  // Home Position: "H"
  else if(cmd == "H" || cmd == "h") {
    if(current_position != 90) {
      current_position = 90;
      lidarServo.write(current_position);
      
      is_moving = true;
      move_start_time = millis();
      
      Serial.println("MOVING:90");
      digitalWrite(LED_PIN, HIGH);
    } else {
      Serial.println("REACHED:90");
    }
  }
  
  // Status abfragen: "?"
  else if(cmd == "?") {
    Serial.print("STATUS:");
    Serial.print(current_position);
    Serial.print(":");
    Serial.println(is_moving ? "MOVING" : "IDLE");
  }
  
  // Unbekannter Befehl
  else {
    Serial.print("ERROR:Unknown:");
    Serial.println(cmd);
  }
}

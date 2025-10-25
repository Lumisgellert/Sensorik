/*
 * 3D LIDAR Scanner - Arduino Servo Controller
 * 
 * Empfängt JSON-Befehle über Serial und steuert LIDAR-Neige-Servo
 * Sendet Bestätigung wenn Position erreicht ist
 * 
 * Verkabelung:
 * - Servo Signal → Pin 9
 * - Servo VCC → 5V (extern empfohlen!)
 * - Servo GND → GND
 * 
 * LIDAR-Orientierung:
 * - 90° = LIDAR horizontal (normaler 2D-Scan)
 * - 45° = Nach oben geneigt
 * - 135° = Nach unten geneigt
 */

#include <Servo.h>
#include <ArduinoJson.h>

#define SERVO_PIN 9

Servo lidarServo;

int current_position = 90;  // Startposition (horizontal)
int target_position = 90;
bool is_moving = false;
unsigned long move_start_time = 0;
const int SETTLING_TIME = 500;  // 500ms Wartezeit nach Bewegung

void setup() {
  Serial.begin(115200);
  
  lidarServo.attach(SERVO_PIN);
  lidarServo.write(current_position);
  
  delay(1000);
  
  // Startup-Nachricht
  StaticJsonDocument<100> doc;
  doc["status"] = "ready";
  doc["position"] = current_position;
  doc["range_min"] = 45;
  doc["range_max"] = 135;
  
  serializeJson(doc, Serial);
  Serial.println();
}

void loop() {
  // Befehle empfangen
  if(Serial.available()) {
    String json_string = Serial.readStringUntil('\n');
    processCommand(json_string);
  }
  
  // Prüfen ob Servo Position erreicht hat
  if(is_moving) {
    unsigned long elapsed = millis() - move_start_time;
    
    // Warten bis Servo Position erreicht hat + Settling Time
    if(elapsed >= SETTLING_TIME) {
      is_moving = false;
      current_position = target_position;
      sendPositionReached();
    }
  }
}

void processCommand(String json_string) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, json_string);
  
  if(error) {
    sendError("Invalid JSON");
    return;
  }
  
  // Kommando: Position setzen
  if(doc.containsKey("position")) {
    int pos = doc["position"];
    
    // Sicherheit: Bereich begrenzen
    if(pos < 45 || pos > 135) {
      sendError("Position out of range (45-135)");
      return;
    }
    
    target_position = pos;
    lidarServo.write(target_position);
    is_moving = true;
    move_start_time = millis();
    
    // Bestätigung senden
    StaticJsonDocument<100> response;
    response["status"] = "moving";
    response["target"] = target_position;
    response["current"] = current_position;
    
    serializeJson(response, Serial);
    Serial.println();
  }
  
  // Kommando: Aktuelle Position abfragen
  else if(doc.containsKey("get_position")) {
    StaticJsonDocument<100> response;
    response["status"] = "position";
    response["position"] = current_position;
    response["is_moving"] = is_moving;
    
    serializeJson(response, Serial);
    Serial.println();
  }
  
  // Kommando: Home Position (90°)
  else if(doc.containsKey("home")) {
    target_position = 90;
    lidarServo.write(target_position);
    is_moving = true;
    move_start_time = millis();
    
    StaticJsonDocument<100> response;
    response["status"] = "homing";
    
    serializeJson(response, Serial);
    Serial.println();
  }
}

void sendPositionReached() {
  StaticJsonDocument<100> doc;
  doc["status"] = "reached";
  doc["position"] = current_position;
  
  serializeJson(doc, Serial);
  Serial.println();
}

void sendError(const char* message) {
  StaticJsonDocument<100> doc;
  doc["status"] = "error";
  doc["message"] = message;
  
  serializeJson(doc, Serial);
  Serial.println();
}

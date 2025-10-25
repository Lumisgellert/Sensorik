/*
 * MINIMAL TEST - Serial Kommunikation
 * Sendet jede Sekunde eine Nachricht
 */

void setup() {
  Serial.begin(115200);
  
  // LED auf Pin 13 (eingebaut)
  pinMode(13, OUTPUT);
  
  delay(1000);
  
  Serial.println("=== ARDUINO TEST ===");
  Serial.println("Wenn du das liest, funktioniert Serial!");
  Serial.println("Arduino ist bereit.");
  Serial.println("");
}

void loop() {
  static int counter = 0;
  
  Serial.print("Test-Nachricht #");
  Serial.println(counter);
  counter++;
  
  // LED blinken
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}

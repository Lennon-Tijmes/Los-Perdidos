void setup() {
  Serial.begin(9600);    // For communication with the computer (monitor)
  Serial1.begin(9600);   // For communication with the HC-12 module
  delay(1000);
  Serial.println("Master Started");
}

unsigned long lastPollTime = 0;
const int pollInterval = 1000;  // 1 second between polls

// Slave IDs
const int numSlaves = 3;
const char slaveIDs[] = {'1', '2', '3'};
int currentSlave = 0;

void loop() {
  unsigned long currentTime = millis();

  // Poll slaves at regular intervals
  if (currentTime - lastPollTime >= pollInterval) {
    lastPollTime = currentTime;
    
    // Poll the current slave
    String pollCommand = String(slaveIDs[currentSlave]) + "?";
    Serial1.println(pollCommand);  // Poll command (e.g., "1?", "2?", "3?")
    Serial.print("Polling Slave ");
    Serial.println(slaveIDs[currentSlave]);

    // Wait for a response with a timeout
    unsigned long responseTimeout = millis();
    while (millis() - responseTimeout < 500) {
      if (Serial1.available()) {
        String response = Serial1.readStringUntil('\n');
        
        // Filter out self-responses (ensure response is valid)
        if (!response.startsWith("Received: " + String(slaveIDs[currentSlave]) + "?")) {
          Serial.print("Received: ");
          Serial.println(response);
          break;
        }
      }
    }

    // Move to the next slave
    currentSlave = (currentSlave + 1) % numSlaves;
    delay(200);  // Short delay to avoid immediate re-reading
  }
}
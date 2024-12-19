#define SLAVE_ID 1  // This slave's ID

void setup() {
  Serial.begin(9600);  // Initialize UART
  delay(1000);         // Allow HC-12 to initialize
  Serial.println("Slave started");
}

void loop() {
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(message);

    if (message.length() >= 2 && message[0] == SLAVE_ID + '0' && message[1] == '?') {
      Serial.print("Responding to Master: Slave ");
      Serial.print(SLAVE_ID);
      Serial.println(" Response");
    }
  }
}
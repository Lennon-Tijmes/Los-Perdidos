#define SLAVE_ID 3  // This slave's ID (Mort)

void setup() {
  Serial.begin(9600);  // Initialize UART for HC-12 communication
  delay(1000);         // Allow HC-12 module to initialize
  Serial.println("Slave (Mort) started");
}

void loop() {
  // Check if a message is received from the master
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(message);

    // If the message is a polling request for this slave
    if (message.length() >= 2 && message[0] == SLAVE_ID + '0' && message[1] == '?') {
    Serial.print("Responding to Master: Slave ");
    Serial.print(SLAVE_ID);
    Serial.println(" Response");
    Serial.println("Example Response Data"); // Send a response;

      // Simulate sensor data
      float speed = random(50, 150) / 10.0; // Random speed between 5.0 and 15.0
      float object_left = random(10, 100) / 10.0;  // Random object distance (left) in cm
      float object_right = random(10, 100) / 10.0; // Random object distance (right) in cm
      float object_middle = random(10, 100) / 10.0; // Random object distance (middle) in cm
      String line = "On Line";  // Example line state (replace with actual sensor value)

      // Create a response string in the format expected by the master
      String response = String(speed, 1) + "," +
                        String(object_left, 1) + "," +
                        String(object_right, 1) + "," +
                        String(object_middle, 1) + "," +
                        line;

      // Send the response back to the master
      Serial.println(response);
    }
  }
}

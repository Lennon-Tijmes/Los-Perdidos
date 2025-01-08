#include <WiFiS3.h> // Retain Wi-Fi library for later integration
#include <WiFiClient.h>

const char* ssid = "iotroam";        // Replace with your Wi-Fi SSID
const char* password = "JVzVtRgyWn"; // Replace with your Wi-Fi Password
const char* flaskServerIP = "141.252.143.84"; // Replace with Flask server IP
const int flaskServerPort = 5000;         // Flask server port (default 5000)

WiFiClient wifiClient;

const int numSlaves = 3;
const char slaveIDs[] = {'1', '2', '3'};
int currentSlave = 0;

unsigned long lastPollTime = 0;
const int pollInterval = 1000;  // 1 second between polls

void setup() {
  Serial.begin(9600);    // Debugging via Serial Monitor
  Serial1.begin(9600);   // Communication with HC-12 module
  delay(1000);

  Serial.println("Master Started");

  // Connect to Wi-Fi
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastPollTime >= pollInterval) {
    lastPollTime = currentTime;

    // Poll the current slave
    String pollCommand = String(slaveIDs[currentSlave]) + "?";
    Serial1.println(pollCommand);  // Send poll command
    Serial.println("Polling Slave " + String(slaveIDs[currentSlave]));

    // Wait for a response with a timeout
    String response = readSlaveResponse();
    if (!response.isEmpty()) {
      Serial.println("Received from Slave: " + response);

      // Send the data to Flask server
      sendDataToFlask(response, slaveIDs[currentSlave]);
    } else {
      Serial.println("No response from Slave " + String(slaveIDs[currentSlave]));
    }

    // Move to the next slave
    currentSlave = (currentSlave + 1) % numSlaves;
  }
}

String readSlaveResponse() {
  String response = "";
  unsigned long timeoutStart = millis();

  while (millis() - timeoutStart < 1000) { // 1-second timeout
    if (Serial1.available()) {
      response = Serial1.readStringUntil('\n');
      Serial.println("Raw Response: " + response);
      response.trim(); // Clean up response
      return response;
    }
  }

  return ""; // Return empty string if no response
}

void sendDataToFlask(const String& data, char slaveID) {
  if (wifiClient.connect(flaskServerIP, flaskServerPort)) {
    String jsonData = "{\"slave_id\": \"" + String(slaveID) + "\", \"data\": \"" + data + "\"}";
    wifiClient.println("POST /update HTTP/1.1");
    wifiClient.println("Host: " + String(flaskServerIP));
    wifiClient.println("Content-Type: application/json");
    wifiClient.print("Content-Length: ");
    wifiClient.println(jsonData.length());
    wifiClient.println();
    wifiClient.println(jsonData);

    // Wait for a response from the server
    unsigned long timeout = millis();
    while (wifiClient.connected() && millis() - timeout < 5000) {
      if (wifiClient.available()) {
        String response = wifiClient.readStringUntil('\n');
        Serial.println("Flask Response: " + response);
        break;
      }
    }

    wifiClient.stop(); // Close the connection
  } else {
    Serial.println("Failed to connect to Flask server");
  }
}

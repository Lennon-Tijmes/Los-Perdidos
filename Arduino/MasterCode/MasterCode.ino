#include <WiFiS3.h> // Retain Wi-Fi library for later integration
#include <WiFiClient.h>

const char* ssid = "iotroam";        // Replace with your Wi-Fi SSID
const char* password = "JVzVtRgyWn"; // Replace with your Wi-Fi Password
const char* flaskServerURL = "los-perdidos.onrender.com"; // Replace with your Render app URL
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

      // Assuming response is in a comma-separated format like:
      // "speed, object_left, object_right, object_middle, line_status"
      String data[5];
      int index = 0;
      int lastIndex = 0;

      // Parse the response into separate variables
      for (int i = 0; i < response.length(); i++) {
        if (response[i] == ',') {
          data[index] = response.substring(lastIndex, i);
          lastIndex = i + 1;
          index++;
        }
      }
      data[index] = response.substring(lastIndex); // The last part (line status)
      
      // Extract individual data values
      float speed = data[0].toFloat();
      float object_left = data[1].toFloat();
      float object_right = data[2].toFloat();
      float object_middle = (data[3].length() > 0) ? data[3].toFloat() : 0;
      String line_status = data[4];  // "On Line" or "Not On Line"

      // Send the data to Flask server
      sendDataToFlask(speed, object_left, object_right, object_middle, line_status, slaveIDs[currentSlave]);
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

void sendDataToFlask(float speed, float object_left, float object_right, float object_middle, String line_status, char slaveID) {
  Serial.println("Attempting to connect to Flask server...");

  if (wifiClient.connect("los-perdidos.onrender.com", 5000)) {
    // Format the data as a single query parameter
    String data = String(slaveID) + "," + String(speed) + "," + String(object_left) + "," + String(object_right) + "," + String(object_middle) + "," + line_status;
    
    // Construct the GET request URL with the single "data" parameter
    String url = "/update_robot_data?data=" + data;
    Serial.println(url);

    wifiClient.println("GET " + url + " HTTP/1.1");
    wifiClient.println("Host: los-perdidos.onrender.com");
    wifiClient.println("Connection: close");
    wifiClient.println();

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
    Serial.println("Connection closed.");
  } else {
    Serial.println("Failed to connect to Flask server");
  }
}


#include <WiFiS3.h> // Wi-Fi library
#include <WiFiSSLClient.h> // HTTPS communication

// Wi-Fi credentials
const char* ssid = "iotroam"; // Replace with your Wi-Fi SSID
const char* password = "JVzVtRgyWn"; // Replace with your Wi-Fi Password

// Render server details
const char* serverName = "los-perdidos.onrender.com"; // HTTPS server
const int serverPort = 443; // HTTPS port

// Robot communication
const int numSlaves = 3;
const char slaveIDs[] = {'1', '2', '3'};
int currentSlave = 0;

unsigned long lastPollTime = 0;
const int pollInterval = 1000; // 1 second between polls

void setup() {
  Serial.begin(115200);  // Debugging via Serial Monitor
  Serial1.begin(9600);   // Communication with HC-12 module
  delay(1000);

  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastPollTime >= pollInterval) {
    lastPollTime = currentTime;

    // Poll the current slave
    String pollCommand = String(slaveIDs[currentSlave]) + "?";
    Serial1.println(pollCommand); // Send poll command
    Serial.println("Polling Slave " + String(slaveIDs[currentSlave]));

    // Wait for a response with a timeout
    String response = readSlaveResponse();
    if (!response.isEmpty()) {
      Serial.println("Received from Slave: " + response);

      // Parse the response into separate variables
      String data[5];
      int index = 0;
      int lastIndex = 0;

      for (int i = 0; i < response.length(); i++) {
        if (response[i] == ',') {
          data[index] = response.substring(lastIndex, i);
          lastIndex = i + 1;
          index++;
        }
      }
      data[index] = response.substring(lastIndex);

      // Extract individual data values
      float speed = data[0].toFloat();
      float object_left = data[1].toFloat();
      float object_right = data[2].toFloat();
      float object_middle = (data[3].length() > 0) ? data[3].toFloat() : 0;
      String line_status = data[4]; // "On Line" or "Not On Line"

      // Encode the line_status for URL
      line_status.replace(" ", "%20");

      // Send the data to the website
      sendDataToWebsite(speed, object_left, object_right, object_middle, line_status, slaveIDs[currentSlave]);
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

void sendDataToWebsite(float speed, float object_left, float object_right, float object_middle, String line_status, char slaveID) {
  Serial.println("Attempting to connect to server...");

  WiFiSSLClient client; // Use WiFiSSLClient for HTTPS

  if (client.connect(serverName, serverPort)) {
    Serial.println("Connected to server!");

    // Format the data into the new URL structure
    String url = "/update_robot_data?";
    url += "robot_id=" + String(slaveID);
    url += "&speed=" + String(speed, 1);
    url += "&object_left=" + String(object_left, 1);
    url += "&object_right=" + String(object_right, 1);
    url += "&object_middle=" + String(object_middle, 1);
    url += "&line=" + line_status;

    // Replace spaces with %20 for URL encoding
    url.replace(" ", "%20");

    Serial.println("GET Request URL: " + url);

    // Send HTTPS GET request
    String getRequest = "GET " + url + " HTTP/1.1\r\n";
    getRequest += "Host: " + String(serverName) + "\r\n";
    getRequest += "Connection: close\r\n\r\n";

    client.print(getRequest);

    // Read server response
    while (client.connected() || client.available()) {
      if (client.available()) {
        String line = client.readStringUntil('\n');
        Serial.println(line);
      }
    }

    client.stop();
    Serial.println("Connection closed.");
  } else {
    Serial.println("Failed to connect to server.");
  }
}


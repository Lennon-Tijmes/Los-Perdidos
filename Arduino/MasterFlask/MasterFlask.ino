#include <WiFiS3.h>
#include <WiFiSSLClient.h> // Use WiFiSSLClient for HTTPS connections

// Wi-Fi credentials
const char* ssid = "iotroam";          // Replace with your Wi-Fi SSID
const char* pass = "JVzVtRgyWn";       // Replace with your Wi-Fi password

// Render server details
const char* serverName = "los-perdidos.onrender.com"; // HTTPS server
const int serverPort = 443;                           // HTTPS port

void setup() {
  Serial.begin(115200); // Start Serial Monitor for debugging
  delay(1000);

  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to Wi-Fi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiSSLClient client; // Use WiFiSSLClient for HTTPS

    Serial.println("Wi-Fi connected!");

    // Attempt to connect to the server
    if (client.connect(serverName, serverPort)) {
      Serial.println("Connected to server!");

      // Send HTTPS GET request
      String getRequest = "GET /update_robot_data?data=3,0.9,2.1,1.7,1.4,On%20Line HTTP/1.1\r\n";
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
  } else {
    Serial.println("Wi-Fi not connected.");
  }

  delay(10000); // Wait 10 seconds before the next request
}

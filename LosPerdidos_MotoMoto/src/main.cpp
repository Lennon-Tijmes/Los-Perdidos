#include <Arduino.h>

#define motorLFFullSpeed 220
#define motorRFFullSpeed 253
#define motorLBFullSpeed 243
#define motorRBFullSpeed 253
#define motorLHalfSpeed 177
#define motorRHalfSpeed 177
#define motorStop 0
#define motorLB 5
#define motorRB 10
#define motorLF 9
#define motorRF 6
#define trigpin 12
#define echopin 11

const int sensorPins[] = {A0, A1, A2, A3, A4, A5, A6, A7};  // Line sensors
const int sensorCount = sizeof(sensorPins) / sizeof(sensorPins[0]);
const int threshold = 10; // Distance threshold in cm for obstacle detection

void setup() {
    // Ultrasonic sensor pins
  pinMode(trigpin, OUTPUT); // Trig pin as output
  pinMode(echopin, INPUT);  // Echo pin as input

  pinMode(motorLF, OUTPUT);
  pinMode(motorRF, OUTPUT);
  pinMode(motorLB, OUTPUT);
  pinMode(motorRB, OUTPUT);
    // Line sensor pins
  for (int i = 0; i < sensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  Serial.begin(9600); // Initialize serial communication
}


int getDistance() {
  // Function to measure distance using the ultrasonic sensor
  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin, LOW);

  long duration = pulseIn(echopin, HIGH);
  int distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}
void PassObject() {
  stopBot();
  delay(500);
  goRight();
  delay(700);
  goForwards();
  delay(1000);
  goLeft();
  delay(700);
  followLine();
}
void followLine() {
  int error = 0;
  int baseSpeed = 255; // Base speed for motors

  // Read sensors and calculate error
  for (int i = 0; i < sensorCount; i++) {
    int sensorValue = analogRead(sensorPins[i]);
    if (sensorValue > 500) { // Adjust this threshold as needed
      error += (i - sensorCount / 2); // Center the error calculation
    }
  }

  int correction = error * 10; // Adjust correction factor as needed
  int LSpeed = constrain(baseSpeed - correction, 0, 255);
  int RSpeed = constrain(baseSpeed + correction, 0, 255);

  // Set motor speeds
  analogWrite(motorLF, LSpeed);
  analogWrite(motorRF, RSpeed);
}

void goForwards()
{
  analogWrite(motorLF, motorLFFullSpeed);
  analogWrite(motorRF, motorRFFullSpeed);
  analogWrite(motorLB, motorStop);
  analogWrite(motorRB, motorStop);
}

void goBackwards()  
{ 
  analogWrite(motorLB, motorLBFullSpeed);
  analogWrite(motorRB, motorRBFullSpeed);
  analogWrite(motorLF, motorStop);
  analogWrite(motorRF, motorStop);
}

void goRight()
{
  analogWrite(motorLF, motorLFFullSpeed);
  analogWrite(motorRF, motorRHalfSpeed);
  analogWrite(motorLB, motorStop);
  analogWrite(motorRB, motorStop);
}

void goLeft()
{
  analogWrite(motorLF, motorLHalfSpeed);
  analogWrite(motorRF, motorRFFullSpeed);
  analogWrite(motorLB, motorStop);
  analogWrite(motorRB, motorStop);
}

void rotateLeft()
{
  analogWrite(motorLB, motorLBFullSpeed);
  analogWrite(motorRB, motorStop);
  analogWrite(motorRF, motorRFFullSpeed);
  analogWrite(motorLF, motorStop);
}

void rotateRight()
{
  analogWrite(motorRB, motorRBFullSpeed);
  analogWrite(motorLB, motorStop);
  analogWrite(motorLF, motorLFFullSpeed);
  analogWrite(motorRF, motorStop);
}
void stopBot()
{
  analogWrite(motorRB, motorStop);
  analogWrite(motorLB, motorStop);
  analogWrite(motorLF, motorStop);
  analogWrite(motorRF, motorStop);
}
void loop() {
  int distance = getDistance();
  if (distance < threshold) {
    PassObject(); // Call the function to pass the object
  } else {
    followLine(); // Follow the line if no object is detected
    
  }
}
/*
  goForwards();
  delay(5000);
  goBackwards();
  delay(5000);
  goRight();
  delay(2000);
  goLeft();
  delay(2000);
  rotateLeft();
  delay(500);
*/
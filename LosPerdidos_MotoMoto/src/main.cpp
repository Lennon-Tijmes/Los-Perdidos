#include <Arduino.h>

#define TRIG_PIN 12
#define ECHO_PIN 11

// Motor Pins
#define LEFT_MOTOR_FORWARD 9
#define LEFT_MOTOR_BACKWARD 5
#define RIGHT_MOTOR_FORWARD 6
#define RIGHT_MOTOR_BACKWARD 10

// Distance Threshold for Obstacle (in cm)
#define OBSTACLE_DISTANCE 20

#define LMOTOR_HALF_SPEED 130
#define RMOTOR_HALF_SPEED 130
#define RMOTOR_FULL_SPEED 255
#define LMOTOR_FULL_SPEED 255
#define MOTOR_STOP 0

const int sensorPins[] = {A0, A1, A2, A3, A4, A5, A6, A7};  // Line sensors
const int sensorCount = sizeof(sensorPins) / sizeof(sensorPins[0]);

void setup() {
  // Ultrasonic Sensor Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Motor Control Pins
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  // Line sensor pins
  for (int i = 0; i < sensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  Serial.begin(9600); // For debugging
}

void loop() {
  long distance = readDistance();

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance > 0 && distance < OBSTACLE_DISTANCE)
  {
    // Obstacle detected, avoid it
    stopMotors();
    delay(200); // Pause for clarity
    passObject();
  } 
  else 
  {
    // No obstacle, keep moving forward
    followLine();
  }

  delay(100); // Short delay for stability
}

// Function to read distance from HC-SR04
long readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

// Function to read line sensor values and follow the line
void followLine() {
  int sensorValues[sensorCount];
  int position = 0;
  int totalValue = 0;

  // Read sensor values
  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    position += sensorValues[i] * (i + 1); // Weighted sum
    totalValue += sensorValues[i];
  }

  // Calculate the line position
  if (totalValue > 0) {
    position /= totalValue; // Normalize to get the line position
  } else {
    position = -1; // No line detected
  }

  Serial.print("Line Position: ");
  Serial.println(position);

  // Control motors based on the line position
  if (position == -1) {
    // No line detected, stop
    stopMotors();
  } else if (position < sensorCount / 2) {
    // Line is to the left
    goLeft();
  } else if (position > sensorCount / 2) {
    // Line is to the right
    goRight();
  } else {
    // Line is centered
    moveForward();
  }
}

// Function to move forward
void moveForward() 
{
  digitalWrite(LEFT_MOTOR_FORWARD, LMOTOR_FULL_SPEED);
  digitalWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
  digitalWrite(RIGHT_MOTOR_FORWARD, RMOTOR_FULL_SPEED);
  digitalWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
}

// Function to stop motors
void stopMotors() 
{
  digitalWrite(LEFT_MOTOR_FORWARD, MOTOR_STOP);
  digitalWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
  digitalWrite(RIGHT_MOTOR_FORWARD, MOTOR_STOP);
  digitalWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
}

// Function to move backward
void moveBackward() 
{
  digitalWrite(LEFT_MOTOR_FORWARD, MOTOR_STOP);
  digitalWrite(LEFT_MOTOR_BACKWARD, LMOTOR_FULL_SPEED);
  digitalWrite(RIGHT_MOTOR_FORWARD, MOTOR_STOP);
  digitalWrite(RIGHT_MOTOR_BACKWARD, RMOTOR_FULL_SPEED);
}

// Function to avoid obstacle
void passObject() 
{
  
  stopMotors();
  delay(500);
  goRight();
  delay(700);
  moveForward();
  delay(1000);
  goLeft();
  delay(700);
  moveForward();
  delay(1000);
  goLeft();
  delay(700);
  moveForward();
  delay(1000);
  
}
  void goRight()
  {
  digitalWrite(LEFT_MOTOR_FORWARD, LMOTOR_HALF_SPEED);
  digitalWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
  digitalWrite(RIGHT_MOTOR_FORWARD, MOTOR_STOP);
  digitalWrite(RIGHT_MOTOR_BACKWARD, RMOTOR_HALF_SPEED);
  delay(500); // Adjust turning time based on your bot
  stopMotors();
  }
  
  void goLeft()
  {
    digitalWrite(RIGHT_MOTOR_FORWARD, RMOTOR_HALF_SPEED);
    digitalWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
    digitalWrite(LEFT_MOTOR_FORWARD, MOTOR_STOP);
    digitalWrite(LEFT_MOTOR_BACKWARD, LMOTOR_HALF_SPEED);
    delay(500);
    stopMotors();
  }

  //pass the object
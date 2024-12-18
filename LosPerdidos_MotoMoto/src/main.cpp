#include <Arduino.h>

#define MOTOR_LF_PIN 9
#define MOTOR_RF_PIN 6
#define MOTOR_LB_PIN 5
#define MOTOR_RB_PIN 10
#define TRIG_PIN 12
#define ECHO_PIN 11
#define GRIPPER_PIN 8
#define NEOPIXEL_PIN 13

// Motor speed controls
#define MOTOR_LF_SPEED 215
#define MOTOR_RF_SPEED 220
#define MOTOR_LB_SPEED 210
#define MOTOR_RB_SPEED 220
#define MOTOR_STOP 0

#define GRIPPER_OPEN   1600  // Value for the gripper to be open
#define GRIPPER_CLOSED 1010  // Value for the gripper to be closed

// Line sensors
const int LIGHT_SENSORS[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int LIGHT_THRESHOLD = 850;

// Auxiliary variables
float distance = 0;
bool gripperClosed = false; // Current state of the gripper (open/closed)

void setup() {
  pinMode(MOTOR_LF_PIN, OUTPUT);
  pinMode(MOTOR_RF_PIN, OUTPUT);
  pinMode(MOTOR_LB_PIN, OUTPUT);
  pinMode(MOTOR_RB_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(GRIPPER_PIN, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(LIGHT_SENSORS[i], INPUT);
  }

  Serial.begin(9600);
  stopAllMotors();
  setGripper(GRIPPER_OPEN); // Start with the gripper open
}

void loop() {
  measureDistance();

  // Close the gripper if the distance is less than or equal to 10 cm
  if (distance <= 10 && !gripperClosed) {
    setGripper(GRIPPER_CLOSED);
    gripperClosed = true; // Update the state
  }
  else if (distance > 10 && gripperClosed) {
    setGripper(GRIPPER_OPEN);
    gripperClosed = false; // Update the state
  }
  followLine();
}

// Measure distance with the ultrasonic sensor
void measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  float duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2; // Convert to cm
}

// Gripper functions
void setGripper(int pulse) {
  for (int i = 0; i < 8; i++) { // Maintain the pulse for 1 second
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GRIPPER_PIN, LOW);
  }
}

// Motor control functions
void followLine() {
  int left = analogRead(LIGHT_SENSORS[0]) > LIGHT_THRESHOLD || analogRead(LIGHT_SENSORS[1]) > LIGHT_THRESHOLD;
  int right = analogRead(LIGHT_SENSORS[6]) > LIGHT_THRESHOLD || analogRead(LIGHT_SENSORS[7]) > LIGHT_THRESHOLD;
  int center = analogRead(LIGHT_SENSORS[2]) > LIGHT_THRESHOLD || analogRead(LIGHT_SENSORS[3]) > LIGHT_THRESHOLD;

  // Line detection and direction control
  if (left && !right) {
    goRight(); // If the left side is off the line, turn right
  } else if (right && !left) {
    goLeft(); // If the right side is off the line, turn left
  } else if (center) {
    driveForward(); // If the center detects the line, move forward
  } else {
    stopAllMotors(); // If no sensor detects the line, stop the robot
  }
}

// Motor movement functions
void driveForward() {
  analogWrite(MOTOR_LF_PIN, MOTOR_LF_SPEED);
  analogWrite(MOTOR_RF_PIN, MOTOR_RF_SPEED);
  analogWrite(MOTOR_LB_PIN, MOTOR_STOP);
  analogWrite(MOTOR_RB_PIN, MOTOR_STOP);
}

void goLeft() {
  analogWrite(MOTOR_LB_PIN, MOTOR_STOP);
  analogWrite(MOTOR_RF_PIN, MOTOR_RF_SPEED);
  analogWrite(MOTOR_LF_PIN, 187); // Adjust left motor speed
  analogWrite(MOTOR_RB_PIN, MOTOR_STOP);
}

void goRight() {
  analogWrite(MOTOR_LB_PIN, MOTOR_STOP);
  analogWrite(MOTOR_RF_PIN, 190); // Adjust right motor speed
  analogWrite(MOTOR_LF_PIN, MOTOR_LF_SPEED);
  analogWrite(MOTOR_RB_PIN, MOTOR_STOP);
}

// Stop all motors function
void stopAllMotors() {
  analogWrite(MOTOR_LF_PIN, MOTOR_STOP);
  analogWrite(MOTOR_RF_PIN, MOTOR_STOP);
  analogWrite(MOTOR_LB_PIN, MOTOR_STOP);
  analogWrite(MOTOR_RB_PIN, MOTOR_STOP);
}

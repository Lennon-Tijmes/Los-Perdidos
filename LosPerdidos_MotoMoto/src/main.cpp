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
#define BASE_SPEED 200  // Adjust as necessary
#define Kp 20           // Proportional constant for speed adjustment


unsigned long previousMillis = 0;
const int TURN_DURATION = 700;
const int FORWARD_DURATION = 1000;
const int PAUSE_DURATION = 600;
long distance = -1;
unsigned int colorBlack = 900;
unsigned int colorWhite = 500;
bool allBlack = false;
bool allWhite = false;

const unsigned char LINE_SENSOR[] = {A7, A6, A5, A4, A3, A2, A1, A0};
int lineSensorValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// State variable for obstacle passing
enum State { STOP, TURN_RIGHT, MOVE_FORWARD, TURN_LEFT, RETURN_FORWARD, RETURN_LEFT, RETURN_FORWARD_A, RETURN_LEFT_A, DONE };
State currentState = STOP;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  Serial.begin(9600); // For debugging

  for (char i=0; i < 8; i++) 
  {
    pinMode(LINE_SENSOR[i], INPUT);
  }
}

void loop() {
  
  readLineSensor();
  followLine();
  readDistance();
  // if (distance > 0 && distance < OBSTACLE_DISTANCE) {
  //   avoidObject();
  // } else {
  //   followLine2();
  // }

}

// Function to move forward
void moveForward() {
  analogWrite(LEFT_MOTOR_FORWARD, LMOTOR_FULL_SPEED);
  analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
  analogWrite(RIGHT_MOTOR_FORWARD, RMOTOR_FULL_SPEED);
  analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
}

// Function to stop motors
void stopMotors() {
  analogWrite(LEFT_MOTOR_FORWARD, MOTOR_STOP);
  analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
  analogWrite(RIGHT_MOTOR_FORWARD, MOTOR_STOP);
  analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
}

// Function to avoid obstacle
void passObject() {
  unsigned long currentMillis = millis();

  switch (currentState) {
    case STOP:
      stopMotors();
      if (currentMillis - previousMillis >= PAUSE_DURATION) {
        previousMillis = currentMillis;
        currentState = TURN_RIGHT; // Turn right to start avoiding obstacle
      }
      break;

    case TURN_RIGHT:
      goRight();
      if (currentMillis - previousMillis >= TURN_DURATION) {
        previousMillis = currentMillis;
        currentState = MOVE_FORWARD; // Move forward past the obstacle
      }
      break;

    case MOVE_FORWARD:
      moveForward();
      if (currentMillis - previousMillis >= FORWARD_DURATION) {
        previousMillis = currentMillis;
        currentState = TURN_LEFT; // Turn back to the original direction
      }
      break;

    case TURN_LEFT:
      goLeft();
      if (currentMillis - previousMillis >= TURN_DURATION) {
        previousMillis = currentMillis;
        currentState = RETURN_FORWARD; // Move back to align with the original path
      }
      break;

    case RETURN_FORWARD:
      moveForward();
      if (currentMillis - previousMillis >= FORWARD_DURATION) {
        previousMillis = currentMillis;
        currentState = RETURN_LEFT; // Complete return sequence
      }
      break;

    case RETURN_LEFT:
      goLeft(); // Turn opposite to the first turn
      if (currentMillis - previousMillis >= TURN_DURATION) {
        previousMillis = currentMillis;
        currentState = DONE; // Finished obstacle avoidance
      }
      break;

    case DONE:
      moveForward(); // Optionally pause before resuming normal operation
      break;
  } 
}

// Function to go right
void goRight() {
  analogWrite(LEFT_MOTOR_FORWARD, LMOTOR_HALF_SPEED);
  analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
  analogWrite(RIGHT_MOTOR_FORWARD, MOTOR_STOP);
  analogWrite(RIGHT_MOTOR_BACKWARD, RMOTOR_HALF_SPEED);
}

// Function to go left
void goLeft() {
  analogWrite(RIGHT_MOTOR_FORWARD, RMOTOR_HALF_SPEED);
  analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
  analogWrite(LEFT_MOTOR_FORWARD, MOTOR_STOP);
  analogWrite(LEFT_MOTOR_BACKWARD, LMOTOR_HALF_SPEED);
}

// Function to read distance
void readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2; // Convert to cm
}

// Function to read line sensor values
void readLineSensor() {
  allBlack= true;
  allWhite= true;

   
  for (unsigned char i =0; i < 8; i++){
    lineSensorValue[i] = analogRead(LINE_SENSOR[i]);
  }
  allBlack = (lineSensorValue[0] >= colorBlack) && (lineSensorValue[1] >= colorBlack) && (lineSensorValue[2] >= colorBlack) && (lineSensorValue[3] >= colorBlack) && (lineSensorValue[4] >= colorBlack) && (lineSensorValue[5] >= colorBlack); // true if all sensors are on black

  allWhite = (lineSensorValue[0] <= colorWhite) && (lineSensorValue[1] <= colorWhite) && (lineSensorValue[2] <= colorWhite) && (lineSensorValue[3] <= colorWhite) && (lineSensorValue[4] <= colorWhite) && (lineSensorValue[5] <= colorWhite); // true if all sensors are on white
}

// Function to avoid object
void avoidObject() {
  if (distance > 0 && distance < OBSTACLE_DISTANCE) {
    stopMotors();
    analogWrite(LEFT_MOTOR_FORWARD, 200);
    analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
    analogWrite(RIGHT_MOTOR_FORWARD, 155);
    analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
    delay(500);
    moveForward();
    delay(2000);
    stopMotors();
    while (allWhite) {
      readLineSensor();
      analogWrite(LEFT_MOTOR_FORWARD, 200);
      analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
      analogWrite(RIGHT_MOTOR_FORWARD, 255);
      analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
    }
  }
}

// Function to follow line
void followLine() {
  bool forwards = (lineSensorValue[3] >= colorBlack) && (lineSensorValue[4] >= colorBlack);
  bool turnLeft = (lineSensorValue[0] >= colorBlack) || (lineSensorValue[1] >= colorBlack) || (lineSensorValue[2] >= colorBlack);
  bool turnRight = (lineSensorValue[5] >= colorBlack) || (lineSensorValue[6] >= colorBlack) || (lineSensorValue[7] >= colorBlack);
  unsigned char lastValue = 0;

  if (!allWhite) {
    if (forwards == true) {
      moveForward();
    }
    else if (turnRight == true) {
      analogWrite(LEFT_MOTOR_FORWARD, 250);
      analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
      analogWrite(RIGHT_MOTOR_FORWARD, MOTOR_STOP);
      analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
      lastValue = 1;
    }
    else if (turnLeft == true) {
      analogWrite(LEFT_MOTOR_FORWARD, MOTOR_STOP);
      analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
      analogWrite(RIGHT_MOTOR_FORWARD, 250);
      analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
      lastValue = 2;
    }
  }
  else {
    if (lastValue == 1) {
      analogWrite(LEFT_MOTOR_FORWARD, 250);
      analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
      analogWrite(RIGHT_MOTOR_FORWARD, MOTOR_STOP);
      analogWrite(RIGHT_MOTOR_BACKWARD, 200);
    }
    else {
      analogWrite(LEFT_MOTOR_FORWARD, MOTOR_STOP);
      analogWrite(LEFT_MOTOR_BACKWARD, 200);
      analogWrite(RIGHT_MOTOR_FORWARD, 250);
      analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
    }
  }
}

void followLine2() {
  int error = 0;
  int weights[] = {-3, -2, -1, 0, 0, 1, 2, 3}; // Adjust weights based on sensor placement
  
  bool onLine = false;

  // Calculate the error based on sensor readings
  for (unsigned char i = 0; i < 8; i++) {
    if (lineSensorValue[i] >= colorBlack) { // Check if sensor detects black
      error += weights[i];
      onLine = true; // At least one sensor is on the line
    }
  }

  if (onLine) {
    // Proportional control for motor speeds
    int leftSpeed = BASE_SPEED - (Kp * error);
    int rightSpeed = BASE_SPEED + (Kp * error);

    // Constrain motor speeds to valid PWM range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Drive the motors
    analogWrite(LEFT_MOTOR_FORWARD, leftSpeed);
    analogWrite(RIGHT_MOTOR_FORWARD, rightSpeed);
  } else if (allWhite) {
    // Handle lost line scenario
    stopMotors();
    Serial.println("Lost the line!"); // Debugging message
  } else if (allBlack) {
    // Handle junction scenario
    moveForward();
    Serial.println("Junction detected!"); // Debugging message
  }
}


void readLineSensor2() {
  // Reset allBlack and allWhite flags
  allBlack = true;
  allWhite = true;

  // Iterate through all sensors to read values
  for (unsigned char i = 0; i < 8; i++) {
    lineSensorValue[i] = analogRead(LINE_SENSOR[i]);

    // Check if any sensor is not black or white
    if (lineSensorValue[i] < colorBlack) {
      allBlack = false; // At least one sensor is not black
    }
    if (lineSensorValue[i] > colorWhite) {
      allWhite = false; // At least one sensor is not white
    }
  }

  // Debugging: Print sensor values to the Serial Monitor (optional)
  Serial.print("Line Sensor Values: ");
  for (unsigned char i = 0; i < 8; i++) {
    Serial.print(lineSensorValue[i]);
    Serial.print(" ");
  }
  Serial.print(" | allBlack: ");
  Serial.print(allBlack);
  Serial.print(" | allWhite: ");
  Serial.println(allWhite);
}
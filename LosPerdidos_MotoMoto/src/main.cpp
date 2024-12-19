#include <Arduino.h>

#define TRIG_PIN 12
#define ECHO_PIN 11

// Motor Pins
#define LEFT_MOTOR_FORWARD 9
#define LEFT_MOTOR_BACKWARD 5
#define RIGHT_MOTOR_FORWARD 6
#define RIGHT_MOTOR_BACKWARD 10

// Distance Threshold for Obstacle (in cm)
//#define OBSTACLE_DISTANCE 20

#define LMOTOR_HALF_SPEED 107
#define RMOTOR_HALF_SPEED 110
#define RMOTOR_FULL_SPEED 220
#define LMOTOR_FULL_SPEED 215
#define MOTOR_STOP 0

//Gripper
#define GRIPPER_OPEN   1600  
#define GRIPPER_CLOSED 1010
#define GRIPPER_PIN 8


unsigned long previousMillis = 0;
const int TURN_DURATION = 700;
const int FORWARD_DURATION = 1000;
const int PAUSE_DURATION = 600;
long distance = -1;
unsigned int colorBlack = 500;
unsigned int colorWhite = 900;
bool allBlack = false;
bool allWhite = false;

const unsigned char LINE_SENSOR[] = {A7, A6, A5, A4, A3, A2, A1, A0};
int lineSensorValue[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// State variable for obstacle passing

bool gripperClosed = false;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(GRIPPER_PIN, OUTPUT);

  Serial.begin(9600); // For debugging

  for (char i=0; i < 8; ++i) 
  {
    pinMode(LINE_SENSOR[i], INPUT);
  }
  setGripper(GRIPPER_OPEN); // Inicializa o gripper aberto

  // Ações antes do loop principal
  delay(1000);
  moveForward();                 // Faz o robô avançar
  delay(5000);                   // Avança por 2 segundos
  stopMotors();
  delay(5000);                  // Para os motores
  setGripper(GRIPPER_CLOSED);    // Fecha o gripper
  delay(1000);                   // Aguarda um momento para garantir que o gripper feche completamente
}

void loop() {
  followLine();
  /*if (distance <= 10 && !gripperClosed) {
    setGripper(GRIPPER_CLOSED);
    gripperClosed = true;
  }
  else if (distance > 10 && gripperClosed) {
    setGripper(GRIPPER_OPEN);
    gripperClosed = false; 
  }*/
  readDistance();
  readLineSensor();
  avoidObject();
  delay(100);
  
}
void start(){
  
}
// Function to move forward
void moveForward() {
  Serial.println("Moving Forward...");
  analogWrite(LEFT_MOTOR_FORWARD, LMOTOR_FULL_SPEED);
  analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
  analogWrite(RIGHT_MOTOR_FORWARD, RMOTOR_FULL_SPEED);
  analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
}


// Function to stop motors
void stopMotors() {
  Serial.println("Stop Motors...");
  analogWrite(LEFT_MOTOR_FORWARD, MOTOR_STOP);
  analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
  analogWrite(RIGHT_MOTOR_FORWARD, MOTOR_STOP);
  analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
}
void measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  float duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2; 
} 

void setGripper(int pulse) {
  Serial.println("Grip...");
  for (int i = 0; i < 8; i++) { 
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GRIPPER_PIN, LOW);
    delay(20); 
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
  for (unsigned char i = 0; i < 8; i++) {
    lineSensorValue[i] = analogRead(LINE_SENSOR[i]);
  }

  allBlack = (lineSensorValue[0] >= colorBlack) && (lineSensorValue[1] >= colorBlack) && (lineSensorValue[2] >= colorBlack) && (lineSensorValue[3] >= colorBlack) && (lineSensorValue[4] >= colorBlack) && (lineSensorValue[5] >= colorBlack); // true if all sensors are on black

  allWhite = (lineSensorValue[0] <= colorWhite) && (lineSensorValue[1] <= colorWhite) && (lineSensorValue[2] <= colorWhite) && (lineSensorValue[3] <= colorWhite) && (lineSensorValue[4] <= colorWhite) && (lineSensorValue[5] <= colorWhite); // true if all sensors are on white
}

// Function to avoid object
void avoidObject() {
  if (allWhite) {
    readLineSensor();
      analogWrite(LEFT_MOTOR_FORWARD, 150);
      analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
      analogWrite(RIGHT_MOTOR_FORWARD, 255);
      analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
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
      analogWrite(RIGHT_MOTOR_FORWARD, 180);
      analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
      lastValue = 1;
    }
    else if (turnLeft == true) {
      analogWrite(LEFT_MOTOR_FORWARD, 180);
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
      analogWrite(RIGHT_MOTOR_FORWARD, 180);
      analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
    }
    else {
      analogWrite(LEFT_MOTOR_FORWARD, 180);
      analogWrite(LEFT_MOTOR_BACKWARD, MOTOR_STOP);
      analogWrite(RIGHT_MOTOR_FORWARD, 250);
      analogWrite(RIGHT_MOTOR_BACKWARD, MOTOR_STOP);
    }
  }
}
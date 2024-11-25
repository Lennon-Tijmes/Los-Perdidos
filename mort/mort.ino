// Section for the motors
#define MOTOR_LF           9    // Forwards left motor
#define MOTOR_RF           10   // Forwards right motor
#define MOTOR_LB           7    // Backwards left motor
#define MOTOR_RB           8    // Backwards right motor
#define MOTOR_L_FULL_SPEED 250  // Left motor full speed
#define MOTOR_L_HALF_SPEED 140  // Left motor half speed
#define MOTOR_R_FULL_SPEED 255  // Right motor full speed
#define MOTOR_R_HALF_SPEED 140  // Right motor half speed
#define MOTOR_STOP         0    // Motor stopping speed
#define MOTOR_LR           2    // Left rotation sensor
#define MOTOR_RR           3    // Right rotation sensor

// Section for the servo motor
#define GRIPPER_PIN        4     // Pin for the servo gripper
#define GRIPPER_OPEN       1600  // Value for gripper being open
#define GRIPPER_CLOSED     1010  // Value for gripper being closed

// Section for the Ultrasonic distance sensor
#define SONAR_TRIG_PIN     12    // Sonar trig pin
#define SONAR_ECHO_PIN     13    // Sonar echo pin 
int distance = 999;

// Section for the line sensor
const int LINE_SENSOR[] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Array for line sensor pins
int lineSensorValue[8] = {0};                               // Array for line sensor values

// Section for the Rotation values
int LRRotations = 0;
int RRRotations = 0;
const unsigned long debounce = 10;

// Code to run once
void setup() 
{
  Serial.begin(9600);
  pinMode(MOTOR_LF, OUTPUT);
  pinMode(MOTOR_RF, OUTPUT);
  pinMode(MOTOR_LB, OUTPUT);
  pinMode(MOTOR_RB, OUTPUT);
  pinMode(MOTOR_LR, INPUT_PULLUP);
  pinMode(MOTOR_RR, INPUT_PULLUP);
  pinMode(GRIPPER_PIN, OUTPUT);
  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  for (int i = 0; i < 7; i++) {
    pinMode(LINE_SENSOR[i], INPUT);  // sets every linesensor pin on the correct pinmode
  }
  setGripper(GRIPPER_OPEN);
  attachInterrupt(digitalPinToInterrupt(MOTOR_LR), rotateLR, CHANGE); //interrupt activates when rotation sensor changes
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR), rotateRR, CHANGE); //interrupt activates when rotation sensor changes
}

// Code to keep repeating
void loop() 
{
  readLineSensor(); 
}

// Counts the interrupts of the rotation sensor for the left wheel
void rotateLR()
{
  static unsigned long timer;
  static bool lastState;
  noInterrupts();
  if (millis() > timer)
  {
    bool state = digitalRead(MOTOR_LR);
    if(state != lastState)
    {
      LRRotations++;
      lastState = state;
    }
    timer = millis() + debounce;
  }
  interrupts();
}

// Counts the interrupts of the rotation sensor for the right wheel
void rotateRR() 
{
  static unsigned long timer;
  static bool lastState;
  noInterrupts();
  if (millis() > timer)
  {
    bool state = digitalRead(MOTOR_RR);
    if(state != lastState)
    {
      RRRotations++;
      lastState = state;
    }
    timer = millis() + debounce;
  }
  interrupts();
}

// Makes the relaybot drive in a straight line forward
// TODO: Make the robot drive a certain speed, calibrated with rotation sensor
void goForwards(int speed)
{
  LRRotations = 0;
  RRRotations = 0;
  Serial.print("Left: ");
  Serial.print(LRRotations);
  Serial.print("  Right: ");
  Serial.println(RRRotations);
  analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
  analogWrite(MOTOR_RF, MOTOR_R_FULL_SPEED);
  digitalWrite(MOTOR_LB, MOTOR_STOP);
  digitalWrite(MOTOR_RB, MOTOR_STOP);
}

// Makes the relaybot drive in a straight line backwards
// TODO: Make the robot drive a certain speed, calibrated with rotation sensor
void goBackwards(int speed)
{
  digitalWrite(MOTOR_LB, 1);
  digitalWrite(MOTOR_RB, 1);
  analogWrite(MOTOR_LF, (-MOTOR_L_FULL_SPEED - speed));
  analogWrite(MOTOR_RF, (MOTOR_R_FULL_SPEED - speed));
}

// Stops all the motors
void stopDriving()
{
  Serial.print("Left: ");
  Serial.print(LRRotations);
  Serial.print("  Right: ");
  Serial.println(RRRotations);
  digitalWrite(MOTOR_LB, MOTOR_STOP);
  digitalWrite(MOTOR_RB, MOTOR_STOP);
  analogWrite(MOTOR_LF, MOTOR_STOP);
  analogWrite(MOTOR_RF, MOTOR_STOP); 
}

// Rotate the relaybot on its axis to the right
// TODO: Make it turn correctly
void rotateRight()
{
  stopDriving();
  delay(500);
  RRRotations = 0;
  // TODO: make the magic number less magical 
  while (RRRotations < 12)
  {
    Serial.print("Right:"); 
    Serial.println(RRRotations);
    digitalWrite(MOTOR_LB, MOTOR_STOP);
    digitalWrite(MOTOR_RB, 1);
    analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
    analogWrite(MOTOR_RF, MOTOR_STOP); 
  }
  stopDriving();
}

// Rotate the relaybot on its axis to the left
// TODO: Make it turn correctly
void rotateLeft()
{
  stopDriving();
  delay(500);
  LRRotations = 0;
  // TODO: make the magic number less magical 
  while (LRRotations < 12)
  {
    Serial.print("Left:");
    Serial.println(LRRotations); 
    digitalWrite(MOTOR_RB, MOTOR_STOP);
    digitalWrite(MOTOR_LB, 1);
    analogWrite(MOTOR_RF, MOTOR_L_FULL_SPEED);
    analogWrite(MOTOR_LF, MOTOR_STOP); 
  }
  stopDriving();
}

// Function for making a left turn
// TODO: Make it turn smoothly using the rotation sensors
void turnLeft()
{
  LRRotations = 0;
  RRRotations = 0;
  while (LRRotations < 60 && RRRotations < 50)
  {
    Serial.print("Left: ");
    Serial.print(LRRotations);
    Serial.print("  Right: ");
    Serial.println(RRRotations);
    digitalWrite(MOTOR_RB, MOTOR_STOP);
    digitalWrite(MOTOR_LB, MOTOR_STOP);
    analogWrite(MOTOR_RF, MOTOR_R_FULL_SPEED);
    analogWrite(MOTOR_LF, MOTOR_L_HALF_SPEED);
  }
  stopDriving();
}

// Function for making a right turn
// TODO: Make it turn smoothly using the rotation sensors
// TODO: Make it full throttle in the beginning so it starts properly
void turnRight()
{
  LRRotations = 0;
  RRRotations = 0;
  while (LRRotations < 50 && RRRotations < 60)
  {
    Serial.print("Left: ");
    Serial.print(LRRotations);
    Serial.print("  Right: ");
    Serial.println(RRRotations);
    digitalWrite(MOTOR_RB, MOTOR_STOP);
    digitalWrite(MOTOR_LB, MOTOR_STOP);
    analogWrite(MOTOR_RF, MOTOR_R_HALF_SPEED);
    analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
  }
  stopDriving();
}

// Sets the gripper position to the given pulse
void setGripper(int pulse) 
{
  for (int i = 0; i < 8; i++) 
  {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GRIPPER_PIN, LOW);
  }
}

// Reads the sonar and returns the distance in centimeters
void readSonar()
{
  static long startTime = 0;
  static long duration = 0;
  static bool trigStarted = false;
  static bool echoStarted = false;
  static bool echoEnded = false;

   // Clears the trig pin
  if (!trigStarted) 
  {
    digitalWrite(SONAR_TRIG_PIN, LOW);
    startTime = micros();
    trigStarted = true;
  }

  // Turn trig pin on after 2 microseconds 
  if (trigStarted && (micros() - startTime >= 2))
   {
    digitalWrite(SONAR_TRIG_PIN, HIGH);
    trigStarted = false;
    echoStarted = true;
    startTime = micros(); 
  }

  // Turn trig pin off after 10 microseconds
  if (echoStarted && (micros() - startTime >= 10)) 
  {
    digitalWrite(SONAR_TRIG_PIN, LOW);
    echoStarted = false;
  }

  // Reads the echoPin, returns the sound wave travel time in microseconds
  if (!echoEnded) {
    duration = pulseIn(SONAR_ECHO_PIN, HIGH);
    if (duration > 0) 
    {
      echoEnded = true;
      distance = duration * 0.034 / 2; // Calculate distance in cm
      Serial.println(distance);
    }
  }

  // Reset the variables
  if (echoEnded) 
  {
    trigStarted = false;
    echoStarted = false;
    echoEnded = false;
  }
}

void readLineSensor() 
{
  for (int i = 0; i < 8; i++) 
  {
    lineSensorValue[i] = analogRead(LINE_SENSOR[i]);
  }
}

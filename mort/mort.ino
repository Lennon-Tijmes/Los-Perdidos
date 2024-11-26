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
#define SONAR_TRIG_PIN_FORWARD     12    // Sonar forward trig pin
#define SONAR_ECHO_PIN_FORWARD     13    // Sonar forward echo pin 
#define SONAR_TRIG_PIN_RIGHT       5     // Sonar forward trig pin
#define SONAR_ECHO_PIN_RIGHT       6     // Sonar forward echo pin 
int distanceForwards = 999;              // Value for distance from distance sensor, set to 999 for initialization
int distanceRight = 999;                 // Value for distance from distance sensor, set to 999 for initialization

// Section for the line sensor
const int LINE_SENSOR[] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Array for line sensor pins
int lineSensorValue[8] = {0};                               // Array for line sensor values

// Section for the Rotation values
int LRRotations = 0;                // Amount of rotation sensor changes on the left wheel
int RRRotations = 0;                // Amount of rotation sensor changes on the left wheel
const unsigned long debounce = 10;  // Debounce time for more accurate rotation sensor reading

// Code to run once
void setup() 
{
  Serial.begin(9600);                                                 // Begin the serial monitor
  pinMode(MOTOR_LF, OUTPUT);                                          // Initialize the left motor forwards as output
  pinMode(MOTOR_RF, OUTPUT);                                          // Initialize the right motor forwards as output
  pinMode(MOTOR_LB, OUTPUT);                                          // Initialize the left motor backwards as output
  pinMode(MOTOR_RB, OUTPUT);                                          // Initialize the right motor backwards as output
  pinMode(MOTOR_LR, INPUT_PULLUP);                                    // Initialize the rotation sensor of the left wheel as a pullup input
  pinMode(MOTOR_RR, INPUT_PULLUP);                                    // Initialize the rotation sensor of the right wheel as a pullup input
  pinMode(GRIPPER_PIN, OUTPUT);                                       // Initialize the gripper pin as output
  pinMode(SONAR_TRIG_PIN_FORWARD, OUTPUT);                            // Initialize the sonar trig pin as output
  pinMode(SONAR_ECHO_PIN_FORWARD, INPUT);                             // Initialize the sonar echo pin as input
  pinMode(SONAR_TRIG_PIN_RIGHT, OUTPUT);                              // Initialize the sonar trig pin as output
  pinMode(SONAR_ECHO_PIN_RIGHT, INPUT);                               // Initialize the sonar echo pin as input
  for (int i = 0; i < 7; i++) 
  {
    pinMode(LINE_SENSOR[i], INPUT);                                   // Initialize the line sensor pins as input
  }
  setGripper(GRIPPER_OPEN);                                           // Set the gripper open at the start
  attachInterrupt(digitalPinToInterrupt(MOTOR_LR), rotateLR, CHANGE); // Interrupt activates when left wheel rotation sensor changes
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR), rotateRR, CHANGE); // Interrupt activates when right wheel rotation sensor changes
  readSonarForward();                                                 // Set the distance accuratly
  readSonarRight();                                                   // Set the distance accuratly
}

// Code to keep repeating
void loop() 
{
  followRightWall();
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
    digitalWrite(MOTOR_LB, MOTOR_STOP);
    digitalWrite(MOTOR_RB, 1);
    analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
    analogWrite(MOTOR_RF, MOTOR_STOP); 
    Serial.println("panicRight");
  }
  Serial.println("goodRight");
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
    digitalWrite(MOTOR_RB, MOTOR_STOP);
    digitalWrite(MOTOR_LB, 1);
    analogWrite(MOTOR_RF, MOTOR_L_FULL_SPEED);
    analogWrite(MOTOR_LF, MOTOR_STOP); 
    Serial.println("panicLeft");
  }
  Serial.println("GoodLeft");
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

// Reads the sonar forward and returns the distance in centimeters
void readSonarForward()
{
  static long startTime = 0;
  static long duration = 0;
  static bool trigStarted = false;
  static bool echoStarted = false;
  static bool echoEnded = false;

   // Clears the trig pin
  if (!trigStarted) 
  {
    digitalWrite(SONAR_TRIG_PIN_FORWARD, LOW);
    startTime = micros();
    trigStarted = true;
  }

  // Turn trig pin on after 2 microseconds 
  if (trigStarted && (micros() - startTime >= 2))
   {
    digitalWrite(SONAR_TRIG_PIN_FORWARD, HIGH);
    trigStarted = false;
    echoStarted = true;
    startTime = micros(); 
  }

  // Turn trig pin off after 10 microseconds
  if (echoStarted && (micros() - startTime >= 10)) 
  {
    digitalWrite(SONAR_TRIG_PIN_FORWARD, LOW);
    echoStarted = false;
  }

  // Reads the echoPin, returns the sound wave travel time in microseconds
  if (!echoEnded) {
    duration = pulseIn(SONAR_ECHO_PIN_FORWARD, HIGH);
    if (duration > 0) 
    {
      echoEnded = true;
      distanceForwards = duration * 0.034 / 2; // Calculate distance in cm
      Serial.print("distance forward: ");
      Serial.println(distanceForwards);
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

// Reads the sonar right and returns the distance in centimeters
void readSonarRight()
{
  static long startTime = 0;
  static long duration = 0;
  static bool trigStarted = false;
  static bool echoStarted = false;
  static bool echoEnded = false;

   // Clears the trig pin
  if (!trigStarted) 
  {
    digitalWrite(SONAR_TRIG_PIN_RIGHT, LOW);
    startTime = micros();
    trigStarted = true;
  }

  // Turn trig pin on after 2 microseconds 
  if (trigStarted && (micros() - startTime >= 2))
   {
    digitalWrite(SONAR_TRIG_PIN_RIGHT, HIGH);
    trigStarted = false;
    echoStarted = true;
    startTime = micros(); 
  }

  // Turn trig pin off after 10 microseconds
  if (echoStarted && (micros() - startTime >= 10)) 
  {
    digitalWrite(SONAR_TRIG_PIN_RIGHT, LOW);
    echoStarted = false;
  }

  // Reads the echoPin, returns the sound wave travel time in microseconds
  if (!echoEnded) {
    duration = pulseIn(SONAR_ECHO_PIN_RIGHT, HIGH);
    if (duration > 0) 
    {
      echoEnded = true;
      distanceRight = duration * 0.034 / 2; // Calculate distance in cm
      Serial.print("distance right: ");
      Serial.println(distanceRight);
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

// Read all the line sensor pins
void readLineSensor() 
{
  for (int i = 0; i < 8; i++) 
  {
    lineSensorValue[i] = analogRead(LINE_SENSOR[i]);
  }
}

void followRightWall()
{
  readSonarForward();
  readSonarRight();
  if (distanceRight > 15)
  {
    stopDriving();
    goForwards(200);
    delay(500);
    stopDriving();
    rotateRight();
    stopDriving();
    goForwards(200);
    delay(500);
    stopDriving();
  }
  else if (distanceForwards < 15)
  {
    stopDriving();
    rotateLeft();
    stopDriving();
  }
  else
  {
    goForwards(200);
  }
}

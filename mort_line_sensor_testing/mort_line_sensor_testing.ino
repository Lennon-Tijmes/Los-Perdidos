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

// Section for the line sensor
const int LINE_SENSOR[] = {A1, A2, A3, A4, A5, A6}; // Array for line sensor pins
int lineSensorValue[6] = {0};                               // Array for line sensor values
#define BLACK_LINE        800 // temporary value for the signoff

// Section for the Rotation values
int LRRotations = 0;                // Amount of rotation sensor changes on the left wheel
int RRRotations = 0;                // Amount of rotation sensor changes on the left wheel
const unsigned long debounce = 10;  // Debounce time for more accurate rotation sensor reading

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);               // Begin the serial monitor
  pinMode(MOTOR_LF, OUTPUT);        // Initialize the left motor forwards as output
  pinMode(MOTOR_RF, OUTPUT);        // Initialize the right motor forwards as output
  pinMode(MOTOR_LB, OUTPUT);        // Initialize the left motor backwards as output
  pinMode(MOTOR_RB, OUTPUT);        // Initialize the right motor backwards as output
  pinMode(MOTOR_LR, INPUT_PULLUP);  // Initialize the rotation sensor of the left wheel as a pullup input
  pinMode(MOTOR_RR, INPUT_PULLUP);  // Initialize the rotation sensor of the right wheel as a pullup input
  for (int i = 0; i < 5; i++) 
  {
    pinMode(LINE_SENSOR[i], INPUT); // Initialize the line sensor pins as input
  }
  attachInterrupt(digitalPinToInterrupt(MOTOR_LR), rotateLR, CHANGE); // Interrupt activates when left wheel rotation sensor changes
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR), rotateRR, CHANGE); // Interrupt activates when right wheel rotation sensor changes
}

void loop() {
  // put your main code here, to run repeatedly:
  followLine();
  unsigned long testing = millis();
  static unsigned long print = 0;
  if (testing - print >= 1000)
  {
    print = testing;
    int sensorValue = analogRead(A6); // Read the line sensor value from A0
    Serial.print("Sensor Value: "); 
    Serial.println(sensorValue); // Print the value to the Serial Monitor
  }
}

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
  while (RRRotations < 11)
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

// Read all the line sensor pins
void readLineSensor() 
{
  for (int i = 0; i < 6; i++) 
  {
    lineSensorValue[i] = analogRead(LINE_SENSOR[i]);
  }
}

void followLine()
{
  if (analogRead(LINE_SENSOR[2]) > BLACK_LINE || analogRead(LINE_SENSOR[3]) > BLACK_LINE)
  {
    goForwards(255);
  }
  else if (analogRead(LINE_SENSOR[5]) > BLACK_LINE || analogRead(LINE_SENSOR[4]) > BLACK_LINE || analogRead(LINE_SENSOR[5]) > BLACK_LINE && analogRead(LINE_SENSOR[0]) < BLACK_LINE)
  {
    lineTurnLeft();
  }
  else if (analogRead(LINE_SENSOR[4]) > BLACK_LINE || analogRead(LINE_SENSOR[5]))
  {
    lineTurnLeft();
  }
  else if (analogRead(LINE_SENSOR[0]) > BLACK_LINE || analogRead(LINE_SENSOR[1]) > BLACK_LINE || analogRead(LINE_SENSOR[0]) > BLACK_LINE && analogRead(LINE_SENSOR[5]) < BLACK_LINE)
  {
    lineTurnRight();
  }
  else if (analogRead(LINE_SENSOR[6]) < BLACK_LINE)
  {
    lineTurnRight();
  }
}

void lineTurnLeft()
{
  digitalWrite(MOTOR_RB, 1);
  digitalWrite(MOTOR_LB, MOTOR_STOP);
  analogWrite(MOTOR_RF, 0);
  analogWrite(MOTOR_LF, 255);
}

void lineTurnRight()
{
  digitalWrite(MOTOR_RB, MOTOR_STOP);
  digitalWrite(MOTOR_LB, 1);
  analogWrite(MOTOR_RF, 255);
  analogWrite(MOTOR_LF, 0);
}

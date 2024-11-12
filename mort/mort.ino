// Section for the motors
#define MOTOR_LF           9    // Forwards left motor
#define MOTOR_RF           10   // Forwards right motor
#define MOTOR_LB           5    // Backwards left motor
#define MOTOR_RB           6    // Backwards right motor
#define MOTOR_L_FULL_SPEED 250  // Left motor full speed
#define MOTOR_R_FULL_SPEED 255  // Right motor full speed
#define MOTOR_STOP         0    // Motor stopping speed

// TODO: Figure out which one is actually left and right
#define MOTOR_LR           2    // Left rotation sensor
#define MOTOR_RR           3    // Right rotation sensor

// Section for the servo motor
// TODO: get a servo pin
#define GRIPPER_PIN        4    // Pin for the servo gripper
#define GRIPPER_OPEN       1600 // Value for gripper being open
#define GRIPPER_CLOSED     950  // Value for gripper being closed

// Section for the Rotation values
int LRRotations = 0;
int RRRotations = 0;
unsigned long LRotationTime = 0;
unsigned long RRotationTime = 0;
const unsigned long debounce = 10;

// Code to run once
void setup() 
{
  Serial.begin(9600);
  pinMode(GRIPPER_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(MOTOR_LR), rotateLR, CHANGE); //interrupt activates when rotation sensor changes
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR), rotateRR, CHANGE); //interrupt activates when rotation sensor changes
}

// Code to keep repeating
void loop() 
{
  goForwards();
  delay(500);
  turnLeft();
  delay(500);
}

// Counts the interrupts of the rotation sensor for the left wheel
void rotateLR()
{
  unsigned long time = millis();
  if (time - LRotationTime >= debounce)
  {
    LRRotations++;
    LRotationTime = time;
  }
}

// Counts the interrupts of the rotation sensor for the right wheel
void rotateRR() 
{
  unsigned long time = millis();
  if (time - RRotationTime >= debounce)
  {
    RRRotations++;
    RRotationTime = time;
  }
}

// Makes the relaybot drive in a straight line forward
// TODO: Make the robot drive a certain speed, calibrated with rotation sensor
void goForwards()
{
  analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
  analogWrite(MOTOR_RF, MOTOR_R_FULL_SPEED);
  analogWrite(MOTOR_LB, MOTOR_STOP);
  analogWrite(MOTOR_RB, MOTOR_STOP);
}

// Makes the relaybot drive in a straight line backwards
// TODO: Make the robot drive a certain speed, calibrated with rotation sensor
void goBackwards()
{
  analogWrite(MOTOR_LB, MOTOR_L_FULL_SPEED);
  analogWrite(MOTOR_RB, MOTOR_R_FULL_SPEED);
  analogWrite(MOTOR_LF, MOTOR_STOP);
  analogWrite(MOTOR_RF, MOTOR_STOP);
}

// Stops all the motors
void stopDriving()
{
  analogWrite(MOTOR_LB, MOTOR_STOP);
  analogWrite(MOTOR_RB, MOTOR_STOP);
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
  while (RRRotations < 190)
  {
    Serial.print("Right:"); 
    Serial.println(RRRotations);
    analogWrite(MOTOR_LB, MOTOR_STOP);
    analogWrite(MOTOR_RB, MOTOR_R_FULL_SPEED);
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
  while (LRRotations < 190)
  {
    Serial.print("Left:");
    Serial.println(LRRotations); 
    analogWrite(MOTOR_RB, MOTOR_STOP);
    analogWrite(MOTOR_LB, MOTOR_R_FULL_SPEED);
    analogWrite(MOTOR_RF, MOTOR_L_FULL_SPEED);
    analogWrite(MOTOR_LF, MOTOR_STOP); 
  }
  stopDriving();
}

// Function for making a left turn
// TODO: Make this function
void turnLeft()
{

}

// Function for making a right turn
// TODO: Make this function
void turnRight()
{
  
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

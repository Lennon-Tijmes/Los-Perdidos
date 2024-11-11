// motors
#define MOTOR_L_FULL_SPEED 250  // Left motor full speed
#define MOTOR_R_FULL_SPEED 255  // Right motor full speed
#define MOTOR_STOP         0    // Motor stopping speed
#define MOTOR_LF           9    // Forwards left motor
#define MOTOR_RF           10   // Forwards right motor
#define MOTOR_LB           5    // Backwards left motor
#define MOTOR_RB           6    // Backwards right motor
#define MOTOR_LR           2    // Left rotation sensor
#define MOTOR_RR           3    // Right rotation sensor

int LRRotations = 0;
int RRRotations = 0;

void setup() 
{
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(MOTOR_LR), rotateLR, CHANGE); //interrupt activates when rotation sensor changes
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR), rotateRR, CHANGE); //interrupt activates when rotation sensor changes
}

void loop() 
{

}

void rotateLR() 
{
  LRRotations++;
}

void rotateRR() 
{
  RRRotations++;
}

void goForwards()
{
  analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
  analogWrite(MOTOR_RF, MOTOR_R_FULL_SPEED);
  analogWrite(MOTOR_LB, MOTOR_STOP);
  analogWrite(MOTOR_RB, MOTOR_STOP);
}

void goBackwards()
{
  analogWrite(MOTOR_LB, MOTOR_L_FULL_SPEED);
  analogWrite(MOTOR_RB, MOTOR_R_FULL_SPEED);
  analogWrite(MOTOR_LF, MOTOR_STOP);
  analogWrite(MOTOR_RF, MOTOR_STOP);
}

void stopDriving()
{
  analogWrite(MOTOR_LB, MOTOR_STOP);
  analogWrite(MOTOR_RB, MOTOR_STOP);
  analogWrite(MOTOR_LF, MOTOR_STOP);
  analogWrite(MOTOR_RF, MOTOR_STOP); 
}

// TODO: Make it turn correctly
void turnRight()
{
  stopDriving();
  delay(500);
  LRRotations = 0;
  while (LRRotations < 190)
  {
    Serial.println(LRRotations);
    analogWrite(MOTOR_LB, MOTOR_STOP);
    analogWrite(MOTOR_RB, MOTOR_R_FULL_SPEED);
    analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
    analogWrite(MOTOR_RF, MOTOR_STOP); 
  }
  stopDriving();
}

// TODO: Make it turn correctly
void turnLeft()
{
  stopDriving();
  delay(500);
  RRRotations = 0;
  while (RRRotations < 190)
  {
    Serial.println(LRRotations);
    analogWrite(MOTOR_RB, MOTOR_STOP);
    analogWrite(MOTOR_LB, MOTOR_R_FULL_SPEED);
    analogWrite(MOTOR_RF, MOTOR_L_FULL_SPEED);
    analogWrite(MOTOR_LF, MOTOR_STOP); 
  }
  stopDriving();
}

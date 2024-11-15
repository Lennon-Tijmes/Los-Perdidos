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
  attachInterrupt(digitalPinToInterrupt(MOTOR_LR), rotateLR, CHANGE); //interrupt activates when rotation sensor changes
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR), rotateRR, CHANGE); //interrupt activates when rotation sensor changes
}

// Code to keep repeating
void loop() 
{
  goForwards(255);
  delay(600);
  stopDriving();
  delay(1000);
}

// Counts the interrupts of the rotation sensor for the left wheel
void rotateLR()
{
  int left_reading;
  unsigned long time = millis();
  if (time - LRotationTime >= debounce)
  {
    LRRotations++;
    LRotationTime = time;
    left_reading = digitalRead(MOTOR_LR);
    Serial.print("Analog= ");
    Serial.println(left_reading);
  }
}

// Counts the interrupts of the rotation sensor for the right wheel
void rotateRR() 
{
  int Right_A0;
  int Right_D0;
  unsigned long time = millis();
  if (time - RRotationTime >= debounce)
  {
    RRRotations++;
    RRotationTime = time;
    Right_D0 = digitalRead(MOTOR_RR);
    Serial.print("Digital=");
    Serial.println(Right_D0);
  }
}

// Makes the relaybot drive in a straight line forward
// TODO: Make the robot drive a certain speed, calibrated with rotation sensor
void goForwards(int speed)
{
  LRRotations = 0;
  RRRotations = 0;
  LRotationTime = millis();
  RRotationTime = millis();

  analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
  analogWrite(MOTOR_RF, MOTOR_R_FULL_SPEED);
  digitalWrite(MOTOR_LB, MOTOR_STOP);
  digitalWrite(MOTOR_RB, MOTOR_STOP);

   // unsigned long testing = millis() + 600; // Testing for 5 sec, 10 sec and it drives off the table
    //while (millis() < testing) 
    //{
      //Serial.print("Left: ");
      //Serial.print(LRRotations);
      //Serial.print("  Right: ");
      //Serial.println(RRRotations);
      //delay(300); // Prints every 500 ms
    //}
  //stopDriving();
}

// Makes the relaybot drive in a straight line backwards
// TODO: Make the robot drive a certain speed, calibrated with rotation sensor
void goBackwards(int speed)
{
  digitalWrite(MOTOR_LB, 1);
  digitalWrite(MOTOR_RB, 1);
  analogWrite(MOTOR_LF, (MOTOR_L_FULL_SPEED - speed));
  analogWrite(MOTOR_RF, (MOTOR_R_FULL_SPEED - speed));
}

// Stops all the motors
void stopDriving()
{
  Serial.print("Left: ");
  Serial.print(LRRotations);
  Serial.print("  Right: ");
  Serial.println(RRRotations);
  analogWrite(MOTOR_LB, MOTOR_STOP);
  analogWrite(MOTOR_RB, MOTOR_STOP);
  analogWrite(MOTOR_LF, MOTOR_STOP);
  analogWrite(MOTOR_RF, MOTOR_STOP); 
}




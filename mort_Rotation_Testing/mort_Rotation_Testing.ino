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
  attachInterrupt(digitalPinToInterrupt(MOTOR_LR), rotateLR, CHANGE); //interrupt activates when rotation sensor changes
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR), rotateRR, CHANGE); //interrupt activates when rotation sensor changes
}

// Code to keep repeating
void loop() 
{
  delay(200);
  goForwards(200);
  stopDriving();
  delay(2000);
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

void pseudoForward(int speed)
{
  LRRotations = 0;
  RRRotations = 0;

  while (LRRotations <= 200 && RRRotations <= 200)
  {
  
    int difference = LRRotations - RRRotations;
    
    int leftSpeed = speed - (difference * 20);
    int rightSpeed = speed;

    analogWrite(MOTOR_LF, leftSpeed);
    analogWrite(MOTOR_RF, rightSpeed);
    digitalWrite(MOTOR_LB, MOTOR_STOP);
    digitalWrite(MOTOR_RB, MOTOR_STOP);

    Serial.print("Left: ");
    Serial.print(LRRotations);
    Serial.print("  Right: ");
    Serial.println(RRRotations);
  }

  float distancePerPulse = 0.51; // cm/pulse
  float totalPulses = 180;       // Total pulses (assume both wheels are equal)
  float totalDistance = totalPulses * distancePerPulse; // Total distance in cm

  Serial.print("Distance: ");
  Serial.println(totalDistance);
}



// Makes the relaybot drive in a straight line forward
// TODO: Make the robot drive a certain speed, calibrated with rotation sensor
void goForwards(int speed)
{
  unsigned long timerz = millis();
  LRRotations = 0;
  RRRotations = 0;

  while (LRRotations <= 60 && RRRotations <= 60)
  {
    unsigned long currentTime = millis();

    if (currentTime - timerz >= 200)
    {
      timerz = currentTime;

      int difference = RRRotations - LRRotations;
    
      int leftSpeed = speed - difference;
      int rightSpeed = speed;

      leftSpeed = constrain(leftSpeed, 150, 255);
      rightSpeed = constrain(rightSpeed, 150, 255);

      analogWrite(MOTOR_LF, speed - difference);
      analogWrite(MOTOR_RF, speed);
      digitalWrite(MOTOR_LB, MOTOR_STOP);
      digitalWrite(MOTOR_RB, MOTOR_STOP);

      Serial.print("Left: ");
      Serial.print(LRRotations);
      Serial.print("  Right: ");
      Serial.println(RRRotations);
    }
  }


  float distancePerPulse = 0.51; // cm/pulse
  float totalPulses = 60;       // Total pulses (assume both wheels are equal)
  float totalDistance = totalPulses * distancePerPulse; // Total distance in cm

  Serial.print("Distance: ");
  Serial.println(totalDistance);
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
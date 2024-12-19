// #define DEBUG 1



///////////
// SETUP //
///////////

#include <Adafruit_NeoPixel.h> //neopixel library
#define NUM_PIXELS 4 //number of neopixels
#define PIXEL_PIN 11 //pin for the neopixels
Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_RGB + NEO_KHZ800);

const unsigned char LINE_SENSOR[] = {A2, A3, A4, A5, A6, A7}; // Line sensor pins
#define SONAR_TRIG_PIN_FORWARD 12  // Sonar forward trig pin
#define SONAR_ECHO_PIN_FORWARD 13  // Sonar forward echo pin
#define SONAR_TRIG_PIN_LEFT   5   // Sonar right trig pin
#define SONAR_ECHO_PIN_LEFT   6   // Sonar right echo pin
#define SONAR_TRIG_PIN_RIGHT   A1  // Sonar left trig pin
#define SONAR_ECHO_PIN_RIGHT   A0  // Sonar left echo pin
#define MOTOR_LF               9   // Forwards left motor pin
#define MOTOR_RF               10  // Forwards right motor pin
#define MOTOR_LB               7   // Backwards left motor pin
#define MOTOR_RB               8   // Backwards right motor pin
#define MOTOR_LR               2   // Left rotation sensor pin
#define MOTOR_RR               3   // Right rotation sensor pin
#define GRIPPER_PIN            4   // Servo gripper pin

unsigned int distanceForwards = 0;
unsigned int distanceRight = 0;
unsigned int distanceLeft = 0;
unsigned int leftSpeed = 0;
unsigned int rightSpeed = 0;

unsigned int LRRotations = 0;       // Amount of rotation sensor changes on the left wheel
unsigned int RRRotations = 0;       // Amount of rotation sensor changes on the left wheel

#define NO_TASK    0
#define FORWARD    1
#define RIGHT_90   2
#define LEFT_90    3
#define RIGHT_180  4
#define LEFT_180   5
#define ROTATE_180 6
#define FORWARD_BEFORE_RIGHT 7
unsigned char currentTask = NO_TASK;
unsigned long currentTaskStart = 0;
unsigned long currentTaskDuration = 0;
unsigned long currentTime = micros();

bool allBlack = false;
bool allWhite = false;
unsigned int colorBlack = 850;
unsigned int colorWhite = 700;

bool mazeEntered = false;
bool squarePassed = false;
bool needToFindFinish = false;
bool finishFound = false;

void setup() 
{
  Serial.begin(9600);                       // Begin the serial monitor
  pixels.begin();
  pinMode(MOTOR_LF, OUTPUT);                // Initialize the left motor forwards as output
  pinMode(MOTOR_RF, OUTPUT);                // Initialize the right motor forwards as output
  pinMode(MOTOR_LB, OUTPUT);                // Initialize the left motor backwards as output
  pinMode(MOTOR_RB, OUTPUT);                // Initialize the right motor backwards as output
  pinMode(MOTOR_LR, INPUT_PULLUP);          // Initialize the rotation sensor of the left wheel as a pullup input
  pinMode(MOTOR_RR, INPUT_PULLUP);          // Initialize the rotation sensor of the right wheel as a pullup input
  pinMode(GRIPPER_PIN, OUTPUT);             // Initialize the gripper pin as output
  pinMode(SONAR_TRIG_PIN_LEFT, OUTPUT);     // Initialize the sonar trig pin as output
  pinMode(SONAR_ECHO_PIN_LEFT, INPUT);      // Initialize the sonar echo pin as input
  pinMode(SONAR_TRIG_PIN_FORWARD, OUTPUT);  // Initialize the sonar trig pin as output
  pinMode(SONAR_ECHO_PIN_FORWARD, INPUT);   // Initialize the sonar echo pin as input
  pinMode(SONAR_TRIG_PIN_RIGHT, OUTPUT);    // Initialize the sonar trig pin as output
  pinMode(SONAR_ECHO_PIN_RIGHT, INPUT);     // Initialize the sonar echo pin as input

  for (char i=0; i < 6; ++i) 
  {
    pinMode(LINE_SENSOR[i], INPUT);         // Initialize the line sensor pins as input
  }

  attachInterrupt(digitalPinToInterrupt(MOTOR_LR), countRotationsLeft, CHANGE);  // Interrupt activates when left wheel rotation sensor changes
  attachInterrupt(digitalPinToInterrupt(MOTOR_RR), countRotationsRight, CHANGE); // Interrupt activates when right wheel rotation sensor changes
}



#define WAIT_FOR_START       0
#define EXPLORE_AND_MAP_MAZE 1
#define RETURN_TO_START      2
#define WAIT_FOR_CARGO       3
#define DRIVE_MAZE           4
#define PARK_CARGO           5
#define FINISH               6

unsigned char programPhase = WAIT_FOR_START;
unsigned long startTime = currentTime;
bool waitForStart = true;

void loop() 
{
  currentTime = micros();

  // // updateSonar();
  readLineSensor();
  hazardLights();

  if (waitForStart)
  {
    if (currentTime - startTime > 2000000)
    {
      waitForStart = false;
    }
    return;
  }

  // followRightWall();
  // driveToSquare();
  // followLineStart();
  // findFinish();
  // followLineEnd();

  #ifdef DEBUG
    printDebugMessage();
  #endif
}

///////////
// DEBUG //
///////////
#ifdef DEBUG
  unsigned int maxDistanceLeft = 0;
  unsigned int maxDistanceForwards = 0;
  unsigned int maxDistanceRight = 0;
  unsigned long lastDebug = currentTime;
  unsigned long debugStart = currentTime;

  String translateCurrentTask(unsigned char taskId) 
  {
    switch (taskId) 
    {
      case 0: return "NO_TASK   ";
      case 1: return "FORWARD   ";
      case 2: return "RIGHT_90  ";
      case 3: return "LEFT_90   ";
      case 4: return "RIGHT_180 ";
      case 5: return "LEFT_180  ";
      case 6: return "ROTATE_180";
    }
    return "INVALID";
  }

  void printDebugMessage()
  {
    if (currentTime - lastDebug > 1000613)
    {
      debugStart = millis();

      String debugMessage = "\n---- ---- DIAGNOSTIC DEBUG DATA ---- ----";
      debugMessage += "\nTime (us):             " + String(currentTime);
      debugMessage += "\nSonar left (cm):       " + int2String4(distanceLeft)    +" (max " + String(maxDistanceLeft)    +")";
      debugMessage += "\nSonar forwards (cm):   " + int2String4(distanceForwards)+" (max " + String(maxDistanceForwards)+")";
      debugMessage += "\nSonar right (cm):      " + int2String4(distanceRight)   +" (max " + String(maxDistanceRight)   +")";
      debugMessage += "\nWheel rotations right: " + String(RRRotations);
      debugMessage += "\nWheel rotations left:  " + String(LRRotations);
      debugMessage += "\nTask:                  " + translateCurrentTask(currentTask);
      debugMessage += "\nTask time left (us):   " + String(currentTaskStart + currentTaskDuration - currentTime);
      debugMessage += "\nLeft motor speed:      " + int2String4(leftSpeed);
      debugMessage += "\nRight motor speed:     " + int2String4(rightSpeed);
      

      Serial.println(debugMessage);
      Serial.println("Debug duration (ms):   " + String(millis() - debugStart));

      maxDistanceLeft = 0;
      maxDistanceForwards = 0;
      maxDistanceRight = 0;
      lastDebug = currentTime;
    }
  }

  String int2String4(unsigned int number)
  {
    String result = "";

    if (number < 10)
      result = "   ";
    else if (number < 100)
      result = "  ";
    else if (number < 1000)
      result = " ";

    return String(number) + result;
  }
#endif



/////////////////
// LINE SENSOR //
/////////////////

int lineSensorValue[6] = {0, 0, 0, 0, 0, 0}; // Array for line sensor values

// Read all the line sensor pins
void readLineSensor() 
{
  for (unsigned char i = 0; i < 6; i++) 
  {
    lineSensorValue[i] = analogRead(LINE_SENSOR[i]);
  }

  allBlack = (lineSensorValue[0] >= colorBlack) 
           && (lineSensorValue[1] >= colorBlack) 
           && (lineSensorValue[2] >= colorBlack) 
           && (lineSensorValue[3] >= colorBlack) 
           && (lineSensorValue[4] >= colorBlack) 
           && (lineSensorValue[5] >= colorBlack);  //true if all the line sensor bits are looking at black

  allWhite = (lineSensorValue[0] <= colorWhite) 
           && (lineSensorValue[1] <= colorWhite) 
           && (lineSensorValue[2] <= colorWhite) 
           && (lineSensorValue[3] <= colorWhite) 
           && (lineSensorValue[4] <= colorWhite) 
           && (lineSensorValue[5] <= colorWhite);  //true if all the line sensor bits are looking at white

}



///////////
// SONAR //
///////////

// Sonar phases
#define SONAR_LEFT_START_SIGNAL    1
#define SONAR_LEFT_STOP_SIGNAL     2
#define SONAR_LEFT_READ_SIGNAL     3
#define SONAR_FORWARD_START_SIGNAL 4
#define SONAR_FORWARD_STOP_SIGNAL  5
#define SONAR_FORWARD_READ_SIGNAL  6
#define SONAR_RIGHT_START_SIGNAL   7
#define SONAR_RIGHT_STOP_SIGNAL    8
#define SONAR_RIGHT_READ_SIGNAL    9

// The HC-SR04 manual recommends sending a 10 microsecond pulse to TRIG pin
#define SONAR_SIGNAL_DURATION_US   10

// Delay before switching to next sonar
#define SONAR_DELAY_US             10000

// We assume the maze is 7x7, which means the longest distance to measure should be 7*30cm = 210cm
// Speed of sound is around 35cm/ms=210cm/6ms, so the sound signal should take at most 12ms to come back to the robot
// Accounting for some inaccuracies it's safe to set the timeout to 15ms
#define SONAR_RECEIVER_TIMEOUT_US  15000

void updateSonar() 
{
  static unsigned char sonarPhase = SONAR_LEFT_START_SIGNAL;
  static unsigned long lastActionTime = micros();
  static unsigned long duration = 0;

  switch (sonarPhase)
   {

    // Left sonar

    // Start pulse on left sonar
    case SONAR_LEFT_START_SIGNAL:
      if (currentTime - lastActionTime > SONAR_DELAY_US)
      {
        digitalWrite(SONAR_TRIG_PIN_LEFT, HIGH);
        lastActionTime = currentTime;
        sonarPhase = SONAR_LEFT_STOP_SIGNAL;
      }
      break;

    // After SONAR_SIGNAL_DURATION_US microseconds stop pulse on left sonar
    case SONAR_LEFT_STOP_SIGNAL:
      if (currentTime - lastActionTime > SONAR_SIGNAL_DURATION_US) 
      {
        digitalWrite(SONAR_TRIG_PIN_LEFT, LOW);
        lastActionTime = currentTime;
        sonarPhase = SONAR_LEFT_READ_SIGNAL;
      }
      break;

    // Get HIGH state duration from ECHO pin, or move to next phase if no signal after SONAR_RECEIVER_TIMEOUT_US microseconds
    case SONAR_LEFT_READ_SIGNAL:
      duration = pulseIn(SONAR_ECHO_PIN_LEFT, HIGH);
      if (duration > 0) 
      {
        distanceLeft = duration * 0.034 / 2;
        #ifdef DEBUG
        if (distanceLeft > maxDistanceLeft) maxDistanceLeft = distanceLeft;
        #endif
        duration = 0;
        lastActionTime = currentTime;
        sonarPhase = SONAR_FORWARD_START_SIGNAL;
      }
      else if (currentTime - lastActionTime > SONAR_RECEIVER_TIMEOUT_US) 
      {
        lastActionTime = currentTime;
        sonarPhase = SONAR_FORWARD_START_SIGNAL;
      }
      break;

    // Forward sonar

    // Start pulse on forward sonar
    case SONAR_FORWARD_START_SIGNAL:
      if (currentTime - lastActionTime > SONAR_DELAY_US)
      {
        digitalWrite(SONAR_TRIG_PIN_FORWARD, HIGH);
        lastActionTime = currentTime;
        sonarPhase = SONAR_FORWARD_STOP_SIGNAL;
      }
      break;

    // After SONAR_SIGNAL_DURATION_US microseconds stop pulse on forward sonar
    case SONAR_FORWARD_STOP_SIGNAL:
      if (currentTime - lastActionTime > SONAR_SIGNAL_DURATION_US) 
      {
        digitalWrite(SONAR_TRIG_PIN_FORWARD, LOW);
        lastActionTime = currentTime;
        sonarPhase = SONAR_FORWARD_READ_SIGNAL;
      }
      break;

    // Get HIGH state duration from ECHO pin, or move to next phase if no signal after SONAR_RECEIVER_TIMEOUT_US microseconds 
    case SONAR_FORWARD_READ_SIGNAL:
      duration = pulseIn(SONAR_ECHO_PIN_FORWARD, HIGH);
      if (duration > 0)
       {
        distanceForwards = duration * 0.034 / 2;
        #ifdef DEBUG
        if (distanceForwards > maxDistanceForwards) maxDistanceForwards = distanceForwards;
        #endif
        duration = 0;
        lastActionTime = currentTime;
        sonarPhase = SONAR_RIGHT_START_SIGNAL;
      }
      else if (currentTime - lastActionTime > SONAR_RECEIVER_TIMEOUT_US) 
      {
        lastActionTime = currentTime;
        sonarPhase = SONAR_RIGHT_START_SIGNAL;
      }
      break;

    // Right sonar

    // Start pulse on right sonar
    case SONAR_RIGHT_START_SIGNAL:
      if (currentTime - lastActionTime > SONAR_DELAY_US)
      {
        digitalWrite(SONAR_TRIG_PIN_RIGHT, HIGH);
        lastActionTime = currentTime;
        sonarPhase = SONAR_RIGHT_STOP_SIGNAL;
      }
      break;

    // After SONAR_SIGNAL_DURATION_US microseconds stop pulse on right sonar
    case SONAR_RIGHT_STOP_SIGNAL:
      if (currentTime - lastActionTime > SONAR_SIGNAL_DURATION_US) 
      {
        digitalWrite(SONAR_TRIG_PIN_RIGHT, LOW);
        lastActionTime = currentTime;
        sonarPhase = SONAR_RIGHT_READ_SIGNAL;
      }
      break;

    // Get HIGH state duration from ECHO pin, or move to next phase if no signal after SONAR_RECEIVER_TIMEOUT_US microseconds
    case SONAR_RIGHT_READ_SIGNAL: 
    {
      duration = pulseIn(SONAR_ECHO_PIN_RIGHT, HIGH);
      if (duration > 0) 
      {
        distanceRight = duration * 0.034 / 2;
        #ifdef DEBUG
        if (distanceRight > maxDistanceRight) maxDistanceRight = distanceRight;
        #endif
        duration = 0;
        lastActionTime = currentTime;
        sonarPhase = SONAR_LEFT_START_SIGNAL;
      }
      else if (currentTime - lastActionTime > SONAR_RECEIVER_TIMEOUT_US) 
      {
        lastActionTime = currentTime;
        sonarPhase = SONAR_LEFT_START_SIGNAL;
      }
    } 
    break;
  }
}



////////////////////////////
// WHEEL ROTATION TRACKER //
////////////////////////////

const unsigned long debounce = 10;  // Debounce time for more accurate rotation sensor reading

// Counts the interrupts of the rotation sensor for the left wheel
void countRotationsLeft() 
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
void countRotationsRight() 
{
  static unsigned long timer;
  static bool lastState;
  noInterrupts();

  if (millis() > timer) 
  {
    bool state = digitalRead(MOTOR_RR);

    if(state != lastState) {
      RRRotations++;
      lastState = state;
    }

    timer = millis() + debounce;
  }

  interrupts();
}



///////////
// MOTOR //
///////////

#define SPEED 200
#define MOTOR_STOP         0    // Motor stopping speed

// Makes the relaybot drive in a straight line forward
void goForwards() 
{
  analogWrite(MOTOR_LF, SPEED - 5);
  analogWrite(MOTOR_RF, SPEED);
  digitalWrite(MOTOR_LB, MOTOR_STOP);
  digitalWrite(MOTOR_RB, MOTOR_STOP);
}

void adjustLeft()
{
  analogWrite(MOTOR_LF, SPEED - 40);
  analogWrite(MOTOR_RF, SPEED + 40);
  digitalWrite(MOTOR_LB, MOTOR_STOP);
  digitalWrite(MOTOR_RB, MOTOR_STOP);  
}

void adjustRight()
{
  analogWrite(MOTOR_LF, SPEED + 40);
  analogWrite(MOTOR_RF, SPEED - 35);
  digitalWrite(MOTOR_LB, MOTOR_STOP);
  digitalWrite(MOTOR_RB, MOTOR_STOP);  
}

// Makes the relaybot drive in a straight line backwards
void goBackwards(unsigned int speed) 
{
  digitalWrite(MOTOR_LB, 1);
  digitalWrite(MOTOR_RB, 1);
  analogWrite(MOTOR_LF, (255 - speed));
  analogWrite(MOTOR_RF, (255 - speed));
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
void rotateRight() 
{
  digitalWrite(MOTOR_LB, MOTOR_STOP);
  digitalWrite(MOTOR_RB, 1);
  analogWrite(MOTOR_LF, 255);
  analogWrite(MOTOR_RF, MOTOR_STOP); 
}

// Rotate the relaybot on its axis to the left
void rotateLeft()
{
  digitalWrite(MOTOR_RB, MOTOR_STOP);
  digitalWrite(MOTOR_LB, 1);
  analogWrite(MOTOR_RF, 255);
  analogWrite(MOTOR_LF, MOTOR_STOP); 
}

void turnAround()
{
  digitalWrite(MOTOR_LB, 1);
  digitalWrite(MOTOR_RB, 1);
  analogWrite(MOTOR_RF, 50);
  analogWrite(MOTOR_LF, 150);
  delay(2000);
  stopDriving();
}

void setMotors(int leftSpeed, int rightSpeed) 
{
  if (leftSpeed >= 0) 
  {
    analogWrite (MOTOR_LF, leftSpeed);
    digitalWrite(MOTOR_LB, 0);
  }
  else 
  { // leftSpeed < 0
    analogWrite (MOTOR_LF, 255+leftSpeed);
    digitalWrite(MOTOR_LB, 1);
  }

  if (rightSpeed >= 0) 
  {
    analogWrite (MOTOR_RF, rightSpeed);
    digitalWrite(MOTOR_RB, 0);
  }
  else
  { // rightSpeed < 0
    analogWrite (MOTOR_RF, 255+rightSpeed);
    digitalWrite(MOTOR_RB, 1);
  }
}

void driveStraight()
{
  static unsigned long timer = micros();
  unsigned int difference = 0;
  if (currentTime - timer >= 10000)
  {
      timer = currentTime;

      if (RRRotations < LRRotations)
      {
        difference = LRRotations - RRRotations;
        leftSpeed = SPEED - (difference);
        leftSpeed = constrain(leftSpeed, 160, 255);
        rightSpeed = SPEED;
      }
      else
      {
        difference = RRRotations - LRRotations;
        rightSpeed = SPEED - (difference);
        rightSpeed = constrain(rightSpeed, 160, 255);
        leftSpeed = SPEED;
      }

      analogWrite(MOTOR_LF, leftSpeed);
      analogWrite(MOTOR_RF, rightSpeed);
      digitalWrite(MOTOR_LB, MOTOR_STOP);
      digitalWrite(MOTOR_RB, MOTOR_STOP);
  }
}



/////////////
// GRIPPER //
/////////////

#define GRIPPER_OPEN   1600  // Value for gripper being open
#define GRIPPER_CLOSED 1010  // Value for gripper being closed

// Sets the gripper position to the given pulse
void setGripper(int pulse) 
{
  for (unsigned char i = 0; i < 8; i++)
   {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GRIPPER_PIN, LOW);
  }
}

////////
// AI //
////////

#define MAZE_SIZE 8
unsigned char path[MAZE_SIZE*MAZE_SIZE];  // 512 bytes

void followRightWall() 
{
  switch (currentTask) 
  {
    static unsigned int oldRRR = RRRotations;
    static unsigned int oldLRR = LRRotations;
    case NO_TASK:
      oldRRR = RRRotations;
      oldLRR = LRRotations;
      if (distanceRight > 20)
      {
        RRRotations = 0;
        LRRotations = 0;
        goForwards();
        currentTask = FORWARD_BEFORE_RIGHT;
      }
      else if (distanceForwards < 20)
      {
        RRRotations = 0;
        currentTask = LEFT_90;
      }
      else  
      {
        currentTask = FORWARD;
      }
      break;
    case FORWARD: 
      if ((RRRotations - oldRRR) > 10)
      {
        stopDriving();
        currentTask = NO_TASK;     
      }
      driveStraight();
      break;
    case RIGHT_90:
      if (LRRotations > 4)
      {
        stopDriving();
        currentTask = NO_TASK;
      }
      break;
    case LEFT_90:
      if (RRRotations > 3)
      {
        stopDriving();
        currentTask = NO_TASK;
      }
      break;
    case FORWARD_BEFORE_RIGHT:
      if (RRRotations > 5)
      {
        LRRotations = 0;
        rotateRight();
        currentTask = RIGHT_90;
      }
      break;
  }
}


void driveToSquare()
{
  // // TODO: implement start signal
  static bool startSignalRecieved = true;
  if (startSignalRecieved)
  {
    while (!allBlack)
    {
      readLineSensor();
      goForwards();
      delay(100);
    }
    readLineSensor();
    if (allBlack)
    {
      goForwards();
      delay(500);
      setGripper(GRIPPER_CLOSED);
      rotateLeft();
      delay(480);
      goForwards();
      delay(500);
      squarePassed = true;
      startSignalRecieved = false;
    }
  }
}

void followLineStart()
{
  bool forwards = (lineSensorValue[2] >= colorBlack) && (lineSensorValue[3] >= colorBlack);
  bool turnLeft = (lineSensorValue[0] >= colorBlack) || (lineSensorValue[1] >= colorBlack);
  bool turnRight = (lineSensorValue[4] >= colorBlack) || (lineSensorValue[5] >= colorBlack);
  unsigned char lastValue = 0;
  static unsigned long timer = currentTime;

  if ((currentTime - timer) > 1000)
  {
    if (squarePassed)
    {
      if (!allWhite)
      {
        if (forwards == true)
        {
          goForwards();
        }
        else if (turnRight == true)
        {
          adjustRight(); 
          lastValue = 1;
        }
        else if (turnLeft == true)
        {
          adjustLeft();
          lastValue = 2;
        }
      }
      else
      {
        if (lastValue == 1)
        {
          adjustRight();
        }
        else
        {
          adjustLeft();
        }
      }
    }
      else
      {
        mazeEntered = true;
      }
    timer = currentTime;
  }
}

void followLineEnd()
{
  bool forwards = (lineSensorValue[2] >= colorBlack) && (lineSensorValue[3] >= colorBlack);
  bool turnLeft = (lineSensorValue[0] >= colorBlack) || (lineSensorValue[1] >= colorBlack);
  bool turnRight = (lineSensorValue[4] >= colorBlack) || (lineSensorValue[5] >= colorBlack);
  unsigned char lastValue = 0;
  static unsigned long timer = currentTime;

  if ((currentTime - timer) > 1000)
  {
    if (finishFound)
    {
      if (!allBlack)
      {
        if (!allWhite)
        {
          if (forwards == true)
          {
            goForwards();
          }
          else if (turnRight == true)
          {
            adjustRight(); 
            lastValue = 1;
          }
          else if (turnLeft == true)
          {
            adjustLeft();
            lastValue = 2;
          }
        }
        else
        {
          if (lastValue == 1)
          {
            adjustRight();
          }
          else
          {
            adjustLeft();
          }
        }
      }
      else
      {
        goForwards();
        delay(50);
        setGripper(GRIPPER_OPEN);
        stopDriving();
        while (true){}
      }
    }
    timer = currentTime;
  }
}

void findFinish()
{
  if ((!allWhite) && (needToFindFinish))
  {
    finishFound = true;
  }
}


///////////////
// NEOPIXELS //
///////////////

#define BACK_LEFT     0
#define BACK_RIGHT    1
#define FORWARD_RIGHT 2
#define FORWARD_LEFT  3


void idleLights() {
  pixels.setPixelColor(BACK_LEFT, pixels.Color(255, 255, 255));
  pixels.setPixelColor(BACK_RIGHT, pixels.Color(255, 255, 255));
  pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(255, 255, 255));
  pixels.setPixelColor(FORWARD_LEFT, pixels.Color(255, 255, 255));
  pixels.show();
}

void turnRightLights()
{
  static bool lightOn = false;
  static unsigned long timer = micros();
  if ((currentTime - timer) > 500000)
  {
    if (lightOn)
    {
        pixels.setPixelColor(BACK_LEFT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(BACK_RIGHT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(FORWARD_LEFT, pixels.Color(255, 255, 255));
        pixels.show();
    }
    else
    {
        pixels.setPixelColor(BACK_LEFT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(BACK_RIGHT, pixels.Color(135, 99, 24));
        pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(135, 99, 24));
        pixels.setPixelColor(FORWARD_LEFT, pixels.Color(255, 255, 255));
        pixels.show();
    }
    lightOn = !lightOn;
    timer = currentTime;
  }
}

void turnLeftLights()
{
  static bool lightOn = false;
  static unsigned long timer = micros();
  if ((currentTime - timer) > 500000)
  {
    if (lightOn)
    {
        pixels.setPixelColor(BACK_LEFT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(BACK_RIGHT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(FORWARD_LEFT, pixels.Color(255, 255, 255));
        pixels.show();
    }
    else
    {
        pixels.setPixelColor(BACK_RIGHT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(BACK_LEFT, pixels.Color(135, 99, 24));
        pixels.setPixelColor(FORWARD_LEFT, pixels.Color(135, 99, 24));
        pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(255, 255, 255));
        pixels.show();
    }
    lightOn = !lightOn;
    timer = currentTime;
  }
}

void brakeLights()
{
  pixels.setPixelColor(BACK_LEFT, pixels.Color(255, 0, 0));
  pixels.setPixelColor(BACK_RIGHT, pixels.Color(255, 0, 0));
  pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(255, 255, 255));
  pixels.setPixelColor(FORWARD_LEFT, pixels.Color(255, 255, 255));
  pixels.show();
}

void backwardsLights()
{
  static bool lightOn = false;
  static unsigned long timer = micros();
  if ((currentTime - timer) > 500000)
  {
    if (lightOn)
    {
        pixels.setPixelColor(BACK_LEFT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(BACK_RIGHT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(FORWARD_LEFT, pixels.Color(255, 255, 255));
        pixels.show();
    }
    else
    {
        pixels.setPixelColor(BACK_RIGHT, pixels.Color(255, 0, 0));
        pixels.setPixelColor(BACK_LEFT, pixels.Color(255, 0, 0));
        pixels.setPixelColor(FORWARD_LEFT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(255, 255, 255));
        pixels.show();
    }
    lightOn = !lightOn;
    timer = currentTime;
  }
}


void hazardLights()
{
  static bool lightOn = false;
  static unsigned long timer = micros();
  if ((currentTime - timer) > 500000)
  {
    if (lightOn)
    {
        pixels.setPixelColor(BACK_LEFT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(BACK_RIGHT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(255, 255, 255));
        pixels.setPixelColor(FORWARD_LEFT, pixels.Color(255, 255, 255));
        pixels.show();
    }
    else
    {
        pixels.setPixelColor(BACK_LEFT, pixels.Color(135, 99, 24));
        pixels.setPixelColor(BACK_RIGHT, pixels.Color(135, 99, 24));
        pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(135, 99, 24));
        pixels.setPixelColor(FORWARD_LEFT, pixels.Color(135, 99, 24));
        pixels.show();
    }
    lightOn = !lightOn;
    timer = currentTime;
  }
}
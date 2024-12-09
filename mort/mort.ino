#define DEBUG 1



///////////
// SETUP //
///////////

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

void setup() 
{
  Serial.begin(9600);                       // Begin the serial monitor
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

  updateSonar();

  if (waitForStart)
  {
    if (currentTime - startTime > 2000000)
    {
      waitForStart = false;
    }
    return;
  }

  followRightWall();

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
      debugMessage += "\nWheel rotations left:  " + String(RRRotations);
      debugMessage += "\nWheel rotations right: " + String(LRRotations);
      debugMessage += "\nTask:                  " + translateCurrentTask(currentTask);
      debugMessage += "\nTask time left (us):   " + String(currentTaskStart + currentTaskDuration - currentTime);
      

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

  void printDebugMessage_old()
  {
    if (currentTime - lastDebug > 1000000) {
      debugStart = millis();

      Serial.println("");
      Serial.println("---- ---- DIAGNOSTIC DEBUG DATA ---- ----\ntest");
      Serial.print  ("Time (us):             ");
      Serial.println(currentTime);
      Serial.print  ("Sonar left (cm):       ");
      Serial.println(distanceLeft);
      Serial.print  ("Sonar forwards (cm):   ");
      Serial.println(distanceForwards);
      Serial.print  ("Sonar right (cm):      ");
      Serial.println(distanceRight);
      Serial.print  ("Sonar left max:        ");
      Serial.println(maxDistanceLeft);
      Serial.print  ("Sonar forwards max:    ");
      Serial.println(maxDistanceForwards);
      Serial.print  ("Sonar right max:       ");
      Serial.println(maxDistanceRight);
      Serial.print  ("Wheel rotations left:  ");
      Serial.println(RRRotations);
      Serial.print  ("Wheel rotations right: ");
      Serial.println(LRRotations);
      Serial.print  ("Task:                  ");
      Serial.println(translateCurrentTask(currentTask));
      Serial.print  ("Task time left:        ");
      Serial.println(currentTaskStart + currentTaskDuration - currentTime);
      Serial.print  ("Debug duration (ms):   ");
      Serial.println(millis() - debugStart);

      maxDistanceLeft = 0;
      maxDistanceForwards = 0;
      maxDistanceRight = 0;
      lastDebug = currentTime;
    }
  }
#endif



/////////////////
// LINE SENSOR //
/////////////////

int lineSensorValue[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Array for line sensor values

// Read all the line sensor pins
void readLineSensor() 
{
  for (unsigned char i = 0; i < 8; i++) 
  {
    lineSensorValue[i] = analogRead(LINE_SENSOR[i]);
  }
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

#define MOTOR_L_FULL_SPEED 250  // Left motor full speed
#define MOTOR_L_HALF_SPEED 140  // Left motor half speed
#define MOTOR_R_FULL_SPEED 255  // Right motor full speed
#define MOTOR_R_HALF_SPEED 140  // Right motor half speed
#define MOTOR_STOP         0    // Motor stopping speed

// Makes the relaybot drive in a straight line forward
// TODO: Make the robot drive a certain speed, calibrated with rotation sensor
void goForwards() 
{
  analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
  analogWrite(MOTOR_RF, MOTOR_R_FULL_SPEED);
  digitalWrite(MOTOR_LB, MOTOR_STOP);
  digitalWrite(MOTOR_RB, MOTOR_STOP);
}

// Makes the relaybot drive in a straight line backwards
// TODO: Make the robot drive a certain speed, calibrated with rotation sensor
void goBackwards(unsigned char speed) 
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
  digitalWrite(MOTOR_LB, MOTOR_STOP);
  digitalWrite(MOTOR_RB, 1);
  analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
  analogWrite(MOTOR_RF, MOTOR_STOP); 
}

// Rotate the relaybot on its axis to the left
// TODO: Make it turn correctly
void rotateLeft()
{
  digitalWrite(MOTOR_RB, MOTOR_STOP);
  digitalWrite(MOTOR_LB, 1);
  analogWrite(MOTOR_RF, MOTOR_L_FULL_SPEED);
  analogWrite(MOTOR_LF, MOTOR_STOP); 
  Serial.println("panicLeft");
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



/////////////
// GRIPPER //
/////////////

#define GRIPPER_OPEN   1600  // Value for gripper being open
#define GRIPPER_CLOSED 1010  // Value for gripper being closed

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



////////
// AI //
////////

#define MAZE_SIZE 8
unsigned char path[MAZE_SIZE*MAZE_SIZE];  // 512 bytes

void followRightWall() 
{
  switch (currentTask) 
  {
    case NO_TASK:
      if (distanceRight > 15)
      {
        RRRotations = 0;
        LRRotations = 0;
        goForwards();
        currentTask = FORWARD_BEFORE_RIGHT;
      }
      else if (distanceForwards < 15)
      {
        RRRotations = 0;
        rotateLeft();
        currentTask = LEFT_90;
      }
      else
      {
        RRRotations = 0;
        LRRotations = 0;
        goForwards();
        currentTask = FORWARD;
      }
      break;
    case FORWARD:
      if (RRRotations > 10)
      {
        stopDriving();
        currentTask = NO_TASK;     
      }
      break;
    case RIGHT_90:
      if (LRRotations > 6)
      {
        stopDriving();
        currentTask = NO_TASK;
      }
      break;
    case LEFT_90:
      if (RRRotations > 6)
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

unsigned long distanceRatio = 10;

// turning ratio: 0.44865 * 250 = 112.1625
// max speed: 13.77cm/s
// outer wheel quarter circle: 32.53cm / 13.77cm/s = 2.3624s = 2362.4ms = 2362400us
void turnRight_test() 
{
  analogWrite(MOTOR_RF, 112);
  analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
  digitalWrite(MOTOR_RB, MOTOR_STOP);
  digitalWrite(MOTOR_LB, MOTOR_STOP);

  currentTask = RIGHT_90;
  currentTaskStart = currentTime;
  currentTaskDuration = 23624*distanceRatio;
}

void turnLeft_test() 
{
  analogWrite(MOTOR_RF, MOTOR_R_FULL_SPEED);
  analogWrite(MOTOR_LF, 110);
  digitalWrite(MOTOR_RB, MOTOR_STOP);
  digitalWrite(MOTOR_LB, MOTOR_STOP);

  currentTask = LEFT_90;
  currentTaskStart = currentTime;
  currentTaskDuration = 23624*distanceRatio;
}

void forward_test()
{
  analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
  analogWrite(MOTOR_RF, MOTOR_R_FULL_SPEED);
  digitalWrite(MOTOR_LB, MOTOR_STOP);
  digitalWrite(MOTOR_RB, MOTOR_STOP);

  currentTask = FORWARD;
  currentTaskStart = currentTime;
  currentTaskDuration = 21786*distanceRatio; // 30cm / 13.77cm/s = 2.17865s = 2178650us
}

void turnAround_test() 
{
  analogWrite(MOTOR_LF, MOTOR_L_FULL_SPEED);
  analogWrite(MOTOR_RF, MOTOR_STOP); 
  digitalWrite(MOTOR_LB, MOTOR_STOP);
  digitalWrite(MOTOR_RB, 1);

  currentTask = ROTATE_180;
  currentTaskStart = currentTime;
  currentTaskDuration = 13000*distanceRatio; // 17.9353cm / 13.77cm/s = 1.3s = 1300000us
}

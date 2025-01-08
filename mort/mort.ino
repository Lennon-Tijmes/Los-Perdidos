// ██████╗ ███████╗██╗      █████╗ ██╗   ██╗██████╗  ██████╗ ████████╗
// ██╔══██╗██╔════╝██║     ██╔══██╗╚██╗ ██╔╝██╔══██╗██╔═══██╗╚══██╔══╝
// ██████╔╝█████╗  ██║     ███████║ ╚████╔╝ ██████╔╝██║   ██║   ██║   
// ██╔══██╗██╔══╝  ██║     ██╔══██║  ╚██╔╝  ██╔══██╗██║   ██║   ██║   
// ██║  ██║███████╗███████╗██║  ██║   ██║   ██████╔╝╚██████╔╝   ██║   
// ╚═╝  ╚═╝╚══════╝╚══════╝╚═╝  ╚═╝   ╚═╝   ╚═════╝  ╚═════╝    ╚═╝   
//                ███╗   ███╗ ██████╗ ██████╗ ████████╗               
//                ████╗ ████║██╔═══██╗██╔══██╗╚══██╔══╝               
//                ██╔████╔██║██║   ██║██████╔╝   ██║                  
//                ██║╚██╔╝██║██║   ██║██╔══██╗   ██║                  
//                ██║ ╚═╝ ██║╚██████╔╝██║  ██║   ██║                  
//                ╚═╝     ╚═╝ ╚═════╝ ╚═╝  ╚═╝   ╚═╝                  



//#define DEBUG 1
#define STARTUP_DELAY 2000



///////////
// SETUP //
///////////

#include <Adafruit_NeoPixel.h> //neopixel library
#define NUM_PIXELS 4 //number of neopixels
#define PIXEL_PIN 11 //pin for the neopixels
Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_RGB + NEO_KHZ800);

// Pins
const unsigned char LINE_SENSOR_PINS[] = {A2, A3, A4, A5, A6, A7};
#define SONAR_LEFT_TRIG_PIN       5
#define SONAR_LEFT_ECHO_PIN       6
#define SONAR_FRONT_TRIG_PIN      12
#define SONAR_FRONT_ECHO_PIN      13
#define SONAR_RIGHT_TRIG_PIN      A1 // 15
#define SONAR_RIGHT_ECHO_PIN      A0 // 14
#define ROTATION_SENSOR_LEFT_PIN  2
#define ROTATION_SENSOR_RIGHT_PIN 3
#define MOTOR_LEFT_POWER_PIN      9
#define MOTOR_RIGHT_POWER_PIN     10
#define MOTOR_LEFT_MODE_PIN       7
#define MOTOR_RIGHT_MODE_PIN      8
#define GRIPPER_PIN               4

// Phase
#define STARTUP_WAIT      0
#define WAIT_FOR_START    1 // Wait for start signal
#define CALIBRATE         2 // Drive a bit and calculate motor strength and timings
#define MAP_MAZE          3 // Go look for maze solution
#define RETURN_TO_START   4 // Stop looking and drive back to maze start
#define WAIT_FOR_SIGNAL   5 // Wait for signal from master
#define DRIVE_TO_SQUARE   6 // drive forward till black square, grab the pin and turn left
#define FOLLOW_LINE_START 7 // follow the line untill inside of the maze
#define DRIVE_PATH        8 // Drive the mapped path through the maze
#define CONTINUE_MAZE     9 // If maze isn't solved yet, solve the rest of the maze
#define FOLLOW_LINE_END   10 // Follow the finishing line and put pin inside black square
#define FINISH            11
unsigned char programPhase;
unsigned long programStartTime;

// Task
#define CHOOSE_TASK        0
#define DRIVE_FORWARDS_30  1
#define DRIVE_BACKWARDS_30 2
#define TURN_LEFT_90       3
#define TURN_RIGHT_90      4
#define SHORT_LEFT_90      5
#define SHORT_RIGHT_90     6
#define ROTATE_LEFT_180    7  // anti-clockwise
#define ROTATE_RIGHT_180   8  // clockwise
#define TURN_AROUND_LEFT   9  // anti-clockwise
#define TURN_AROUND_RIGHT  10 // clockwise
#define ADJUST_ROTATION    11
#define ADJUST_POSITION    12
#define PAUSE              13
#define UPDATE_SONAR       14
#define DRIVE_FORWARDS_2   15
#define UPDATE_SONAR_2     16
#define PATH_FINISHED      99
#define GIVE_UP            100
unsigned char currentTask = 0;
unsigned long currentTaskStart = 0;
unsigned long currentTaskDuration = 0;
unsigned char nextTask = 0;

// Movement
#define STOP              0
#define FORWARDS          1
#define BACKWARDS         2
#define TURN_LEFT         3
#define TURN_RIGHT        4
#define TURN_LEFT_BACK    5
#define TURN_RIGHT_BACK   6
#define SHORT_LEFT        7
#define SHORT_RIGHT       8
#define SHORT_LEFT_BACK   9
#define SHORT_RIGHT_BACK  10
#define ROTATE_LEFT       11
#define ROTATE_RIGHT      12
#define ADJUST_LEFT       13
#define ADJUST_RIGHT      14
#define ADJUST_LEFT_HARD  15
#define ADJUST_RIGHT_HARD 16
int leftSpeed = 0;
int rightSpeed = 0;

int   forwardsLeftMotorSpeed = 250;
int  forwardsRightMotorSpeed = 255;
int  backwardsLeftMotorSpeed = -250;
int backwardsRightMotorSpeed = -255;
int   leftTurnLeftMotorSpeed = 98;
int  leftTurnRightMotorSpeed = 255;
int  rightTurnLeftMotorSpeed = 255;
int rightTurnRightMotorSpeed = 105;

// Sensors
unsigned char sonarPhase = 0;
unsigned long sonarLastActionTime = 0;
unsigned long sonarSignalDuration = 0;

#define SONAR_TOO_FAR 1023
#define SONAR_NO_READING 1024
double distanceLeft = SONAR_NO_READING;
double distanceFront = SONAR_NO_READING;
double distanceRight = SONAR_NO_READING;
double minDistanceLeft = SONAR_NO_READING;
double minDistanceFront = SONAR_NO_READING;
double minDistanceRight = SONAR_NO_READING;
double maxDistanceLeft = 0;
double maxDistanceFront = 0;
double maxDistanceRight = 0;

#define MAX_SONAR_UPDATES 3
unsigned int sonarUpdates = 0;
double distancesLeft[MAX_SONAR_UPDATES];
double distancesFront[MAX_SONAR_UPDATES];
double distancesRight[MAX_SONAR_UPDATES];

#define MAX_INT 2147483647
unsigned int leftRotationTicks = 0;
unsigned int rightRotationTicks = 0;
unsigned int prevLeftRotationTicks = 0;
unsigned int prevRightRotationTicks = 0;
unsigned int targetLeftRotationTicks = MAX_INT;
unsigned int targetRightRotationTicks = MAX_INT;

// Lights
bool allBlack = false;
bool allWhite = false;
unsigned int colorBlack = 850;
unsigned int colorWhite = 600;

// Other
#define SLAVE_ID 1
unsigned long currentTime;

bool mazeEntered = false;
bool squarePassed = false;
bool finishFound = false;
bool startSignalRecieved = false;

void setup()
{
  Serial.begin(9600);                                           // Begin the serial monitor
  pixels.begin();
  Serial.println("Slave started");
  pinMode(SONAR_LEFT_TRIG_PIN, OUTPUT);                         // Initialize the left  sonar trig pin as output
  pinMode(SONAR_LEFT_ECHO_PIN, INPUT);                          // Initialize the left  sonar echo pin as input
  pinMode(SONAR_FRONT_TRIG_PIN, OUTPUT);                        // Initialize the front sonar trig pin as output
  pinMode(SONAR_FRONT_ECHO_PIN, INPUT);                         // Initialize the front sonar echo pin as input
  pinMode(SONAR_RIGHT_TRIG_PIN, OUTPUT);                        // Initialize the right sonar trig pin as output
  pinMode(SONAR_RIGHT_ECHO_PIN, INPUT);                         // Initialize the right sonar echo pin as input
  pinMode(ROTATION_SENSOR_LEFT_PIN, INPUT_PULLUP);              // Initialize the left  rotation sensor as pullup input
  pinMode(ROTATION_SENSOR_RIGHT_PIN, INPUT_PULLUP);             // Initialize the right rotation sensor as pullup input
  pinMode(MOTOR_LEFT_POWER_PIN, OUTPUT);                        // Initialize the left  motor power pin as output
  pinMode(MOTOR_RIGHT_POWER_PIN, OUTPUT);                       // Initialize the right motor power pin as output
  pinMode(MOTOR_LEFT_MODE_PIN, OUTPUT);                         // Initialize the left  motor mode  pin  as output
  pinMode(MOTOR_RIGHT_MODE_PIN, OUTPUT);                        // Initialize the right motor mode  pin  as output
  pinMode(GRIPPER_PIN, OUTPUT);                                 // Initialize the gripper pin as output
  for (char i = 0; i < 6; i++) pinMode(LINE_SENSOR_PINS[i], INPUT); // Initialize the line sensor pins as input

  attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_LEFT_PIN),  countRotationsLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_RIGHT_PIN), countRotationsRight, RISING);

  currentTime = millis();
  programStartTime = currentTime;

  programPhase = STARTUP_WAIT;
  currentTask = UPDATE_SONAR;
  currentTaskDuration = 2000;
  nextTask = CHOOSE_TASK;
}

void loop()
{
  currentTime = millis();
  updateSonar();
  updateLineSensor();

  switch(programPhase)
  {
    case STARTUP_WAIT:
    {
      phase_startupWait();
    } break;
    case WAIT_FOR_START:
    {
      phase_waitForStart();
    } break;
    case CALIBRATE:
    {
      phase_calibrate();
    } break;
    case MAP_MAZE:
    {
      phase_mapMaze();
    } break;
    case RETURN_TO_START:
    {
      phase_returnToStart();
    } break;
    case WAIT_FOR_SIGNAL:
    {
      phase_waitForSignal();
    } break;
    case DRIVE_TO_SQUARE:
    {
      phase_driveToSquare();
    } break;
    case FOLLOW_LINE_START:
    {
      phase_followLineStart();
    } break;
    case CONTINUE_MAZE:
    {
      phase_continueMaze();
    } break;
    case FOLLOW_LINE_END:
    {
      phase_followLineEnd();
    } break;
    case FINISH:
    {
      phase_finish();
    } break;
  }
}



/////////////////
// LINE SENSOR //
/////////////////

int lineSensorValue[6] = {0, 0, 0, 0, 0, 0};

// Read all the line sensor pins
void updateLineSensor()
{
  for (unsigned char i = 0; i < 6; i++)
  {
    lineSensorValue[i] = analogRead(LINE_SENSOR_PINS[i]);
  }

  allBlack =  (lineSensorValue[0] >= colorBlack)
           && (lineSensorValue[1] >= colorBlack)
           && (lineSensorValue[2] >= colorBlack)
           && (lineSensorValue[3] >= colorBlack)
           && (lineSensorValue[4] >= colorBlack)
           && (lineSensorValue[5] >= colorBlack); // true if all the line sensor bits are looking at black

  allWhite =  (lineSensorValue[0] <= colorWhite)
           && (lineSensorValue[1] <= colorWhite)
           && (lineSensorValue[2] <= colorWhite)
           && (lineSensorValue[3] <= colorWhite)
           && (lineSensorValue[4] <= colorWhite)
           && (lineSensorValue[5] <= colorWhite); // true if all the line sensor bits are looking at white
}



///////////
// SONAR //
///////////

// The HC-SR04 manual recommends sending a 10 microsecond pulse to TRIG pin
#define SONAR_SIGNAL_DURATION_US  10

// Minimum delay before switching to next sonar
#define SONAR_DELAY_MS            10

// We assume the maze is 7x7, which means the longest distance to measure should be 7*30cm = 210cm
// Speed of sound is around 35cm/ms=210cm/6ms, so the sound signal should take at most 12ms to come back to the robot
// Accounting for some inaccuracies it's safe to set the timeout to 15ms
#define SONAR_RECEIVER_TIMEOUT_MS 15

// Sonar phases
#define SONAR_LEFT_SEND_SIGNAL  0
#define SONAR_LEFT_READ_SIGNAL  1
#define SONAR_FRONT_SEND_SIGNAL 2
#define SONAR_FRONT_READ_SIGNAL 3
#define SONAR_RIGHT_SEND_SIGNAL 4
#define SONAR_RIGHT_READ_SIGNAL 5

const double microsecondsToCentimeters = 0.017; // 340m/s = 34000cm/s = 34cm/ms = 0.034cm/us, sound travels to object and back so divide by 2 for distance to object

void updateSonar()
{
  if (sonarUpdates >= MAX_SONAR_UPDATES)
  {
    return;
  }

  switch (sonarPhase)
  {
    // Left sonar

    // Send pulse on left sonar
    case SONAR_LEFT_SEND_SIGNAL:
      if (currentTime - sonarLastActionTime > SONAR_DELAY_MS)
      {

        digitalWrite(SONAR_LEFT_TRIG_PIN, HIGH);
        delayMicroseconds(SONAR_SIGNAL_DURATION_US);
        digitalWrite(SONAR_LEFT_TRIG_PIN, LOW);
        sonarLastActionTime = currentTime;
        sonarPhase = SONAR_LEFT_READ_SIGNAL;
      }
      break;

    // Get HIGH state duration from ECHO pin, or move to next phase if no signal after SONAR_RECEIVER_TIMEOUT_MS milliseconds
    case SONAR_LEFT_READ_SIGNAL:
      sonarSignalDuration = pulseIn(SONAR_LEFT_ECHO_PIN, HIGH);
      if (sonarSignalDuration > 0)
      {
        if (sonarSignalDuration > 12353) distanceLeft = SONAR_TOO_FAR; // if more than 210cm
        else distanceLeft = (double)sonarSignalDuration * microsecondsToCentimeters;
        if (minDistanceLeft > distanceLeft) minDistanceLeft = distanceLeft;
        if (maxDistanceLeft < distanceLeft) maxDistanceLeft = distanceLeft;
        distancesLeft[sonarUpdates] = distanceLeft;
        
        sonarSignalDuration = 0;
        sonarLastActionTime = currentTime;
        sonarPhase = SONAR_FRONT_SEND_SIGNAL;
      }
      else if (currentTime - sonarLastActionTime > SONAR_RECEIVER_TIMEOUT_MS)
      {
        distanceLeft = SONAR_NO_READING;
        distancesLeft[sonarUpdates] = distanceLeft;
        sonarLastActionTime = currentTime;
        sonarPhase = SONAR_FRONT_SEND_SIGNAL;
      }
      break;

    // Front sonar

    // Send pulse on front sonar
    case SONAR_FRONT_SEND_SIGNAL:
      if (currentTime - sonarLastActionTime > SONAR_DELAY_MS)
      {
        digitalWrite(SONAR_FRONT_TRIG_PIN, HIGH);
        delayMicroseconds(SONAR_SIGNAL_DURATION_US);
        digitalWrite(SONAR_FRONT_TRIG_PIN, LOW);
        sonarLastActionTime = currentTime;
        sonarPhase = SONAR_FRONT_READ_SIGNAL;
      }
      break;

    // Get HIGH state duration from ECHO pin, or move to next phase if no signal after SONAR_RECEIVER_TIMEOUT_MS milliseconds
    case SONAR_FRONT_READ_SIGNAL:
      sonarSignalDuration = pulseIn(SONAR_FRONT_ECHO_PIN, HIGH);
      if (sonarSignalDuration > 0)
      {
        if (sonarSignalDuration > 12353) distanceFront = SONAR_TOO_FAR; // if more than 210cm
        else distanceFront = (double)sonarSignalDuration * microsecondsToCentimeters;
        if (minDistanceFront > distanceFront) minDistanceFront = distanceFront;
        if (maxDistanceFront < distanceFront) maxDistanceFront = distanceFront;
        distancesFront[sonarUpdates] = distanceFront;

        sonarSignalDuration = 0;
        sonarLastActionTime = currentTime;
        sonarPhase = SONAR_RIGHT_SEND_SIGNAL;
      }
      else if (currentTime - sonarLastActionTime > SONAR_RECEIVER_TIMEOUT_MS)
      {
        distanceFront = SONAR_NO_READING;
        distancesFront[sonarUpdates] = distanceFront;
        sonarLastActionTime = currentTime;
        sonarPhase = SONAR_RIGHT_SEND_SIGNAL;
      }
      break;

    // Right sonar

    // Send pulse on right sonar
    case SONAR_RIGHT_SEND_SIGNAL:
      if (currentTime - sonarLastActionTime > SONAR_DELAY_MS)
      {
        digitalWrite(SONAR_RIGHT_TRIG_PIN, HIGH);
        delayMicroseconds(SONAR_SIGNAL_DURATION_US);
        digitalWrite(SONAR_RIGHT_TRIG_PIN, LOW);
        sonarLastActionTime = currentTime;
        sonarPhase = SONAR_RIGHT_READ_SIGNAL;
      }
      break;

    // Get HIGH state duration from ECHO pin, or move to next phase if no signal after SONAR_RECEIVER_TIMEOUT_MS milliseconds
    case SONAR_RIGHT_READ_SIGNAL:
    {
      sonarSignalDuration = pulseIn(SONAR_RIGHT_ECHO_PIN, HIGH);
      if (sonarSignalDuration > 0)
      {
        if (sonarSignalDuration > 12353) distanceRight = SONAR_TOO_FAR; // if more than 210cm
        else distanceRight = (double)sonarSignalDuration * microsecondsToCentimeters;
        if (minDistanceRight > distanceRight) minDistanceRight = distanceRight;
        if (maxDistanceRight < distanceRight) maxDistanceRight = distanceRight;
        distancesRight[sonarUpdates] = distanceRight;

        sonarSignalDuration = 0;
        sonarLastActionTime = currentTime;
        sonarPhase = SONAR_LEFT_SEND_SIGNAL;
        sonarUpdates++;
      }
      else if (currentTime - sonarLastActionTime > SONAR_RECEIVER_TIMEOUT_MS)
      {
        distanceRight = SONAR_NO_READING;
        distancesRight[sonarUpdates] = distanceRight;
        sonarLastActionTime = currentTime;
        sonarPhase = SONAR_LEFT_SEND_SIGNAL;
        sonarUpdates++;
      }
    }
    break;
  }
}



/////////////////////
// ROTATION SENSOR //
/////////////////////

#define DEBOUNCE_TIME_MS 10; // Debounce time for more accurate rotation sensor reading

// Counts the interrupts of the rotation sensor for the left wheel
void countRotationsLeft()
{
  static unsigned long timer;
  noInterrupts();

  if (millis() > timer)
  {
    leftRotationTicks++;
    timer = millis() + DEBOUNCE_TIME_MS;
  }

  interrupts();
}

// Counts the interrupts of the rotation sensor for the right wheel
void countRotationsRight()
{
  static unsigned long timer;
  noInterrupts();

  if (millis() > timer)
  {
    rightRotationTicks++;
    timer = millis() + DEBOUNCE_TIME_MS;
  }

  interrupts();
}



///////////
// MOTOR //
///////////

#define FORWARDS_MODE LOW
#define BACKWARDS_MODE HIGH

void setMotors(int leftMotorSpeed, int rightMotorSpeed)
{
  leftSpeed = leftMotorSpeed;
  rightSpeed = rightMotorSpeed;

  if (leftMotorSpeed >= 0)
  {
    analogWrite (MOTOR_LEFT_POWER_PIN, leftMotorSpeed);
    digitalWrite(MOTOR_LEFT_MODE_PIN, FORWARDS_MODE);
  }
  else /* leftMotorSpeed < 0 */
  {
    analogWrite (MOTOR_LEFT_POWER_PIN, 255 + leftMotorSpeed);
    digitalWrite(MOTOR_LEFT_MODE_PIN, BACKWARDS_MODE);
  }

  if (rightMotorSpeed >= 0)
  {
    analogWrite (MOTOR_RIGHT_POWER_PIN, rightMotorSpeed);
    digitalWrite(MOTOR_RIGHT_MODE_PIN, FORWARDS_MODE);
  }
  else /* rightMotorSpeed < 0 */
  {
    analogWrite (MOTOR_RIGHT_POWER_PIN, 255 + rightMotorSpeed);
    digitalWrite(MOTOR_RIGHT_MODE_PIN, BACKWARDS_MODE);
  }
}

void drive(char mode)
{
  switch(mode)
  {
    case STOP:
    {
      setMotors(0, 0);
    } break;
    case FORWARDS:
    {
      setMotors(forwardsLeftMotorSpeed, forwardsRightMotorSpeed);
    } break;
    case BACKWARDS:
    {
      setMotors(backwardsLeftMotorSpeed, backwardsRightMotorSpeed);
    } break;

    case TURN_LEFT:
    {
      setMotors(leftTurnLeftMotorSpeed, leftTurnRightMotorSpeed);
    } break;
    case TURN_RIGHT:
    {
      setMotors(rightTurnLeftMotorSpeed, rightTurnRightMotorSpeed);
    } break;
    case TURN_LEFT_BACK:
    {
      setMotors(-leftTurnLeftMotorSpeed, -leftTurnRightMotorSpeed);
    } break;
    case TURN_RIGHT_BACK:
    {
      setMotors(-rightTurnLeftMotorSpeed, -rightTurnRightMotorSpeed);
    } break;

    case SHORT_LEFT:
    {
      setMotors(0, leftTurnRightMotorSpeed);
    } break;
    case SHORT_RIGHT:
    {
      setMotors(rightTurnLeftMotorSpeed, 0);
    } break;
    case SHORT_LEFT_BACK:
    {
      setMotors(0, -leftTurnRightMotorSpeed);
    } break;
    case SHORT_RIGHT_BACK:
    {
      setMotors(-rightTurnLeftMotorSpeed, 0);
    } break;

    case ROTATE_LEFT:
    {
      setMotors(backwardsLeftMotorSpeed, forwardsRightMotorSpeed);
    } break;
    case ROTATE_RIGHT:
    {
      setMotors(forwardsLeftMotorSpeed, backwardsRightMotorSpeed);
    } break;

    case ADJUST_LEFT:
    {
      setMotors(160, 240);
    } break;
    case ADJUST_RIGHT:
    {
      setMotors(240, 165);
    } break;
    case ADJUST_LEFT_HARD:
    {
      setMotors(-190, 240);
    } break;
    case ADJUST_RIGHT_HARD:
    {
      setMotors(240, -190);
    } break;
  }
}



/////////////
// GRIPPER //
/////////////

#define GRIPPER_OPEN   1600 // Value for gripper being open
#define GRIPPER_CLOSED 1010 // Value for gripper being closed

// Sets the gripper position to the given pulse
void setGripper(unsigned int pulse)
{
  for (unsigned char i = 0; i < 8; i++)
  {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GRIPPER_PIN, LOW);
  }
}



//////////////////////////
// PHASE - STARTUP_WAIT //
//////////////////////////

void phase_startupWait()
{
  if (startupDelay())
  {
    programPhase = CALIBRATE;
  }
}
bool startupDelay()
{
  return currentTime - programStartTime > STARTUP_DELAY;
}



////////////////////////////
// PHASE - WAIT_FOR_START //
////////////////////////////

void phase_waitForStart()
{
  if (hasReceivedSignal())
  {
    programPhase = CALIBRATE;
  }
}

bool hasReceivedSignal()
{
  if (Serial.available())
  {
    String message = Serial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(message);

    if (message.length() >= 2 && message[0] == SLAVE_ID + '0' && message[1] == '?')
    {
      Serial.print("Responding to Master: Slave ");
      Serial.print(SLAVE_ID);
      Serial.println(" Response");
      return true;
    }
  }
  else
  {
    // maybe flash lights to say that we cannot connect to base
  }

  return false;
}



///////////////////////
// PHASE - CALIBRATE //
///////////////////////

void phase_calibrate()
{
  // verify distance on left and front sensor (min 5cm)
  // set calibration time based on assumed speed and front distance

  // rotate right
  // forward 5 seconds
  // count rotations
  // backwards 5 seconds (give wheels time to decelerate and accelerate)
  // count rotations
  // adjust wheel speed for straight driving
  // adjust wheel speed for backwards speed same as front

  // drive full circle until we see the line again
  // adjust turn time to 1/4 of full circle time


  // or just don't bother
  programPhase = DRIVE_TO_SQUARE;
}



//////////////////////
// PHASE - MAP_MAZE //
//////////////////////

#define MAX_PATH_LENGTH 49
unsigned char pathLength = 0;
unsigned char pathMapped[MAX_PATH_LENGTH];
unsigned char pathIndex = 0;

void phase_mapMaze()
{
  #ifdef SKIP_MAPPING_MAZE
    programPhsase = DRIVE_TO_SQUARE;
    return;
  #endif

  switch (currentTask)
  {
    case CHOOSE_TASK:
    {
      //TODO check if should go next phase
      drive(STOP);
      // updateAllSonars();

      if (distanceRight > 20)
      {
        addToMappedPath(TURN_RIGHT_90);
        startTask(TURN_RIGHT_90);
      }
      else if (distanceFront > 30)
      {
        addToMappedPath(DRIVE_FORWARDS_30);
        startTask(DRIVE_FORWARDS_30);
      }
      else if (distanceLeft > 20)
      {
        addToMappedPath(TURN_LEFT_90);
        startTask(TURN_LEFT_90);
      }
      else
      {
        cutFromMappedPath(1);
        turnAround();
        //TODO after turning around keep removing from mapped path as long as we keep doing opposite moves to saved ones
      }
    } break;
  }

  updateCurrentTask();

  // checkLineSensorForFinish();
}

void addToMappedPath(unsigned char action)
{
  pathMapped[pathIndex++] = action;
}
void cutFromMappedPath(unsigned char number)
{
  for (unsigned char i = 0; i < number; i++)
  {
    pathMapped[pathIndex--] = CHOOSE_TASK;
  }
}

void resetTaskVariables(unsigned char task, unsigned long duration, unsigned char _nextTask)
{
  printDebug(String("Setting task: ") + translateTask(task) + " for " + duration + "ms");
  printDebug(String("Next task: ") + translateTask(_nextTask));
  currentTaskStart = currentTime;
  currentTask = task;
  currentTaskDuration = duration;
  nextTask = _nextTask;
  prevLeftRotationTicks = leftRotationTicks;
  prevRightRotationTicks = rightRotationTicks;
}
void resetTaskVariables(unsigned char task)
{
  resetTaskVariables(task, 100, CHOOSE_TASK);
}

void updateCurrentTask()
{
  if (currentTime - currentTaskStart > currentTaskDuration)
  {
    startTask(nextTask);
    return;
  }

  if (leftRotationTicks >= targetLeftRotationTicks)
  {
    startTask(nextTask);
    return;
  }

  if (rightRotationTicks >= targetRightRotationTicks)
  {
    startTask(nextTask);
    return;
  }

  if (currentTask == UPDATE_SONAR || currentTask == UPDATE_SONAR_2)
  {
    if (minDistanceLeft < SONAR_NO_READING && minDistanceFront < SONAR_NO_READING && minDistanceRight < SONAR_NO_READING)
    {
      startTask(nextTask);
      printDebug(String("Sonars updated: ") + distanceLeft + ", " + distanceFront + ", " + distanceRight);
    }

    if (sonarUpdates >= MAX_SONAR_UPDATES)
    {
      startTask(nextTask);
      printDebug(String("Sonars updated: ") + distanceLeft + ", " + distanceFront + ", " + distanceRight);
    }
  }
}

void startTask(unsigned char task)
{
  printDebug(String("Starting task ") + translateTask(task));
  targetLeftRotationTicks = MAX_INT;
  targetRightRotationTicks = MAX_INT;
  hazardLights();

  switch (task)
  {
    case CHOOSE_TASK:
    {
      resetTaskVariables(task, 1000, CHOOSE_TASK);
      drive(STOP);
    } break;

    case DRIVE_FORWARDS_30: { // original: 1176 | 30
      resetTaskVariables(task, 1120, UPDATE_SONAR); // 30 ticks => targetLeftRotationTicks = leftRotationTicks+30
      drive(FORWARDS);
    } break;
    case DRIVE_BACKWARDS_30: { // original: 1176 | 30
      resetTaskVariables(task, 1120, UPDATE_SONAR); // 30 ticks
      drive(BACKWARDS);
    } break;

    case TURN_LEFT_90: { // original: 1263 | 8
      resetTaskVariables(task, 1263, UPDATE_SONAR); // 8 ticks
      drive(TURN_LEFT);
    } break;
    case TURN_RIGHT_90: { // original: 1263 | 8
      resetTaskVariables(task, 1255, UPDATE_SONAR); // 8 ticks
      drive(TURN_RIGHT);
    } break;
    case SHORT_LEFT_90: { // original: 1263 | 8
      resetTaskVariables(task, 800, UPDATE_SONAR); // 8 ticks
      drive(SHORT_LEFT);
    } break;
    case SHORT_RIGHT_90: { // original: 1263 | 8
      resetTaskVariables(task, 800, UPDATE_SONAR); // 8 ticks
      drive(SHORT_RIGHT);
    } break;

    case ROTATE_LEFT_180: { // original: 1336 | 8.5
      resetTaskVariables(task, 800, UPDATE_SONAR); // 8.462 ticks
      drive(ROTATE_LEFT);
    } break;
    case ROTATE_RIGHT_180: { // original: 1336 | 8.5
      resetTaskVariables(task, 800, UPDATE_SONAR); // 8.462 ticks
      drive(ROTATE_RIGHT);
    } break;
    case TURN_AROUND_LEFT: { // original: 1263 | 30
      resetTaskVariables(task, 800, SHORT_LEFT_90); // TODO calculate
      drive(SHORT_RIGHT_BACK);
    } break;
    case TURN_AROUND_RIGHT: { // original: 1263 | 30
      resetTaskVariables(task, 800, SHORT_RIGHT_90); // TODO calculate
      drive(SHORT_LEFT_BACK);
    } break;

    case UPDATE_SONAR:
    {
      resetSonar();
      resetTaskVariables(task, 350, DRIVE_FORWARDS_2);
      drive(STOP);
    } break;
    case DRIVE_FORWARDS_2:
    {
      resetTaskVariables(task, 79, UPDATE_SONAR_2);
      drive(FORWARDS);
    } break;
    case UPDATE_SONAR_2:
    {
      resetTaskVariables(task, 350, CHOOSE_TASK);
      drive(STOP);
    } break;
    
    case ADJUST_ROTATION:
    {
      adjustRotation();
      resetTaskVariables(task, 1000, CHOOSE_TASK);
    } break;
    case ADJUST_POSITION:
    {

    } break;

    case PAUSE:
    {
      resetTaskVariables(task, 1000, CHOOSE_TASK);
      drive(STOP);
    } break;
  }
}

void resetSonar()
{
  sonarUpdates = 0;
  minDistanceLeft = SONAR_NO_READING;
  minDistanceFront = SONAR_NO_READING;
  minDistanceRight = SONAR_NO_READING;
  maxDistanceLeft = 0;
  maxDistanceFront = 0;
  maxDistanceRight = 0;

  for (int i = 0; i < MAX_SONAR_UPDATES; i++)
  {
    distancesLeft[i] = distancesFront[i] = distancesRight[i] = SONAR_NO_READING;
  }
}

void adjustRotation()
{
  unsigned int lastTaskLeftRotationTicks = leftRotationTicks - prevLeftRotationTicks;
  unsigned int lastTaskRightRotationTicks = rightRotationTicks - prevRightRotationTicks;

  switch (currentTask)
  {
    case TURN_LEFT_90:
    case SHORT_LEFT_90:
    case ROTATE_LEFT_180:
    {
      lastTaskLeftRotationTicks += 17;
    } break;
    case TURN_RIGHT_90:
    case SHORT_RIGHT_90:
    case ROTATE_RIGHT_180:
    {
      lastTaskRightRotationTicks += 17;
    } break;
  }

  if (lastTaskLeftRotationTicks > lastTaskRightRotationTicks + 1)
  {
    targetRightRotationTicks = rightRotationTicks + lastTaskLeftRotationTicks - lastTaskRightRotationTicks;
    drive(SHORT_LEFT);
    //TODO adjust speed variables depending on task
    printDebug(String("Adjusting rotation left for ") + (targetRightRotationTicks - leftRotationTicks) + " ticks");
  }
  if (lastTaskRightRotationTicks > lastTaskLeftRotationTicks + 1)
  {
    targetLeftRotationTicks = leftRotationTicks + lastTaskRightRotationTicks - lastTaskLeftRotationTicks;
    drive(SHORT_RIGHT);
    //TODO adjust speed variables depending on task
    printDebug(String("Adjusting rotation right for ") + (targetLeftRotationTicks - rightRotationTicks) + " ticks");
  }
}

void adjustPosition()
{

}

// we assume we're surrounded by 3 walls and not touching any wall
void turnAround()
{
  if (distanceRight > distanceLeft)
  {
    if (maxDistanceLeft > 5)
    {
      startTask(TURN_AROUND_RIGHT);
    }
    else
    {
      startTask(ROTATE_RIGHT_180);
    }
  }
  else
  {
    if (maxDistanceRight > 5)
    {
      startTask(TURN_AROUND_LEFT);
    }
    else
    {
      startTask(ROTATE_LEFT_180);
    }
  }
}

void checkLineSensorForFinish()
{
  updateLineSensor();

  if (!allWhite)
  {
    finishFound = true;
  }
}



/////////////////////////////
// PHASE - RETURN_TO_START //
/////////////////////////////

void phase_returnToStart()
{

  programPhase = WAIT_FOR_SIGNAL;
}



/////////////////////////////
// PHASE - WAIT_FOR_SIGNAL //
/////////////////////////////

void phase_waitForSignal()
{
  if (hasReceivedSignal())
  {
    programPhase = DRIVE_TO_SQUARE;
  }
}


long timr = 0;
/////////////////////////////
// PHASE - DRIVE_TO_SQUARE //
/////////////////////////////

void phase_driveToSquare()
{
  while (!allBlack)
  {
    updateLineSensor();
    drive(FORWARDS);
    delay(130);
  }

  updateLineSensor();

  if (allBlack)
  {
    drive(FORWARDS);
    delay(100);
    setGripper(GRIPPER_CLOSED);
    drive(ROTATE_LEFT);
    delay(480);
    drive(FORWARDS);
    delay(500);
    programPhase = FOLLOW_LINE_START;
    timr = currentTime;
  }
}



///////////////////////////////
// PHASE - FOLLOW_LINE_START //
///////////////////////////////

#define NONE  0
#define LEFT  1
#define RIGHT 2

void phase_followLineStart()
{
  static unsigned char lastDirection = NONE;

  bool goForwards = (lineSensorValue[2] >= colorBlack) && (lineSensorValue[3] >= colorBlack);
  bool turnLeft   = (lineSensorValue[0] >= colorBlack) || (lineSensorValue[1] >= colorBlack);
  bool turnRight  = (lineSensorValue[4] >= colorBlack) || (lineSensorValue[5] >= colorBlack);

  if (allWhite)
  {
    if (lastDirection == LEFT)
    {
      drive(ADJUST_LEFT);
    }
    else
    {
      drive(ADJUST_RIGHT);
    }
  }
  else
  {
    if (goForwards == true)
    {
      drive(FORWARDS);
    }
    else if (turnRight == true)
    {
      drive(ADJUST_RIGHT);
      lastDirection = RIGHT;
    }
    else if (turnLeft == true)
    {
      drive(ADJUST_LEFT);
      lastDirection = LEFT;
    }
  }
  setGripper(GRIPPER_CLOSED);
  resetSonar();
  updateSonar();

  if (distanceLeft < 30 && distanceFront < 40 && distanceRight < 30)
  {
    programPhase = CONTINUE_MAZE;
  }
  if (currentTime - timr > 3000)
  {
    programPhase = CONTINUE_MAZE;
  }
}



////////////////////////
// PHASE - DRIVE_PATH //
////////////////////////

void phase_drivePath()
{
  switch (currentTask)
  {
    case CHOOSE_TASK:
    {
      // updateAllSonars();

      if (pathIndex == pathLength)
      {
        programPhase = CONTINUE_MAZE;
      }
      else
      {
        currentTask = pathMapped[pathIndex++];
        startTask(currentTask);
      }
    } break;

    case PATH_FINISHED:
    {
      programPhase = FOLLOW_LINE_END;
    } break;
  }

  updateCurrentTask();
}



///////////////////////////
// PHASE - CONTINUE_MAZE //
///////////////////////////

void phase_continueMaze()
{
  checkLineSensorForFinish();
  if (finishFound)
  {
    programPhase = FOLLOW_LINE_END;
    return;
  }
  switch (currentTask)
  {
    case CHOOSE_TASK:
    {
      if (maxDistanceRight > 20)
      {
        startTask(TURN_RIGHT_90);
      }
      else if (maxDistanceFront > 25)
      {
        startTask(DRIVE_FORWARDS_30);
      }
      else if (maxDistanceLeft > 20)
      {
        startTask(TURN_LEFT_90);
      }
      else
      {
        turnAround();
      }
    } break;
  }
  setGripper(GRIPPER_CLOSED);
  updateCurrentTask();
}



/////////////////////////////
// PHASE - FOLLOW_LINE_END //
/////////////////////////////

void phase_followLineEnd()
{
  bool forwards =  (lineSensorValue[2] >= colorBlack) && (lineSensorValue[3] >= colorBlack);
  bool turnLeft =  (lineSensorValue[0] >= colorBlack) || (lineSensorValue[1] >= colorBlack);
  bool turnRight = (lineSensorValue[4] >= colorBlack) || (lineSensorValue[5] >= colorBlack);
  unsigned char lastValue = 0;
  static unsigned long timer = currentTime;
  greenLights();

  if ((currentTime - timer) > 1000)
  {
      if (!allBlack)
      {
        if (!allWhite)
        {
          if (forwards == true)
          {
            drive(FORWARDS);
          }
          else if (turnRight == true)
          {
            drive(ADJUST_RIGHT);
            lastValue = 1;
          }
          else if (turnLeft == true)
          {
            drive(ADJUST_LEFT);
            lastValue = 2;
          }
        }
        else
        {
          if (lastValue == 1)
          {
            drive(ADJUST_RIGHT_HARD);
          }
          else
          {
            drive(ADJUST_LEFT_HARD);
          }
        }
      }
      else
      {
        setGripper(GRIPPER_OPEN);
        drive(STOP);
        programPhase = FINISH;
      }
    timer = currentTime;
  }
}



////////////////////
// PHASE - FINISH //
////////////////////

void phase_finish()
{
  blueLights();
}



///////////
// DEBUG //
///////////

void printDebug(String message)
{
  #ifdef DEBUG
    Serial.println(message);
  #endif
}

void printDebug(const char* prefix, bool printDetails)
{
  #ifdef DEBUG
    if (printDetails) Serial.println(String(prefix)
        + "motor=(" + leftSpeed + "," + rightSpeed + "), "
        + "sonar=(" + distanceLeft + ", " + distanceFront + ", " + distanceRight + "), "
        + "rotations=(" + leftRotationTicks + ", " + rightRotationTicks + ")");
    else Serial.println(prefix);
  #endif
}

String translateTask(unsigned char task)
{
  switch (task)
  {
    case CHOOSE_TASK: return String("CHOOSE_TASK");
    case DRIVE_FORWARDS_30: return String("DRIVE_FORWARDS_30");
    case DRIVE_BACKWARDS_30: return String("DRIVE_BACKWARDS_30");
    case TURN_LEFT_90: return String("TURN_LEFT_90");
    case TURN_RIGHT_90: return String("TURN_RIGHT_90");
    case SHORT_LEFT_90: return String("SHORT_LEFT_90");
    case SHORT_RIGHT_90: return String("SHORT_RIGHT_90");
    case ROTATE_LEFT_180: return String("ROTATE_LEFT_180");
    case ROTATE_RIGHT_180: return String("ROTATE_RIGHT_180");
    case TURN_AROUND_LEFT: return String("TURN_AROUND_LEFT");
    case TURN_AROUND_RIGHT: return String("TURN_AROUND_RIGHT");
    case ADJUST_ROTATION: return String("ADJUST_ROTATION");
    case ADJUST_POSITION: return String("ADJUST_POSITION");
    case UPDATE_SONAR: return String("UPDATE_SONAR");
    case PAUSE: return String("PAUSE");
    case PATH_FINISHED: return String("PATH_FINISHED");
    case GIVE_UP: return String("GIVE_UP");
  }

  return String("UNKNOWN");
}

unsigned long pauseTimestamp = 0;
int pausedLeftSpeed = 0;
int pausedRightSpeed = 0;
void pause()
{
  pauseMotors();
  pauseTimestamp = millis();
}
void pauseMotors()
{
  pausedLeftSpeed = leftSpeed;
  pausedRightSpeed = rightSpeed;
  setMotors(0, 0);
}

void unpause()
{
  unsigned long pauseDuration = millis() - pauseTimestamp;
  unpauseMotors();
  currentTaskDuration += pauseDuration;
}
void unpauseMotors()
{
  setMotors(pausedLeftSpeed, pausedRightSpeed);
  pausedLeftSpeed = pausedRightSpeed = 0;
}



///////////////
// NEOPIXELS //
///////////////

#define BACK_LEFT     0
#define BACK_RIGHT    1
#define FORWARD_RIGHT 2
#define FORWARD_LEFT  3

void idleLights()
{
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
  static unsigned long timer = millis();

  if ((currentTime - timer) > 500)
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

void discoLights()
{
  static unsigned long timer = micros();

  if ((currentTime - timer) > 100000)
  {
    for (char i = 0; i < 4; i++)
    {
      int randomRed = random(0, 200);
      int randomGreen = random(0, 200);
      int randomBlue = random(0, 200);

      switch (i)
      {
        case 0:
          pixels.setPixelColor(BACK_LEFT, pixels.Color(randomRed, randomGreen, randomBlue));
          break;
        case 1:
          pixels.setPixelColor(BACK_RIGHT, pixels.Color(randomRed, randomGreen, randomBlue));
          break;
        case 2:
          pixels.setPixelColor(FORWARD_LEFT, pixels.Color(randomRed, randomGreen, randomBlue));
          break;
        case 3:
          pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(randomRed, randomGreen, randomBlue));
          break;
      }
    }

    pixels.show();
    timer = currentTime;
  }
}

void blueLights()
{
    pixels.setPixelColor(BACK_LEFT, pixels.Color(0, 0, 255));
    pixels.setPixelColor(BACK_RIGHT, pixels.Color(0, 0, 255));
    pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(0, 0, 255));
    pixels.setPixelColor(FORWARD_LEFT, pixels.Color(0, 0, 255));
    pixels.show();
}


void greenLights()
{
    pixels.setPixelColor(BACK_LEFT, pixels.Color(0, 255, 0));
    pixels.setPixelColor(BACK_RIGHT, pixels.Color(0, 255, 0));
    pixels.setPixelColor(FORWARD_RIGHT, pixels.Color(0, 255, 0));
    pixels.setPixelColor(FORWARD_LEFT, pixels.Color(0, 255, 0));
    pixels.show();
}
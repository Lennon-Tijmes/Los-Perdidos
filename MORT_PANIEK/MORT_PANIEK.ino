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



// #define DEBUG 1
// #define DEBUG2 1
// #define DEBUG3 1
// #define DEBUG4 1
// #define DEBUG5 1
// #define START_AT_MAZE 1
#define WAIT_FOR_SIGNAL 1
#define NEW_INTERRUPTS_HANDLING 1
#define STARTUP_DELAY 2000



// Pins
const unsigned char LINE_SENSOR_PINS[] = { A2, A3, A4, A5, A6, A7 };
#define SONAR_LEFT_TRIG_PIN 5
#define SONAR_LEFT_ECHO_PIN 6
#define SONAR_FRONT_TRIG_PIN 12
#define SONAR_FRONT_ECHO_PIN 13
#define SONAR_RIGHT_TRIG_PIN A1  // 15
#define SONAR_RIGHT_ECHO_PIN A0  // 14
#define ROTATION_SENSOR_LEFT_PIN 2
#define ROTATION_SENSOR_RIGHT_PIN 3
#define MOTOR_LEFT_POWER_PIN 9
#define MOTOR_RIGHT_POWER_PIN 10
#define MOTOR_LEFT_MODE_PIN 7
#define MOTOR_RIGHT_MODE_PIN 8
#define GRIPPER_PIN 4
#define PIXELS_PIN 11

// Phase
#define PHASE_STARTUP_WAIT      100       // wait a bit just in case
#define PHASE_WAIT_FOR_SIGNAL   101    // wait for start signal
#define PHASE_DRIVE_TO_SQUARE   102    // drive forward till black square, grab the pin and turn left
#define PHASE_FOLLOW_LINE_START 103  // follow the line until inside of the maze
#define PHASE_DRIVE_MAZE        104         // solve the maze
#define PHASE_FOLLOW_LINE_END   105    // follow the finishing line and put pin inside black square
#define PHASE_FINISH            106             // stop driving
unsigned char programPhase;
unsigned long programPhaseStartTime;
unsigned long programStartTime;

// Motors
#define STOP              200
#define FORWARDS          201
#define BACKWARDS         202
#define SMOOTH_LEFT       203
#define SMOOTH_RIGHT      204
#define LEFT              205
#define RIGHT             206
#define LEFT_BACK         207
#define RIGHT_BACK        208
#define ROTATE_LEFT       209
#define ROTATE_RIGHT      210
#define ADJUST_LEFT       211
#define ADJUST_RIGHT      212
#define ADJUST_LEFT_HARD  213
#define ADJUST_RIGHT_HARD 214
int leftSpeed = 250;
int rightSpeed = 255;

// Rotation sensor
unsigned int leftRotationTicks = 0;
unsigned int rightRotationTicks = 0;

// Line sensor
bool allBlack = false;
bool allWhite = false;
unsigned int colorBlack = 801;
unsigned int colorWhite = 799;

// Signal receiver
#define SLAVE_ID 1

// LEDs
#include <Adafruit_NeoPixel.h>  // neopixel library
#define NUM_PIXELS 4            // number of neopixels
Adafruit_NeoPixel pixels(NUM_PIXELS, PIXELS_PIN, NEO_RGB + NEO_KHZ800);

#define OFF    300
#define WHITE  301
#define RED    302
#define ORANGE 303
#define YELLOW 304
#define GREEN  305
#define BLUE   306
#define PURPLE 307

#define LED_BACK_LEFT 0
#define LED_BACK_RIGHT 1
#define LED_FRONT_RIGHT 2
#define LED_FRONT_LEFT 3

// Main functions
void setup()
{
  Serial.begin(9600);                                                // Begin the serial monitor
  pixels.begin();                                                    // Initialize the pixels thing
  Serial.println("Slave started");                                   // Send message to base
  pinMode(SONAR_LEFT_TRIG_PIN, OUTPUT);                              // Initialize the left  sonar trig pin as output
  pinMode(SONAR_LEFT_ECHO_PIN, INPUT);                               // Initialize the left  sonar echo pin as input
  pinMode(SONAR_FRONT_TRIG_PIN, OUTPUT);                             // Initialize the front sonar trig pin as output
  pinMode(SONAR_FRONT_ECHO_PIN, INPUT);                              // Initialize the front sonar echo pin as input
  pinMode(SONAR_RIGHT_TRIG_PIN, OUTPUT);                             // Initialize the right sonar trig pin as output
  pinMode(SONAR_RIGHT_ECHO_PIN, INPUT);                              // Initialize the right sonar echo pin as input
  pinMode(ROTATION_SENSOR_LEFT_PIN, INPUT_PULLUP);                   // Initialize the left  rotation sensor as pullup input
  pinMode(ROTATION_SENSOR_RIGHT_PIN, INPUT_PULLUP);                  // Initialize the right rotation sensor as pullup input
  pinMode(MOTOR_LEFT_POWER_PIN, OUTPUT);                             // Initialize the left  motor power pin as output
  pinMode(MOTOR_RIGHT_POWER_PIN, OUTPUT);                            // Initialize the right motor power pin as output
  pinMode(MOTOR_LEFT_MODE_PIN, OUTPUT);                              // Initialize the left  motor mode  pin  as output
  pinMode(MOTOR_RIGHT_MODE_PIN, OUTPUT);                             // Initialize the right motor mode  pin  as output
  pinMode(GRIPPER_PIN, OUTPUT);                                      // Initialize the gripper pin as output
  for (char i = 0; i < 6; i++) pinMode(LINE_SENSOR_PINS[i], INPUT);  // Initialize the line sensor pins as input

  attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_LEFT_PIN), countRotationsLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_RIGHT_PIN), countRotationsRight, RISING);

  programStartTime = millis();
  changePhase(PHASE_STARTUP_WAIT);
  lights(WHITE);
}

void loop()
{
  updateSonar();
  updateLineSensor();

  switch (programPhase)
  {
    case PHASE_STARTUP_WAIT: phase_startupWait(); break;
    case PHASE_WAIT_FOR_SIGNAL: phase_waitForSignal(); break;
    case PHASE_DRIVE_TO_SQUARE: phase_driveToSquare(); break;
    case PHASE_FOLLOW_LINE_START: phase_followLineStart(); break;
    case PHASE_DRIVE_MAZE: phase_driveMaze(); break;
    case PHASE_FOLLOW_LINE_END: phase_followLineEnd(); break;
    case PHASE_FINISH: phase_finish(); break;
  }

  // updateGripper();
  // updateLights();
}

void changePhase(unsigned char phase)
{
  programPhase = phase;
  programPhaseStartTime = millis();
}



/////////////////////
// ROTATION SENSOR //
/////////////////////

#define DEBOUNCE_TIME_MS 30;  // Debounce time for more accurate rotation sensor reading, at max robot speed this should tick every 40ms

// Counts the interrupts of the rotation sensor for the left wheel
void countRotationsLeft()
{
  static unsigned long timer;
  #ifdef NEW_INTERRUPTS_HANDLING
    detachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_LEFT_PIN));
  #else
    noInterrupts();
  #endif

  if (millis() > timer)
  {
    leftRotationTicks++;
    timer = millis() + DEBOUNCE_TIME_MS;
  }

  #ifdef NEW_INTERRUPTS_HANDLING
    attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_LEFT_PIN), countRotationsLeft, RISING);
  #else
    interrupts();
  #endif
}

// Counts the interrupts of the rotation sensor for the right wheel
void countRotationsRight()
{
  static unsigned long timer;
  #ifdef NEW_INTERRUPTS_HANDLING
    detachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_RIGHT_PIN));
  #else
    noInterrupts();
  #endif

  if (millis() > timer)
  {
    rightRotationTicks++;
    timer = millis() + DEBOUNCE_TIME_MS;
  }

  #ifdef NEW_INTERRUPTS_HANDLING
    attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_RIGHT_PIN), countRotationsRight, RISING);
  #else
    interrupts();
  #endif
}



/////////////////
// LINE SENSOR //
/////////////////

int lineSensorValue[6] = { 0, 0, 0, 0, 0, 0 };

// Read all the line sensor pins
void updateLineSensor()
{
  for (unsigned char i = 0; i < 6; i++)
  {
    lineSensorValue[i] = analogRead(LINE_SENSOR_PINS[i]);
  }

  allBlack = (lineSensorValue[0] >= colorBlack)
             && (lineSensorValue[1] >= colorBlack)
             && (lineSensorValue[2] >= colorBlack)
             && (lineSensorValue[3] >= colorBlack)
             && (lineSensorValue[4] >= colorBlack)
             && (lineSensorValue[5] >= colorBlack);  // true if all the line sensor bits are looking at black

  allWhite = (lineSensorValue[0] <= colorWhite)
             && (lineSensorValue[1] <= colorWhite)
             && (lineSensorValue[2] <= colorWhite)
             && (lineSensorValue[3] <= colorWhite)
             && (lineSensorValue[4] <= colorWhite)
             && (lineSensorValue[5] <= colorWhite);  // true if all the line sensor bits are looking at white
}



///////////
// SONAR //
///////////

// The HC-SR04 manual recommends sending a 10 microsecond pulse to TRIG pin
#define SONAR_SIGNAL_DURATION_US 10

// Minimum delay before switching to next sonar
#define SONAR_DELAY_MS 10

// We assume the maze is 7x7, which means the longest distance to measure should be 7*30cm = 210cm
// Speed of sound is around 35cm/ms=210cm/6ms, so the sound signal should take at most 12ms to come back to the robot
// Accounting for some inaccuracies it's safe to set the timeout to 15ms
#define SONAR_RECEIVER_TIMEOUT_MS 15

// Sonar phases
#define SONAR_LEFT_SEND_SIGNAL 0
#define SONAR_LEFT_READ_SIGNAL 1
#define SONAR_FRONT_SEND_SIGNAL 2
#define SONAR_FRONT_READ_SIGNAL 3
#define SONAR_RIGHT_SEND_SIGNAL 4
#define SONAR_RIGHT_READ_SIGNAL 5
#define SONAR_UPDATE_DISTANCES 6

const double microsecondsToCentimeters = 0.017;  // 340m/s = 34000cm/s = 34cm/ms = 0.034cm/us, sound travels to object and back so divide by 2 for distance to object

#define SONAR_TOO_FAR 1023
#define SONAR_NO_READING 1024
double distanceLeft = SONAR_NO_READING;
double distanceFront = SONAR_NO_READING;
double distanceRight = SONAR_NO_READING;
bool sonarsStuck = false;

void updateSonar()
{
  static unsigned char sonarPhase = 0;
  static unsigned long sonarLastActionTime = 0;
  static unsigned long sonarSignalDuration = 0;
  static double currentDistanceLeft = SONAR_NO_READING;
  static double currentDistanceFront = SONAR_NO_READING;
  static double currentDistanceRight = SONAR_NO_READING;

  switch (sonarPhase)
  {
    // Left sonar

    // Send pulse on left sonar
    case SONAR_LEFT_SEND_SIGNAL:
      if (millis() - sonarLastActionTime > SONAR_DELAY_MS)
      {
        digitalWrite(SONAR_LEFT_TRIG_PIN, HIGH);
        delayMicroseconds(SONAR_SIGNAL_DURATION_US);
        digitalWrite(SONAR_LEFT_TRIG_PIN, LOW);
        sonarLastActionTime = millis();
        sonarPhase = SONAR_LEFT_READ_SIGNAL;
      }
      break;

    // Get HIGH state duration from ECHO pin, or move to next phase if no signal after SONAR_RECEIVER_TIMEOUT_MS milliseconds
    case SONAR_LEFT_READ_SIGNAL:
      sonarSignalDuration = pulseIn(SONAR_LEFT_ECHO_PIN, HIGH);
      if (sonarSignalDuration > 0)
      {
        if (sonarSignalDuration > 12353) currentDistanceLeft = SONAR_TOO_FAR;  // if more than 210cm
        else currentDistanceLeft = (double)sonarSignalDuration * microsecondsToCentimeters;

        sonarSignalDuration = 0;
        sonarLastActionTime = millis();
        sonarPhase = SONAR_FRONT_SEND_SIGNAL;
      }
      else if (millis() - sonarLastActionTime > SONAR_RECEIVER_TIMEOUT_MS)
      {
        currentDistanceLeft = SONAR_NO_READING;
        sonarLastActionTime = millis();
        sonarPhase = SONAR_FRONT_SEND_SIGNAL;
      }
      break;

    // Front sonar

    // Send pulse on front sonar
    case SONAR_FRONT_SEND_SIGNAL:
      if (millis() - sonarLastActionTime > SONAR_DELAY_MS)
      {
        digitalWrite(SONAR_FRONT_TRIG_PIN, HIGH);
        delayMicroseconds(SONAR_SIGNAL_DURATION_US);
        digitalWrite(SONAR_FRONT_TRIG_PIN, LOW);
        sonarLastActionTime = millis();
        sonarPhase = SONAR_FRONT_READ_SIGNAL;
      }
      break;

    // Get HIGH state duration from ECHO pin, or move to next phase if no signal after SONAR_RECEIVER_TIMEOUT_MS milliseconds
    case SONAR_FRONT_READ_SIGNAL:
      sonarSignalDuration = pulseIn(SONAR_FRONT_ECHO_PIN, HIGH);
      if (sonarSignalDuration > 0)
      {
        if (sonarSignalDuration > 12353) currentDistanceFront = SONAR_TOO_FAR;  // if more than 210cm
        else currentDistanceFront = (double)sonarSignalDuration * microsecondsToCentimeters;

        sonarSignalDuration = 0;
        sonarLastActionTime = millis();
        sonarPhase = SONAR_RIGHT_SEND_SIGNAL;
      }
      else if (millis() - sonarLastActionTime > SONAR_RECEIVER_TIMEOUT_MS)
      {
        currentDistanceFront = SONAR_NO_READING;
        sonarLastActionTime = millis();
        sonarPhase = SONAR_RIGHT_SEND_SIGNAL;
      }
      break;

    // Right sonar

    // Send pulse on right sonar
    case SONAR_RIGHT_SEND_SIGNAL:
      if (millis() - sonarLastActionTime > SONAR_DELAY_MS)
      {
        digitalWrite(SONAR_RIGHT_TRIG_PIN, HIGH);
        delayMicroseconds(SONAR_SIGNAL_DURATION_US);
        digitalWrite(SONAR_RIGHT_TRIG_PIN, LOW);
        sonarLastActionTime = millis();
        sonarPhase = SONAR_RIGHT_READ_SIGNAL;
      }
      break;

    // Get HIGH state duration from ECHO pin, or move to next phase if no signal after SONAR_RECEIVER_TIMEOUT_MS milliseconds
    case SONAR_RIGHT_READ_SIGNAL:
      sonarSignalDuration = pulseIn(SONAR_RIGHT_ECHO_PIN, HIGH);
      if (sonarSignalDuration > 0)
      {
        if (sonarSignalDuration > 12353) currentDistanceRight = SONAR_TOO_FAR;  // if more than 210cm
        else currentDistanceRight = (double)sonarSignalDuration * microsecondsToCentimeters;

        sonarSignalDuration = 0;
        sonarLastActionTime = millis();
        sonarPhase = SONAR_UPDATE_DISTANCES;
      }
      else if (millis() - sonarLastActionTime > SONAR_RECEIVER_TIMEOUT_MS)
      {
        currentDistanceRight = SONAR_NO_READING;
        sonarLastActionTime = millis();
        sonarPhase = SONAR_UPDATE_DISTANCES;
      }
      break;

    case SONAR_UPDATE_DISTANCES:
      sonarsStuck = distanceLeft == currentDistanceLeft && distanceFront == currentDistanceFront && distanceRight == currentDistanceRight;
      distanceLeft = currentDistanceLeft;
      distanceFront = currentDistanceFront;
      distanceRight = currentDistanceRight;
      sonarPhase = SONAR_LEFT_SEND_SIGNAL;
      break;
  }
}



///////////
// MOTOR //
///////////

#define WHEEL_INERTIA_DELAY_MS 2

#define FORWARDS_MODE LOW
#define BACKWARDS_MODE HIGH

void setMotors(int leftMotorSpeed, int rightMotorSpeed)
{
  if (leftMotorSpeed >= 0)
  {
    digitalWrite(MOTOR_LEFT_MODE_PIN, FORWARDS_MODE);
    analogWrite(MOTOR_LEFT_POWER_PIN, 255);
    delay(WHEEL_INERTIA_DELAY_MS);
    analogWrite(MOTOR_LEFT_POWER_PIN, leftMotorSpeed);
  }
  else /* leftMotorSpeed < 0 */
  {
    digitalWrite(MOTOR_LEFT_MODE_PIN, BACKWARDS_MODE);
    analogWrite(MOTOR_LEFT_POWER_PIN, 0);
    delay(WHEEL_INERTIA_DELAY_MS);
    analogWrite(MOTOR_LEFT_POWER_PIN, 255 + leftMotorSpeed);
  }

  if (rightMotorSpeed >= 0)
  {
    digitalWrite(MOTOR_RIGHT_MODE_PIN, FORWARDS_MODE);
    analogWrite(MOTOR_RIGHT_POWER_PIN, 255);
    delay(WHEEL_INERTIA_DELAY_MS);
    analogWrite(MOTOR_RIGHT_POWER_PIN, rightMotorSpeed);
  }
  else /* rightMotorSpeed < 0 */
  {
    digitalWrite(MOTOR_RIGHT_MODE_PIN, BACKWARDS_MODE);
    analogWrite(MOTOR_RIGHT_POWER_PIN, 0);
    delay(WHEEL_INERTIA_DELAY_MS);
    analogWrite(MOTOR_RIGHT_POWER_PIN, 255 + rightMotorSpeed);
  }
}

unsigned char lastMode = 255;
void drive(unsigned char mode)
{
  if (mode == lastMode) return;

  switch (mode)
  {
    case STOP: setMotors(0, 0); break;
    case FORWARDS: setMotors(leftSpeed, rightSpeed); break;
    case BACKWARDS: setMotors(-leftSpeed, -rightSpeed); break;

    case SMOOTH_LEFT: setMotors(150, rightSpeed); break;
    case SMOOTH_RIGHT: setMotors(leftSpeed, 150); break;
    case LEFT: setMotors(100, rightSpeed); break;
    case RIGHT: setMotors(leftSpeed, 100); break;
    case LEFT_BACK: setMotors(-100, -rightSpeed); break;
    case RIGHT_BACK: setMotors(-leftSpeed, -100); break;

    case ROTATE_LEFT: setMotors(-leftSpeed, rightSpeed); break;
    case ROTATE_RIGHT: setMotors(leftSpeed, -rightSpeed); break;

    case ADJUST_LEFT: setMotors(160, 240); break;
    case ADJUST_RIGHT: setMotors(240, 165); break;
    case ADJUST_LEFT_HARD: setMotors(-190, 240); break;
    case ADJUST_RIGHT_HARD: setMotors(240, -190); break;
  }

  lastMode = mode;
}



/////////////
// GRIPPER //
/////////////

#define GRIPPER_OPEN 1600   // Value for gripper being open
#define GRIPPER_CLOSED 1010 // Value for gripper being closed
unsigned long gripperState = GRIPPER_CLOSED;

// Sets the gripper position to the given pulse
void setGripper_old(unsigned int pulse)
{
  for (unsigned char i = 0; i < 8; i++)
  {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(GRIPPER_PIN, LOW);
  }
}

void setGripper(unsigned long openClose)
{
  gripperState = openClose;
}


void updateGripper()
{
  static unsigned long timer = micros();

  if (micros() - timer >= gripperState)
  {
    digitalWrite(GRIPPER_PIN, LOW);
    digitalWrite(GRIPPER_PIN, HIGH);
    timer = micros();
  }
}



////////////////////////
// PHASE_STARTUP_WAIT //
////////////////////////

void phase_startupWait()
{
  if (startupDelay())
  {
    #ifdef START_AT_MAZE
      changePhase(PHASE_DRIVE_MAZE);
    #else
      changePhase(PHASE_WAIT_FOR_SIGNAL);
    #endif
  }
}
bool startupDelay()
{
  return millis() - programStartTime > STARTUP_DELAY;
}



///////////////////////////
// PHASE_WAIT_FOR_SIGNAL //
///////////////////////////

void phase_waitForSignal()
{
  #ifndef WAIT_FOR_SIGNAL
    changePhase(PHASE_DRIVE_TO_SQUARE);
    return;
  #endif

  if (hasReceivedSignal())
  {
    changePhase(PHASE_DRIVE_TO_SQUARE);
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



///////////////////////////
// PHASE_DRIVE_TO_SQUARE //
///////////////////////////

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
    setGripper_old(GRIPPER_CLOSED);
    drive(ROTATE_LEFT);
    delay(460);
    drive(FORWARDS);
    delay(500);
    changePhase(PHASE_FOLLOW_LINE_START);
  }
}



/////////////////////////////
// PHASE_FOLLOW_LINE_START //
/////////////////////////////

void phase_followLineStart()
{
  static unsigned char lastDirection = STOP;

  updateLineSensor();
  bool turnLeft =   (lineSensorValue[0] >= colorBlack) || (lineSensorValue[1] >= colorBlack);
  bool goForwards = (lineSensorValue[2] >= colorBlack) || (lineSensorValue[3] >= colorBlack);
  bool turnRight =  (lineSensorValue[4] >= colorBlack) || (lineSensorValue[5] >= colorBlack);

  if (allWhite)
    if (lastDirection == LEFT) drive(ADJUST_LEFT);
    else drive(ADJUST_RIGHT);
  else
  {
    if (goForwards)
    {
      drive(FORWARDS);
    }
    else if (turnRight)
    {
      drive(ADJUST_RIGHT);
      lastDirection = RIGHT;
    }
    else if (turnLeft)
    {
      drive(ADJUST_LEFT);
      lastDirection = LEFT;
    }
  }

  updateSonar();

  if (allWhite && distanceLeft < 30 && distanceFront < 40 && distanceRight < 30)
  {
    changePhase(PHASE_DRIVE_MAZE);
  }
  if (millis() - programPhaseStartTime > 3000)
  {
    changePhase(PHASE_DRIVE_MAZE);
  }
}



//////////////////////
// PHASE_DRIVE_MAZE //
//////////////////////

#define CHECK_FINISH_LINE_DELAY_MS 10000

unsigned long activeTaskTimer = millis();

void phase_driveMaze()
{
  static unsigned char finishLineCounter = 0;
  static unsigned long stuckTimer = millis();
  static unsigned int stuckRotations = 0;
  static unsigned char stuckCounter = 0;
  static unsigned char previousTask = STOP;
  previousTask = lastMode;

  if (sonarsStuck)
  {
    doTask(BACKWARDS, 100);
    sonarsStuck = false;
  }

  // If a task is ongoing, let it continue
  if (millis() < activeTaskTimer) return;

  // If a sonar failed, wait until it's fixed
  if (distanceLeft >= SONAR_NO_READING || distanceFront >= SONAR_NO_READING || distanceRight >= SONAR_NO_READING)
  {
    doTask(STOP, 50);
    lights(PURPLE);
    return;
  }

  if (distanceLeft == SONAR_TOO_FAR && distanceRight == SONAR_TOO_FAR)
  {
    doTask(STOP, 100);
    hazardLights(PURPLE);
    return;
  }

  // robot AI
  if (distanceFront < 10)
    if (distanceLeft < 15 && distanceRight < 15) turnAround();
    else doTask(BACKWARDS, 150);
  else if (distanceLeft > 25) doTask(LEFT, 500);
  else if (distanceFront > 25) drive(FORWARDS);
  else if (distanceRight > 20) doTask(RIGHT, 500);
  else if (distanceLeft < 4) doTask(RIGHT, 100);
  else if (distanceRight < 4) doTask(LEFT, 100);
  else if (distanceFront > 10) drive(FORWARDS);
  else turnAround();

  // stuck detection - if wheels not spinning go backwards
  if (millis() - stuckTimer > 200)
  {
    if (leftRotationTicks + rightRotationTicks <= stuckRotations + 5) doTask(BACKWARDS, 200);
    else stuckRotations = leftRotationTicks + rightRotationTicks;

    stuckTimer = millis();
  }

  // if robot is stuck in forwards-backwards loop
  if (lastMode != previousTask)
    if (lastMode == FORWARDS && previousTask == BACKWARDS || lastMode == BACKWARDS && previousTask == FORWARDS) stuckCounter++;
    else stuckCounter = 0;
  if (stuckCounter > 6) doTask(LEFT, 100);
}

void doTask(unsigned char task, unsigned long duration)
{
  drive(task);
  activeTaskTimer = millis() + duration;
}

void turnAround()
{
  if (distanceRight > distanceLeft)
    if (distanceLeft > 5) doTask(ROTATE_RIGHT, 750);
    else doTask(ROTATE_RIGHT, 750);
  else
    if (distanceRight > 5) doTask(ROTATE_LEFT, 750);
    else doTask(ROTATE_LEFT, 750);
}

bool numbersAreClose(double a, double b, double diff)
{
  return abs(a - b) < diff;
}



///////////////////////////
// PHASE_FOLLOW_LINE_END //
///////////////////////////

void phase_followLineEnd()
{
  static unsigned long timer = millis();
  static unsigned char lastDirection = LEFT;

  updateLineSensor();
  bool turnLeft =  (lineSensorValue[0] > colorBlack) || (lineSensorValue[1] > colorBlack);
  bool forwards =  (lineSensorValue[2] > colorBlack) || (lineSensorValue[3] > colorBlack);
  bool turnRight = (lineSensorValue[4] > colorBlack) || (lineSensorValue[5] > colorBlack);

  if (millis() - timer > 500)
  {
    if (!allBlack)
    {
      if (!allWhite)
      {
        if (forwards)
        {
          drive(FORWARDS);
        }
        else if (turnRight)
        {
          drive(ADJUST_RIGHT);
          lastDirection = RIGHT;
        }
        else if (turnLeft)
        {
          drive(ADJUST_LEFT);
          lastDirection = LEFT;
        }
      }
      else
      {
        if (lastDirection == RIGHT)
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
      changePhase(PHASE_FINISH);
    }

    timer = millis();
  }
}



//////////////////
// PHASE_FINISH //
//////////////////

void phase_finish()
{
  discoLights();
}



///////////
// DEBUG //
///////////

void printDebug(String message)
{
  #ifdef DEBUG
    Serial.println(String("[") + millis() + "] " + message);
  #endif
}

void printDebug2(String message)
{
  #ifdef DEBUG2
    Serial.println(String("[") + millis() + "] " + message);
  #endif
}

void printDebug3(String message)
{
  #ifdef DEBUG3
    Serial.println(String("[") + millis() + "] " + message);
  #endif
}

void printDebug4(String message)
{
  #ifdef DEBUG4
    Serial.println(String("[") + millis() + "] " + message);
  #endif
}

void printDebug5(String message)
{
  #ifdef DEBUG5
    Serial.println(String("[") + millis() + "] " + message);
  #endif
}



///////////////
// NEOPIXELS //
///////////////

#define BLINK_DELAY_MS 250

void updateLights()
{
  switch (lastMode)
  {
    case STOP: brakeLights(); break;
    case FORWARDS: lights(WHITE); break;
    case BACKWARDS: backwardsLights(RED); break;

    case LEFT: leftBlinker(ORANGE); break;
    case RIGHT: rightBlinker(ORANGE); break;
    case LEFT_BACK: backLeftBlinker(ORANGE); break;
    case RIGHT_BACK: backRightBlinker(ORANGE); break;

    case ROTATE_LEFT: leftBlinker(YELLOW); break;
    case ROTATE_RIGHT: rightBlinker(YELLOW); break;

    case ADJUST_LEFT: light(LED_FRONT_LEFT, ORANGE); break;
    case ADJUST_RIGHT: light(LED_FRONT_RIGHT, ORANGE); break;
    case ADJUST_LEFT_HARD: light(LED_FRONT_LEFT, RED); break;
    case ADJUST_RIGHT_HARD: light(LED_FRONT_RIGHT, RED); break;
  }
}

void lights(unsigned int color)
{
  switch (color)
  {
    case OFF: lights(0, 0, 0); break;
    case WHITE: lights(255, 255, 255); break;
    case RED: lights(255, 0, 0); break;
    case ORANGE: lights(0, 160, 16); break;
    case YELLOW: lights(0, 225, 32); break;
    case GREEN: lights(0, 255, 0); break;
    case BLUE: lights(0, 0, 255); break;
    case PURPLE: lights(160, 32, 255); break;
    default: lights(0, 0, 0);
  }
}

void light(unsigned int ledID, unsigned int color)
{
  switch (color)
  {
    case OFF: light(ledID, 0, 0, 0); break;
    case WHITE: light(ledID, 255, 255, 255); break;
    case RED: light(ledID, 255, 0, 0); break;
    case ORANGE: light(ledID, 135, 99, 24); break;
    case YELLOW: light(ledID, 0, 225, 32); break;
    case GREEN: light(ledID, 0, 255, 0); break;
    case BLUE: light(ledID, 0, 0, 255); break;
    case PURPLE: light(ledID, 160, 32, 255); break;
    default: light(ledID, 0, 0, 0);
  }
}

void lights(unsigned int red, unsigned int green, unsigned int blue)
{
  pixels.setPixelColor(LED_FRONT_LEFT, pixels.Color(red, green, blue));
  pixels.setPixelColor(LED_FRONT_RIGHT, pixels.Color(red, green, blue));
  pixels.setPixelColor(LED_BACK_LEFT, pixels.Color(red, green, blue));
  pixels.setPixelColor(LED_BACK_RIGHT, pixels.Color(red, green, blue));
  pixels.show();
}

void light(unsigned int ledID, unsigned int red, unsigned int green, unsigned int blue)
{
  pixels.setPixelColor(ledID, pixels.Color(red, green, blue));
  pixels.show();
}

void leftBlinker(unsigned int color)
{
  static bool lightOn = false;
  static unsigned long timer = millis();

  if (millis() - timer > BLINK_DELAY_MS)
  {
    if (lightOn)
    {
      light(LED_FRONT_LEFT, WHITE);
      light(LED_BACK_LEFT, WHITE);
    }
    else
    {
      light(LED_FRONT_LEFT, color);
      light(LED_BACK_LEFT, color);
    }

    lightOn = !lightOn;
    timer = millis();
  }
}

void rightBlinker(unsigned int color)
{
  static bool lightOn = false;
  static unsigned long timer = millis();

  if (millis() - timer > BLINK_DELAY_MS)
  {
    if (lightOn)
    {
      light(LED_FRONT_RIGHT, WHITE);
      light(LED_BACK_RIGHT, WHITE);
    }
    else
    {
      light(LED_FRONT_RIGHT, color);
      light(LED_BACK_RIGHT, color);
    }

    lightOn = !lightOn;
    timer = millis();
  }
}

void backLeftBlinker(unsigned int color)
{
  static bool lightOn = false;
  static unsigned long timer = millis();

  if (millis() - timer > BLINK_DELAY_MS)
  {
    if (lightOn)
    {
      light(LED_FRONT_LEFT, WHITE);
      light(LED_BACK_LEFT, RED);
    }
    else
    {
      light(LED_FRONT_LEFT, color);
      light(LED_BACK_LEFT, color);
    }

    lightOn = !lightOn;
    timer = millis();
  }
}

void backRightBlinker(unsigned int color)
{
  static bool lightOn = false;
  static unsigned long timer = millis();

  if (millis() - timer > BLINK_DELAY_MS)
  {
    if (lightOn)
    {
      light(LED_FRONT_RIGHT, WHITE);
      light(LED_BACK_RIGHT, RED);
    }
    else
    {
      light(LED_FRONT_RIGHT, color);
      light(LED_BACK_RIGHT, color);
    }

    lightOn = !lightOn;
    timer = millis();
  }
}

void brakeLights()
{
  light(LED_BACK_LEFT, RED);
  light(LED_BACK_RIGHT, RED);
}

void backwardsLights(unsigned int color)
{
  static bool lightOn = false;
  static unsigned long timer = millis();

  if (millis() - timer > BLINK_DELAY_MS)
  {
    if (lightOn)
    {
      light(LED_BACK_LEFT, WHITE);
      light(LED_BACK_RIGHT, WHITE);
    }
    else
    {
      light(LED_BACK_LEFT, color);
      light(LED_BACK_RIGHT, color);
    }

    lightOn = !lightOn;
    timer = millis();
  }
}

void hazardLights(unsigned int color)
{
  static bool lightOn = false;
  static unsigned long timer = millis();

  if (millis() - timer > BLINK_DELAY_MS)
  {
    if (lightOn)
    {
      lights(WHITE);
    }
    else
    {
      lights(color);
    }

    lightOn = !lightOn;
    timer = millis();
  }
}

void discoLights()
{
  static unsigned long timer = millis();

  if (millis() - timer > 100)
  {
    pixels.setPixelColor(LED_BACK_LEFT, pixels.Color(random(0, 255), random(0, 255), random(0, 255)));
    pixels.setPixelColor(LED_BACK_RIGHT, pixels.Color(random(0, 255), random(0, 255), random(0, 255)));
    pixels.setPixelColor(LED_FRONT_LEFT, pixels.Color(random(0, 255), random(0, 255), random(0, 255)));
    pixels.setPixelColor(LED_FRONT_RIGHT, pixels.Color(random(0, 255), random(0, 255), random(0, 255)));
    pixels.show();

    timer = millis();
  }
}
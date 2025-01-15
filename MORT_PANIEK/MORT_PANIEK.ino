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



// #define DEBUG 1                  // Enables all debug messages
// #define START_AT_MAZE 1          // Is robot starting inside the maze without barrel?
#define WAIT_FOR_SIGNAL 1           // Wait for start signal? if not, start driving immediately
#define NEW_INTERRUPTS_HANDLING 1   // Instead of disabling all interrupts during rotation sensor function, just disable one pin
//#define NEW_GRIPPER_HANDLING 1    // Sometimes the gripper gets forced open when robot drives into a wall, this keeps it closed (DOESN'T WORK)
//#define ENABLE_AUTOMATIC_LIGHTS 1 // Automatically update lights depending on what the robot is doing
#define STARTUP_DELAY 2000          // Wait a bit to let sonars and other things get warmed up



// Pins
const unsigned char LINE_SENSOR_PINS[] = { A2, A3, A4, A5, A6, A7 }; // Pin numbers for line sensor (pin 16 - pin 21)
#define SONAR_LEFT_TRIG_PIN       5  // Left  sonar TRIG pin
#define SONAR_LEFT_ECHO_PIN       6  // Left  sonar ECHO pin
#define SONAR_FRONT_TRIG_PIN      12 // Front sonar TRIG pin
#define SONAR_FRONT_ECHO_PIN      13 // Front sonar ECHO pin
#define SONAR_RIGHT_TRIG_PIN      A1 // Right sonar TRIG pin (pin 15)
#define SONAR_RIGHT_ECHO_PIN      A0 // Right sonar ECHO pin (pin 14)
#define ROTATION_SENSOR_LEFT_PIN  2  // Left  wheel rotation sensor interrupt pin
#define ROTATION_SENSOR_RIGHT_PIN 3  // Right wheel rotation sensor interrupt pin
#define MOTOR_LEFT_POWER_PIN      9  // Left  motor power pin
#define MOTOR_RIGHT_POWER_PIN     10 // Right motor power pin
#define MOTOR_LEFT_MODE_PIN       7  // Left  motor mode pin
#define MOTOR_RIGHT_MODE_PIN      8  // Right motor mode pin
#define GRIPPER_PIN               4  // Gripper control pin
#define PIXELS_PIN                11 // Pixels control pin

// Phase
#define PHASE_STARTUP_WAIT      100 // Wait a few seconds to let sonars and other things warm up
#define PHASE_WAIT_FOR_SIGNAL   101 // Wait for start signal
#define PHASE_DRIVE_TO_SQUARE   102 // Drive forward till black square, grab the pin and turn left
#define PHASE_FOLLOW_LINE_START 103 // Follow the line until inside of the maze
#define PHASE_DRIVE_MAZE        104 // Solve the maze
#define PHASE_FOLLOW_LINE_END   105 // Follow the finishing line and put pin inside black square
#define PHASE_FINISH            106 // Stop driving
unsigned char programPhase;         // Current phase of the program
unsigned long programPhaseStartTime;// Current phase start time
unsigned long programStartTime;     // Program start time

// Motors
#define STOP              200 // Stop both motors
#define FORWARDS          201 // Drive forwards, both motors full speed forwards
#define BACKWARDS         202 // Drive backwards, both motors full speed backwards
#define SMOOTH_LEFT       203 // Turn left slowly, right motor full speed, left motor 75%
#define SMOOTH_RIGHT      204 // Turn right slowly, left motor full speed, right motor 75%
#define LEFT              205 // Turn left, right motor full speed, left motor half speed
#define RIGHT             206 // Turn right, left motor full speed, right motor half speed
#define LEFT_BACK         207 // Turn left backwards, right motor full speed backwards, left motor half speed backwards
#define RIGHT_BACK        208 // Turn right backwards, left motor full speed backwards, right motor half speed backwards
#define ROTATE_LEFT       209 // Rotate in place anti-cloclwise, right motor full speed, left motor full speed backwards
#define ROTATE_RIGHT      210 // Rotate in place      cloclwise, left motor full speed, right motor full speed backwards
#define ADJUST_LEFT       211 // for line following
#define ADJUST_RIGHT      212 // for line following
#define ADJUST_LEFT_HARD  213 // for line following
#define ADJUST_RIGHT_HARD 214 // for line following
// Left and right motors are slightly different so to drive straight we need to set them at slightly different speeds
// This also changes randomly every robot restart, so we set the difference to 5 to drive most straight on average
int leftSpeed = 250;  // Left motor speed
int rightSpeed = 255; // Right motor speed

// Rotation sensor
unsigned int leftRotationTicks = 0;  // Left  wheel "rotation ticks" that rotation sensor counted, there's 20 in wheel's full rotation
unsigned int rightRotationTicks = 0; // Right wheel "rotation ticks" that rotation sensor counted, there's 20 in wheel's full rotation

// Line sensor
// This could be one variable but let's not fix what's not broken
unsigned int colorBlack = 801; // Line sensor value above this indicates black
unsigned int colorWhite = 799; // Line sensor value above this indicates white
bool allBlack = false; // True if all line sensor's values are above colorBlack
bool allWhite = false; // True if all line sensor's values are below colorWhite

// Signal receiver
#define SLAVE_ID 3 // Slave ID for master/slave communication

// NeoPixel
#include <Adafruit_NeoPixel.h> // Load NeoPixel library
#define NUM_PIXELS 4           // Number of LEDs
Adafruit_NeoPixel pixels(NUM_PIXELS, PIXELS_PIN, NEO_RGB + NEO_KHZ800); // Initialize NeoPixel

#define OFF    300 // Easy to use LED color
#define WHITE  301 // Easy to use LED color
#define RED    302 // Easy to use LED color
#define ORANGE 303 // Easy to use LED color
#define YELLOW 304 // Easy to use LED color
#define GREEN  305 // Easy to use LED color
#define BLUE   306 // Easy to use LED color
#define PURPLE 307 // Easy to use LED color

#define LED_BACK_LEFT   0 // Back left LED id
#define LED_BACK_RIGHT  1 // Back right LED id
#define LED_FRONT_RIGHT 2 // Front left LED id
#define LED_FRONT_LEFT  3 // Front right LED id

// Function to set up pins, interrupts, variables and everything else needed
void setup()
{
  Serial.begin(9600);                               // Begin the serial monitor
  pixels.begin();                                   // Initialize the pixels thing
  Serial.println("Slave started");                  // Send message to base
  pinMode(SONAR_LEFT_TRIG_PIN, OUTPUT);             // Initialize the left  sonar trig pin as output
  pinMode(SONAR_LEFT_ECHO_PIN, INPUT);              // Initialize the left  sonar echo pin as input
  pinMode(SONAR_FRONT_TRIG_PIN, OUTPUT);            // Initialize the front sonar trig pin as output
  pinMode(SONAR_FRONT_ECHO_PIN, INPUT);             // Initialize the front sonar echo pin as input
  pinMode(SONAR_RIGHT_TRIG_PIN, OUTPUT);            // Initialize the right sonar trig pin as output
  pinMode(SONAR_RIGHT_ECHO_PIN, INPUT);             // Initialize the right sonar echo pin as input
  pinMode(ROTATION_SENSOR_LEFT_PIN, INPUT_PULLUP);  // Initialize the left  rotation sensor as pullup input
  pinMode(ROTATION_SENSOR_RIGHT_PIN, INPUT_PULLUP); // Initialize the right rotation sensor as pullup input
  pinMode(MOTOR_LEFT_POWER_PIN, OUTPUT);            // Initialize the left  motor power pin as output
  pinMode(MOTOR_RIGHT_POWER_PIN, OUTPUT);           // Initialize the right motor power pin as output
  pinMode(MOTOR_LEFT_MODE_PIN, OUTPUT);             // Initialize the left  motor mode  pin  as output
  pinMode(MOTOR_RIGHT_MODE_PIN, OUTPUT);            // Initialize the right motor mode  pin  as output
  pinMode(GRIPPER_PIN, OUTPUT);                     // Initialize the gripper pin as output
  for (char i = 0; i < 6; i++)                      // Initialize the line sensor pins as input
  {
    pinMode(LINE_SENSOR_PINS[i], INPUT);
  }

  // Attach interrupts for both rotation sensors
  attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_LEFT_PIN), countRotationsLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ROTATION_SENSOR_RIGHT_PIN), countRotationsRight, RISING);

  programStartTime = millis();
  changePhase(PHASE_STARTUP_WAIT); // Start at phase PHASE_STARTUP_WAIT
  lights(WHITE); // Set lights to white
}

// The program's main loop, run many times per millisecond
void loop()
{
  updateSonar();      // Tick the sonar managing function, letting up update sonar readings whenever it decides to
  updateLineSensor(); // Update line sensor readings

  // Since the robot has multiple distinct tasks to do, they're split into "phases"
  // Depending on current phase saved in programPhase variable, appriopriate function is ticked
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

  // Sometimes the gripper gets forced open when robot drives into a wall, this keeps it closed
  // DOESN'T WORK - the gripper requires precise delay between impulses and this method cannot guarantee them
  #ifdef NEW_GRIPPER_HANDLING
    updateGripper();
  #endif
  // Automatically update lights depending on what the robot is doing
  // It seems to slow down the main loop quite a bit, making robot perform poorly
  #ifdef ENABLE_AUTOMATIC_LIGHTS
    updateLights();
  #endif
}

// Set program phase and note down phase start time which some phases use
void changePhase(unsigned char phase)
{
  programPhase = phase;
  programPhaseStartTime = millis();
}



/////////////////////
// ROTATION SENSOR //
/////////////////////

// Debounce time for more accurate rotation sensor reading, at max robot speed sensor should tick every 40ms
#define DEBOUNCE_TIME_MS 30;

// Handle the interrupts of the rotation sensor for the left wheel
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

// Handle the interrupts of the rotation sensor for the right wheel
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
// Could be optimized since lineSensorValue is unused but there was no need
void updateLineSensor()
{
  for (unsigned char i = 0; i < 6; i++)
  {
    lineSensorValue[i] = analogRead(LINE_SENSOR_PINS[i]);
  }

  // True if all the line sensor bits are looking at black
  allBlack =    (lineSensorValue[0] >= colorBlack)
             && (lineSensorValue[1] >= colorBlack)
             && (lineSensorValue[2] >= colorBlack)
             && (lineSensorValue[3] >= colorBlack)
             && (lineSensorValue[4] >= colorBlack)
             && (lineSensorValue[5] >= colorBlack);

  // True if all the line sensor bits are looking at white
  allWhite =    (lineSensorValue[0] <= colorWhite)
             && (lineSensorValue[1] <= colorWhite)
             && (lineSensorValue[2] <= colorWhite)
             && (lineSensorValue[3] <= colorWhite)
             && (lineSensorValue[4] <= colorWhite)
             && (lineSensorValue[5] <= colorWhite);
}



///////////
// SONAR //
///////////

// The HC-SR04 manual recommends sending a 10 microsecond pulse to TRIG pin
// Used in delayMicroseconds function - 10 microseconds is a short enough delay to not interfere with the rest of the program
// Using delayMicroseconds ensures that the sonar works correctly
#define SONAR_SIGNAL_DURATION_US 10

// Minimum delay before switching to next sonar, so signal from previous sonar doesn't interfere
#define SONAR_DELAY_MS 10

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
#define SONAR_UPDATE_DISTANCES  6

// Microseconds to cm converter using the speed of sound
// 340m/s = 34000cm/s = 34cm/ms = 0.034cm/us, sound travels to object and back so divide by 2 for distance to object
const double microsecondsToCentimeters = 0.017;

#define SONAR_TOO_FAR 1023    // distance reading for when sonar returns value above 210cm
#define SONAR_NO_READING 1024 // distance value for sonar timing out before signal can come back
double distanceLeft = SONAR_NO_READING;  // distance in cm from left sonar
double distanceFront = SONAR_NO_READING; // distance in cm from front sonar
double distanceRight = SONAR_NO_READING; // distance in cm from right sonar
bool sonarsStuck = false; // True when sonar readings haven't changed since last read, used for detecting whether robot is stuck

// Sonar managing function - uses sonars to gauge distance to a wall on left, front and right
// Rather than waiting for sonar's sound signal to come back,
//   the function quits and checks for result on next call - should be called every program loop
// Uses one sonar at a time to avoid polluting the reading from another sonars' signal
// After all 3 sonars are read, updates global distance variables
void updateSonar()
{
  static unsigned char sonarPhase = 0;          // Sonar manager's phase, makes sure only one sonar is used at a time
  static unsigned long sonarLastActionTime = 0; // Timestamp of when sonar sent or received a signal, depending on phase. Used for ensuring delay between actions that need it
  static unsigned long sonarSignalDuration = 0; // Variable to save sonar's reading in microseconds
  static double currentDistanceLeft = SONAR_NO_READING;  // Current sonar reading, befoe it's put into a global variable
  static double currentDistanceFront = SONAR_NO_READING; // Current sonar reading, befoe it's put into a global variable
  static double currentDistanceRight = SONAR_NO_READING; // Current sonar reading, befoe it's put into a global variable

  switch (sonarPhase)
  {
    // Left sonar

    // Send pulse on left sonar
    case SONAR_LEFT_SEND_SIGNAL:
      if (millis() - sonarLastActionTime > SONAR_DELAY_MS)
      {
        digitalWrite(SONAR_LEFT_TRIG_PIN, HIGH); // Start sending pulse to sonar
        delayMicroseconds(SONAR_SIGNAL_DURATION_US); // Delay function used to ensure sonar works correctly, 10us delay doesn't affect the robot's performance
        digitalWrite(SONAR_LEFT_TRIG_PIN, LOW); // Stop sending pulse to sonar
        sonarLastActionTime = millis();
        sonarPhase = SONAR_LEFT_READ_SIGNAL;
      }
      break;

    // Get HIGH state duration from ECHO pin, or move to next phase if no signal after SONAR_RECEIVER_TIMEOUT_MS milliseconds
    case SONAR_LEFT_READ_SIGNAL:
      sonarSignalDuration = pulseIn(SONAR_LEFT_ECHO_PIN, HIGH);
      if (sonarSignalDuration > 0) // Duration of HIGH pulse is 0 until sonar receives its sound signal
      {
        // if more than 210cm mark reading as too far, otherwise we save the reading
        if (sonarSignalDuration > 12353)
        {
          currentDistanceLeft = SONAR_TOO_FAR;
        }
        // HC-SR04 sonar returns time between signal being sent and received in microseconds, so for easier use we convert it to cm
        else
        {
          currentDistanceLeft = (double)sonarSignalDuration * microsecondsToCentimeters;
        }

        sonarSignalDuration = 0; // reset variable just in case, even if doesn't seem necessary
        sonarLastActionTime = millis();
        sonarPhase = SONAR_FRONT_SEND_SIGNAL;
      }
      else if (millis() - sonarLastActionTime > SONAR_RECEIVER_TIMEOUT_MS) // if we didn't receive signal within SONAR_RECEIVER_TIMEOUT_MS ms, mark as "no reading" and move on
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
      if (sonarSignalDuration > 0) // Duration of HIGH pulse is 0 until sonar receives its sound signal
      {
        // if more than 210cm mark reading as too far, otherwise we save the reading
        if (sonarSignalDuration > 12353)
        {
          currentDistanceFront = SONAR_TOO_FAR;
        }
        // HC-SR04 sonar returns time between signal being sent and received in microseconds, so for easier use we convert it to cm
        else
        {
          currentDistanceFront = (double)sonarSignalDuration * microsecondsToCentimeters;
        }

        sonarSignalDuration = 0; // reset variable just in case, even if doesn't seem necessary
        sonarLastActionTime = millis();
        sonarPhase = SONAR_RIGHT_SEND_SIGNAL;
      }
      else if (millis() - sonarLastActionTime > SONAR_RECEIVER_TIMEOUT_MS) // if we didn't receive signal within SONAR_RECEIVER_TIMEOUT_MS ms, mark as "no reading" and move on
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
      if (sonarSignalDuration > 0) // Duration of HIGH pulse is 0 until sonar receives its sound signal
      {
        // if more than 210cm mark reading as too far, otherwise we save the reading
        if (sonarSignalDuration > 12353)
        {
          currentDistanceRight = SONAR_TOO_FAR;
        }
        // HC-SR04 sonar returns time between signal being sent and received in microseconds, so for easier use we convert it to cm
        else
        {
          currentDistanceRight = (double)sonarSignalDuration * microsecondsToCentimeters;
        }

        sonarSignalDuration = 0; // reset variable just in case, even if doesn't seem necessary
        sonarLastActionTime = millis();
        sonarPhase = SONAR_UPDATE_DISTANCES;
      }
      else if (millis() - sonarLastActionTime > SONAR_RECEIVER_TIMEOUT_MS) // if we didn't receive signal within SONAR_RECEIVER_TIMEOUT_MS ms, mark as "no reading" and move on
      {
        currentDistanceRight = SONAR_NO_READING;
        sonarLastActionTime = millis();
        sonarPhase = SONAR_UPDATE_DISTANCES;
      }
      break;

    case SONAR_UPDATE_DISTANCES:
      // If none of the sonar readings changed since last read, we mark robot as potentially stuck
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

// The motors sometimes don't move at all when given speed below 150
// We will let wheels start spinning with max motor speed, then after WHEEL_INERTIA_DELAY_MS change to desired speed
#define WHEEL_INERTIA_DELAY_MS 2

#define FORWARDS_MODE LOW
#define BACKWARDS_MODE HIGH

// The motors require very weird input - seems to be a 9-bit number split over 2 pins
// This function makes using motors much simpler, now speed is between -255 and 255
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

// Using drive(FORWARDS) looks much nicer than setMotors(250, 255), so this function exists
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

#define GRIPPER_OPEN 1600   // Pulse duration for gripper which it interprets as opening
#define GRIPPER_CLOSED 1010 // Pulse duration for gripper which it interprets as closing
unsigned long gripperState = GRIPPER_CLOSED;

#ifdef NEW_GRIPPER_HANDLING
  void setGripper(unsigned long openClose)
  {
    gripperState = openClose;
  }

  // Keep gripper in its desired state by constantly sending signals
  // DOESN'T WORK - adds too much delay to main loop
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
#else
  // Immediately sets the gripper position to the given pulse
  void setGripper(unsigned int pulse)
  {
    for (unsigned char i = 0; i < 8; i++)
    {
      digitalWrite(GRIPPER_PIN, HIGH);
      delayMicroseconds(pulse);
      digitalWrite(GRIPPER_PIN, LOW);
    }
  }
#endif






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
    // Maybe flash lights to say that we cannot connect to base
  }

  return false;
}



///////////////////////////
// PHASE_DRIVE_TO_SQUARE //
///////////////////////////

// Drive from starting spot to black square, pick up pin then go left
// Here using delays is fine because no other code needs to run at this time
void phase_driveToSquare()
{
  // Drive to the square
  while (!allBlack)
  {
    updateLineSensor();
    drive(FORWARDS);
    delay(130);
  }

  updateLineSensor();

  // When on the square grab pin and go left
  if (allBlack)
  {
    drive(FORWARDS);
    delay(100);
    setGripper(GRIPPER_CLOSED);
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

// Follow the black line until we're inside the maze or until 3 seconds pass
void phase_followLineStart()
{
  static unsigned char lastDirection = STOP;

  updateLineSensor();
  bool turnLeft =   (lineSensorValue[0] >= colorBlack) || (lineSensorValue[1] >= colorBlack);
  bool goForwards = (lineSensorValue[2] >= colorBlack) || (lineSensorValue[3] >= colorBlack);
  bool turnRight =  (lineSensorValue[4] >= colorBlack) || (lineSensorValue[5] >= colorBlack);

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

// If we start checking for black line immeditely after entering the maze, we'll find the start line and think we're at the finish
#define CHECK_FINISH_LINE_DELAY_MS 20000

unsigned long activeTaskTimer = millis();

void phase_driveMaze()
{
  static unsigned long stuckTimer = millis(); // Used for assuring delay between checking whether robot is stuck
  static unsigned int stuckRotations = 0;     // Keeps track of wheel rotations to see whether robot is stuck
  static unsigned char stuckCounter = 0;      // Increases if robot is stuck, at certain value we unstuck the robot
  static unsigned char previousTask = STOP;   // Keeps track of previous task for stuck checks
  previousTask = lastMode;

  if (sonarsStuck)
  {
    doTask(BACKWARDS, 100);
    sonarsStuck = false;
  }

  // If a task is ongoing, let it continue
  if (millis() < activeTaskTimer)
  {
    return;
  }

  // If a sonar failed to read distance, wait until it's fixed
  if (distanceLeft >= SONAR_NO_READING || distanceFront >= SONAR_NO_READING || distanceRight >= SONAR_NO_READING)
  {
    doTask(STOP, 50);
    lights(PURPLE);
    return;
  }

  // If we're outside the maze, pause - we don't know if we're at the beginning or end
  if (distanceLeft == SONAR_TOO_FAR && distanceRight == SONAR_TOO_FAR)
  {
    doTask(STOP, 100);
    hazardLights(PURPLE);
    return;
  }

  // Maze AI - since the maze has no loops, we can just follow left wall until finish
  if (distanceFront < 10) // If we're too close to a wall at the front - back off, unless we're in a dead end - then turn around
  {
    if (distanceLeft < 15 && distanceRight < 15)
    {
      turnAround();
    }
    else
    {
      doTask(BACKWARDS, 150);
    }
  }
  else if (distanceLeft > 25) // If no wall on the left, go left
  {
    doTask(LEFT, 500); // After some testing, turning for 500ms brought much better results than shorter turns
  }
  else if (distanceFront > 25) // If wall on the left, but no wall in front, go forward
  {
    drive(FORWARDS);
  }
  else if (distanceRight > 20) // If walls on left and front, but no wall on right, go right
  {
    doTask(RIGHT, 500); // After some testing, turning for 500ms brought much better results than shorter turns
  }
  else if (distanceLeft < 4) // If too close to a wall, turn away from it
  {
    doTask(RIGHT, 100);
  }
  else if (distanceRight < 4) // If too close to a wall, turn away from it
  {
    doTask(LEFT, 100);
  }
  // Just noticed these two below don't make much sense
  // At this point left distance is 4-25, front 10-25, right 4-20 so we should just turn around...
  else if (distanceFront > 10)
  {
    drive(FORWARDS);
  }
  else
  {
    turnAround();
  }

  // If wheels are not spinning, go backwards
  if (millis() - stuckTimer > 200)
  {
    if (leftRotationTicks + rightRotationTicks <= stuckRotations + 5) // If wheel ticks changed by less than 5 in 200ms, go backwards to get unstuck
    {
      doTask(BACKWARDS, 200);
    }
    else // Otherwise update the rotation counter
    {
      stuckRotations = leftRotationTicks + rightRotationTicks;
    }

    stuckTimer = millis();
  }

  // If robot is stuck in forwards-backwards loop, turn left a little
  if (lastMode != previousTask)
  {
    if (lastMode == FORWARDS && previousTask == BACKWARDS || lastMode == BACKWARDS && previousTask == FORWARDS)
    {
      stuckCounter++;
    }
    else
    {
      stuckCounter = 0;
    }
  }
  if (stuckCounter > 6) // If robot went between FORWARDS and BACKWARDS 7+ times, turn left to get unstuck
  {
    doTask(LEFT, 100);
  }
}

// Drive for a set duration of time
void doTask(unsigned char task, unsigned long duration)
{
  drive(task);
  activeTaskTimer = millis() + duration;
}

// Choose best way to turn around
void turnAround()
{
  if (distanceRight > distanceLeft)
  {
    if (distanceLeft > 5)
    {
      doTask(ROTATE_RIGHT, 750);
    }
    else
    {
      doTask(ROTATE_RIGHT, 750); // This used to be driving back-left, then forwards-right but no time to recreate it
    }
  }
  else
  {
    if (distanceRight > 5)
    {
      doTask(ROTATE_LEFT, 750);
    }
    else
    {
      doTask(ROTATE_LEFT, 750); // This used to be driving back-right, then forwards-left but no time to recreate it
    }
  }
}

// Returns whether the two numbers are no more than 'diff' apart, currently unused
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



///////////////
// NEOPIXELS //
///////////////

#define BLINK_DELAY_MS 250

// Automatically update lights depending on what the robot is doing
// But it seems to slow down main loop making maze driving bad :(
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

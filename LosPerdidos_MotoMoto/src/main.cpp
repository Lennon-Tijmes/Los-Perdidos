#include <Arduino.h>

#include <Adafruit_NeoPixel.h>

#define   GRIPPER_PIN           8      // servo pin  
#define   GRIPPER_OPEN    1800   // pulse length servo open
#define   GRIPPER_CLOSED  900    // pulse length servo closed
#define   SERVO_INTERVAL        20    // time between pulse
#define   GRIPPER_TOGGLE        1000  // toggle gripper every second
#define   TRIG_PIN              12
#define   ECHO_PIN              11
#define   MOTOR_LEFT_BACKWARD   5    // Motor Pin
#define   MOTOR_LEFT_FORWARD    9    // Motor Pin
#define   MOTOR_RIGHT_BACKWARD  10     // Motor Pin
#define   MOTOR_RIGHT_FORWARD   6     // Motor Pin 
#define   MOTOR_DEVIATION       14     // Deviation of the motor
#define   SLAVE_ID              2  // This slave's ID (MotoMoto)
#define   NEOPIXEL_PIN          13
#define   NUMPIXELS             4

const int LIGHT_SENSOR[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int LIGHT_VALUE  =   850;  // Light value at the beginning

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

long currentMillis;
bool currentLightState = false;
unsigned long previousMillis = 0;
const long interval = 100; // Interval in milliseconden
bool end = false;

unsigned long distanceMillis = 0;
float duration = 0;

void setup() {
  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pixels.begin();

  Serial.begin(9600);
  Serial.println("Slave (MotoMoto) started");

  for (int i = 0; i < 8; i++)
  {
    pinMode(LIGHT_SENSOR[i], INPUT);
  }
  stopLights();
  start();
}
  

void loop() 
{
  followTheLine();
  forwardLights();
  // Check if a message is received from the master
  if (Serial.available()) {
    String
     message = Serial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(message);

    // If the message is a polling request for this slave
    if (message.length() >= 2 && message[0] == SLAVE_ID + '0' && message[1] == '?') {
    Serial.print("Responding to Master: Slave ");
    Serial.print(SLAVE_ID);
    Serial.println(" Response");
    Serial.println("Example Response Data"); // Send a response;

      // Simulate sensor data
      float speed = random(50, 150) / 10.0; // Random speed between 5.0 and 15.0
      float object_middle = random(10, 100) / 10.0; // Random object distance (middle) in cm
      String line = "On Line";  // Example line state (replace with actual sensor value)

      // Create a response string in the format expected by the master
      String response = String(speed, 1) + "," +
                        String(object_middle, 1) + "," +
                        line; 

      // Send the response back to the master
      Serial.println(response);
}
  }
}
void followTheLine()
{
  if (analogRead(LIGHT_SENSOR[7]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[6]) > LIGHT_VALUE)
  {
    specialTurnLeft();
  }
  else if (analogRead(LIGHT_SENSOR[2]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[3]) > LIGHT_VALUE)
  {
    driveForward(210);
  }
  else if (analogRead(LIGHT_SENSOR[5]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[4]) > LIGHT_VALUE)
  {
    turnLeft(205);
  }
  else if (analogRead(LIGHT_SENSOR[0]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[1]) > LIGHT_VALUE)
  {
     turnRight(200);
  }
  else if (analogRead(LIGHT_SENSOR[0]) < LIGHT_VALUE && analogRead(LIGHT_SENSOR[3]) < LIGHT_VALUE && analogRead(LIGHT_SENSOR[5]) < LIGHT_VALUE && analogRead(LIGHT_SENSOR[7]) < LIGHT_VALUE)
  {
    specialTurnRight();
  }
  else 
  {
    driveForward(210);
  }
}

void theEnd()
{ 
  motorStop();
  driveBackwards(245);
  delay(300);
  motorStop();
  for (int i = 0; i < 8; i++)
  {
    servo(GRIPPER_OPEN);
    delay(100);
  }
  delay(10);
  driveBackwards(245);
  delay(3000);
  winLights();
  motorStop();
}

void servo(int pulse) 
{
  static unsigned long timer;
  static int pulse1;
  if (pulse > 0) 
  {
    pulse1 = pulse;
  }
  if (millis() > timer) 
  {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulse1);
    digitalWrite(GRIPPER_PIN, LOW);
    timer = millis() + SERVO_INTERVAL;
  }
}

void driveForward(int speed)
{
  analogWrite(MOTOR_LEFT_BACKWARD, 0);
  analogWrite(MOTOR_LEFT_FORWARD, speed - MOTOR_DEVIATION);
  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  analogWrite(MOTOR_RIGHT_FORWARD, speed);
}

void turnLeft(int speed)
{ 
  analogWrite(MOTOR_LEFT_BACKWARD, speed);
  analogWrite(MOTOR_LEFT_FORWARD, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  analogWrite(MOTOR_RIGHT_FORWARD, speed);
}

void specialTurnLeft()
{
  driveForward(210);
  delay(300);
  if (analogRead(LIGHT_SENSOR[5]) > LIGHT_VALUE && analogRead(LIGHT_SENSOR[2]) > LIGHT_VALUE )
  {
    theEnd();
    while(true)
    {
      
    }
  }
  turnLeft(215);
  delay(300);
  while (true)
  {
    delay(1);
    if (analogRead(LIGHT_SENSOR[2]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[3]) > LIGHT_VALUE)
    {
      break;
    }
  }
  motorStop();
}

void specialTurnRight()
{
  analogWrite(MOTOR_LEFT_BACKWARD, 0);
  analogWrite(MOTOR_LEFT_FORWARD, 255);
  analogWrite(MOTOR_RIGHT_BACKWARD, 255);
  analogWrite(MOTOR_RIGHT_FORWARD, 0);
  while (true)
  {
    delay(1);
    if (analogRead(LIGHT_SENSOR[4]) > LIGHT_VALUE || analogRead(LIGHT_SENSOR[5]) > LIGHT_VALUE)
    {
      break;
    }
  }
  motorStop();
}

void turnRight(int speed)
{
  analogWrite(MOTOR_LEFT_BACKWARD, 0);
  analogWrite(MOTOR_LEFT_FORWARD, 255);
  analogWrite(MOTOR_RIGHT_BACKWARD, 255);
  analogWrite(MOTOR_RIGHT_FORWARD, 0);

  delay(10);
  
  analogWrite(MOTOR_LEFT_BACKWARD, 0);
  analogWrite(MOTOR_LEFT_FORWARD, speed);
  analogWrite(MOTOR_RIGHT_BACKWARD, speed);
  analogWrite(MOTOR_RIGHT_FORWARD, 0);
}


void driveBackwards(int speed)
{
  analogWrite(MOTOR_LEFT_BACKWARD, speed);
  analogWrite(MOTOR_LEFT_FORWARD, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD, speed);
  analogWrite(MOTOR_RIGHT_FORWARD, 0); 
}

void motorStop()
{
  analogWrite(MOTOR_LEFT_BACKWARD, 0);
  analogWrite(MOTOR_LEFT_FORWARD, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  analogWrite(MOTOR_RIGHT_FORWARD, 0);
}

void start()
{
  Serial.print("Current LIGHT_VALUE: ");
  Serial.println(LIGHT_VALUE);
  
  int blackLineSum = 0;
  int blackLineCount = 0;
  
  for (int i = 0; i < 3; i++)
  {
      while(getDistance() > 24);
  }
    stopLights();
    forwardLights();
    driveForward(255);
  
    while(blackLineCount < 4)
    {
      while (true)
      {
        if (analogRead(LIGHT_SENSOR[3]) > LIGHT_VALUE)
        {
          break;
        }
      }
      while (true)
      {
        if (analogRead(LIGHT_SENSOR[3]) < LIGHT_VALUE)
        {
          break;
        }
      }
        blackLineSum += getAverageLightValue();    
        blackLineCount++; 
    }
      motorStop();

     LIGHT_VALUE = blackLineSum / blackLineCount;

     Serial.print("New LIGHT_VALUE: ");
     Serial.println(LIGHT_VALUE);
     for (int i = 0; i < 100; i++)
     {
        delay(10);
        servo(GRIPPER_CLOSED);
     }
      turnLeft(200);
      delay(600);
      while(true)
      {
        if(analogRead(LIGHT_SENSOR[4]) > LIGHT_VALUE)
        {
          break;
        }
       }
  motorStop();
}

int getAverageLightValue()
{
  int sum = 0;
  for (int i = 0; i < 8; i++)
  {
    sum += analogRead(LIGHT_SENSOR[i]);
  }
  return sum / 8;
}

float getDistance()
{
  if (millis() >= distanceMillis) 
  {
    distanceMillis = millis() + 200;
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    Serial.println(0.017 * duration);
    return 0.017 * duration;
  }
}

void forwardLights()
{
  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.setPixelColor(1, pixels.Color(255, 0, 0));
  pixels.setPixelColor(2, pixels.Color(255, 0, 0));
  pixels.setPixelColor(3, pixels.Color(255, 0, 0));
  pixels.show();
}
void stopLights()
{
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
      pixels.setPixelColor(1, pixels.Color(0, 255, 0));
      pixels.setPixelColor(2, pixels.Color(0, 255, 0));
      pixels.setPixelColor(3, pixels.Color(0, 255, 0));
      pixels.show();
}
void winLights()
{
      pixels.setPixelColor(0, pixels.Color(0, 0, 255));
      pixels.setPixelColor(1, pixels.Color(0, 0, 255));
      pixels.setPixelColor(2, pixels.Color(0, 0, 255));
      pixels.setPixelColor(3, pixels.Color(0, 0, 255));
      pixels.show();
}
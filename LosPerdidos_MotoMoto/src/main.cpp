#include <Arduino.h>

#define motorLFFullSpeed 240
#define motorRFFullSpeed 254
#define motorLBFullSpeed 243
#define motorRBFullSpeed 255
#define motorLHalfSpeed 177
#define motorRHalfSpeed 177
#define motorStop 0
#define motorLB 5
#define motorRB 10
#define motorLF 9
#define motorRF 6


void setup() {
  pinMode(motorLF, OUTPUT);
  pinMode(motorRF, OUTPUT);
  pinMode(motorLB, OUTPUT);
  pinMode(motorRB, OUTPUT);
}

void loop() {
  goForwards();
  delay(1000);
  goBackwards();
  delay(1000);
  goRight();
  delay(1000);
  goLeft();
  delay(1000);
  rotateLeft();
  delay(1000);
  rotateRight();
  delay(1000);
}

void goForwards()
{
  analogWrite(motorLF, motorLFFullSpeed);
  analogWrite(motorRF, motorRFFullSpeed);
  analogWrite(motorLB, motorStop);
  analogWrite(motorRB, motorStop);
}

void goBackwards()  
{ 
  analogWrite(motorLB, motorLBFullSpeed);
  analogWrite(motorRB, motorRBFullSpeed);
  analogWrite(motorLF, motorStop);
  analogWrite(motorRF, motorStop);
}

void goRight()
{
  analogWrite(motorLF, motorLFFullSpeed);
  analogWrite(motorRF, motorRHalfSpeed);
  analogWrite(motorLB, motorStop);
  analogWrite(motorRB, motorStop);
}

void goLeft()
{
  analogWrite(motorLB, motorLHalfSpeed);
  analogWrite(motorRB, motorRFFullSpeed);
  analogWrite(motorLF, motorStop);
  analogWrite(motorRF, motorStop);
}

void rotateRight()
{
  analogWrite(motorLB, motorLBFullSpeed);
  analogWrite(motorRB, motorStop);
  analogWrite(motorRF, motorRFFullSpeed);
  analogWrite(motorLF, motorStop);
}

void rotateLeft()
{
  analogWrite(motorRB, motorRBFullSpeed);
  analogWrite(motorLB, motorStop);
  analogWrite(motorLF, motorLFFullSpeed);
  analogWrite(motorLF, motorStop);
}
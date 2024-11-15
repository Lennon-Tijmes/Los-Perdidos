#include <Arduino.h>

#define motorLFFullSpeed 220
#define motorRFFullSpeed 253
#define motorLBFullSpeed 243
#define motorRBFullSpeed 253
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
  delay(5000);
  goBackwards();
  delay(5000);
  goRight();
  delay(2000);
  goLeft();
  delay(2000);
  rotateRight();
  delay(525);
  stopBot();

  while(1)
  {}
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
  analogWrite(motorLF, motorLHalfSpeed);
  analogWrite(motorRF, motorRFFullSpeed);
  analogWrite(motorLB, motorStop);
  analogWrite(motorRB, motorStop);
}

void rotateLeft()
{
  analogWrite(motorLB, motorLBFullSpeed);
  analogWrite(motorRB, motorStop);
  analogWrite(motorRF, motorRFFullSpeed);
  analogWrite(motorLF, motorStop);
}

void rotateRight()
{
  analogWrite(motorRB, motorRBFullSpeed);
  analogWrite(motorLB, motorStop);
  analogWrite(motorLF, motorLFFullSpeed);
  analogWrite(motorRF, motorStop);
}
void stopBot()
{
  analogWrite(motorRB, motorStop);
  analogWrite(motorLB, motorStop);
  analogWrite(motorLF, motorStop);
  analogWrite(motorRF, motorStop);
}

/*
  goForwards();
  delay(5000);
  goBackwards();
  delay(5000);
  goRight();
  delay(2000);
  goLeft();
  delay(2000);
  rotateLeft();
  delay(500);
*/
#include <Arduino.h>

#define motorLSpeed 255
#define motorRSpeed 255
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
}

void goForwards()
{
  analogWrite(motorLF, motorLSpeed);
  analogWrite(motorRF, motorRSpeed);
  analogWrite(motorLB, motorStop);
  analogWrite(motorRB, motorStop);
}

void goBackwards()  
{ 
  analogWrite(motorLB, motorLSpeed);
  analogWrite(motorRB, motorRSpeed);
  analogWrite(motorLF, motorStop);
  analogWrite(motorRF, motorStop);
}

void goRight()
{
  analogWrite(motorLF, motorLSpeed);
  analogWrite(motorRF, motorStop);
  analogWrite(motorLB, motorStop);
  analogWrite(motorRB, motorStop);
}

void goLeft()
{
  analogWrite(motorLB, motorStop);
  analogWrite(motorRB, motorRSpeed);
  analogWrite(motorLF, motorStop);
  analogWrite(motorRF, motorStop);
}
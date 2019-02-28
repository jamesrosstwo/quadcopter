#include <Servo.h>

Servo ESCs[4];

void setup(){
  Serial.begin(9600);
  for(int i = 0; i < 4; i++){
    ESCs[i].attach(8-i);
    ESCs[i].writeMicroseconds(1000); //initialize the signal to 1000
  }
}

void loop(){
  for(int i = 0; i < 4; i++){
    ESCs[i].writeMicroseconds(1300);
  }
}


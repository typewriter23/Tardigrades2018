#include <Servo.h>
int A;
Servo servo1; 

void setup() {
  Serial.begin(9600);
  servo1.attach(4);
}

void loop() {
  A = Serial.parseInt();
  Serial.println(A);
  
  // put your main code here, to run repeatedly:
  servo1.write(A+90);
  delay(500);
  
 
  //servo1.write(0);
  //delay(100);
 
}

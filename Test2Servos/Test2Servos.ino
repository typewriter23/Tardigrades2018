#include <Servo.h>
Servo servo1; 
Servo servo2;
const int servo1Pin = 2;
const int servo2Pin = 3;
const int laserPin = 4;
const int phiMax = 170;

// const int distSensorPin = 8;
// Pins for the distance sensor:
const int trigPin = 7;
const int echoPin = 8;
const int thetaStep = 5;
const int dTheta = 3.1415926/180;
long duration, inches, cm;
int theta = 0;
int phi = 0;
void setup() {
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  pinMode(laserPin, OUTPUT);
  // pinMode(distSensorPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  Serial.println("Starting...");
  setupDistSensor(trigPin);
}
void testLaser(int laserPin){
    // Tests laser with one long blink, and 2 short blinks
  digitalWrite(laserPin, HIGH);
  delay(1000);
  digitalWrite(laserPin, LOW);
  delay(1000);
  digitalWrite(laserPin, HIGH);
  delay(100);
  digitalWrite(laserPin, LOW);
  delay(100);
  digitalWrite(laserPin, HIGH);
  delay(100);
  digitalWrite(laserPin, LOW);
  delay(100);
  }

void test2Servos(Servo servo2, Servo servo1){
  for(int i=0; i<180; i+=thetaStep){
    for(int j=0; j<phiMax; j+=thetaStep){
      servo1.write(i);
      servo2.write(j);
      Serial.println("\t Coordinates: (" + String(i) + ", " + String(j) + ")");
      Serial.println("\t \t Time: " + String(millis()));
      testDistSensor();
      delay(1000);
      }
    }
  }

void setupDistSensor(int trigPin){
  pinMode(trigPin, OUTPUT);
}
void incrementTheta(){
  
  }
  
long getDistance(){
  // Returns distance in centimeters
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  float dist = microsecondsToCentimeters(duration);
  Serial.println("Distance: " + String(dist));
  return dist;
  }

  
long microsecondsToCentimeters(long microseconds){
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

float calcArea(float dist){
  return 0.5*dist*dist*dTheta;
  }
  
void loop() {
  // testLaser(laserPin);
  Serial.println("Time: " + String(millis()));
  //testDistSensor();
  test2Servos(servo1, servo2);
}

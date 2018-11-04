#include <Servo.h>
#include <RunningMedian.h>
//#include <algorithm>

Servo servoTheta; 
Servo servoPhi;
const int servoThetaPin = 3;
const int servoPhiPin = 2;
const int laserPin = 4;
const int phiMax = 170;

// const int distSensorPin = 8;
// Pins for the distance sensor:
const int trigPin = 7;
const int echoPin = 8;
const int thetaStep = 5;
const float dTheta = 3.1415926/180;
long duration, inches, cm;
int theta = 0;
int phi = 0;
void setup() {
  servoTheta.attach(servoThetaPin);
  servoPhi.attach(servoPhiPin);
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

void test2Servos(Servo servoPhi, Servo servoTheta){
  for(int i=0; i<180; i+=thetaStep){
    for(int j=0; j<phiMax; j+=thetaStep){
      servoTheta.write(i);
      servoPhi.write(j);
      Serial.println("\t Coordinates: (" + String(i) + ", " + String(j) + ")");
      Serial.println("\t \t Time: " + String(millis()));
      getDistance();
      delay(1000);
      }
    }
  }

void setupDistSensor(int trigPin){
  pinMode(trigPin, OUTPUT);
}
//  void incrementTheta(){
//    if(){
//      theta+= thetaStep;
//      servoTheta.write(theta);
//    }
  
void move(int theta, int phi){
  // theta: [0, 360] defined as angle in the horizontal plane
  // phi: [0, 90] defined as angle to the vertical 
  int offset = 90;
  if(theta <180 && theta > 0){
    servoTheta.write(theta);
    servoPhi.write(phi+offset);
  }
  else if(theta>180 && theta < 360){
    servoTheta.write(theta-180);
    servoPhi.write(-phi+offset);
    }
  }
  
//void moveTheta(int theta){
//  if(servoTheta.read()>180){
//    }
//  servoTheta.write
//  }
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
  
long getDistance(float angleElev){
  // Returns distance in centimeters
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  float dist = microsecondsToCentimeters(duration)*cos(angleElev*3.1415926/180);
  Serial.println("Distance: " + String(dist));
  return dist;
  }

long getSampledDistance(){
  // Returns the median of a number of samples
  int numSamples = 10;
  int samples [numSamples];   // Heh, SAMples
  float d = 0;
  RunningMedian rm = RunningMedian(numSamples);
  
  for(int i=0; i<numSamples; i++){
    d=getDistance();
    rm.add(d);
    Serial.print(String(d));
    }
    float med = rm.getMedian();
    Serial.println("Median: "+ String(med));
  return med;
  }
  
long getSampledDistance(float angleElev){
  // Returns the median of a number of samples
  int numSamples = 10;
  int samples [numSamples];   // Heh, SAMples
  float d = 0;
  RunningMedian rm = RunningMedian(numSamples);
  
  for(int i=0; i<numSamples; i++){
    d=getDistance(angleElev);
    rm.add(d);
    Serial.print(String(d));
    }
    float med = rm.getMedian();
    Serial.println("Median: "+ String(med));
  return med;
  }
  
long microsecondsToCentimeters(long microseconds){
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

float calcArea(float dist, float dTheta){
  float area = 0.5*dist*dist*dTheta;
  Serial.println("Area of Sector: " + String(area));
  return area;
  }

float getTotalArea(){
  //Sweeps over 360 degrees
  float totalArea = 0;
  const float anglesOfElev[]= {0.0, 5.0, 10.0, 15.0};
  for(int i=0; i< 360; i++){
      //int pt[] = {pointArrayX[i], pointArrayY[i]};
      for(int j=0; j<4; j++){
        int pt[] = {i, 90-anglesOfElev[j]};
        move(pt[0], pt[1]);
        Serial.println("\t Coordinates: (" + String(pt[0]) + ", " + String(pt[1]) + ")");
        Serial.println("\t \t Time: " + String(millis()));
        float d = getSampledDistance(anglesOfElev[j]);
        float sectorArea = calcArea(d, dTheta);
        totalArea += sectorArea/sizeof(anglesOfElev);
        Serial.println("\t Total area thus far at iteration  [" + String(i) + "]: " + String(totalArea) + " cm^2");
        Serial.println("j is " + String(j));
        delay(10);
        }
    }
  Serial.println(String("Total Area: ") + String("totalArea"));
  }


void loop() {
  // testLaser(laserPin);
  // Serial.println("Time: " + String(millis()));
  getTotalArea();
  delay(10000);
  

}

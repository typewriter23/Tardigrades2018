const int servo1 = 2;
const int servo2 = 3;
const int laserPin = 4;

// const int distSensorPin = 8;
// Pins for the distance sensor:
const int trigPin = 7;
const int echoPin = 8;

const int dTheta = 2*3.1415926;
long duration, inches, cm;

void setup() {
  pinMode(servo1, OUTPUT);
  pinMode(servo2, OUTPUT);
  pinMode(laserPin, OUTPUT);
  // pinMode(distSensorPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  Serial.println("Starting...");
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

long testDistSensor(){
  // Returns distance in centimeters
  duration = pulseIn(echoPin, HIGH);
  float dist = microsecondsToCentimeters(duration);
  Serial.println("Distance" + String(dist));
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
  testLaser(laserPin);
  Serial.println("Time: " + String(millis()));
  testDistSensor();

}

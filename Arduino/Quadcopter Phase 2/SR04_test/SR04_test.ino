#include <HCSR04.h>

int triggerPin = 4;
int echoPin = 5;

double duration, distance;
void setup () {
  Serial.begin(9600);
  HCSR04.begin(triggerPin, echoPin);
  // Extra
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop () {
//  double* distances = HCSR04.measureDistanceCm();
  distance = 0;
  for (int i = 0; i< 10; i++){
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
// Calculating the distance
    distance += duration*0.034/2;  
  }
  distance/=10;
// Clears the trigPin
  
// Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
//  Serial.print("1: ");
//  Serial.print(distances[0]);
//  Serial.println("cm");
//  
  Serial.println("---");
  delay(500);
}

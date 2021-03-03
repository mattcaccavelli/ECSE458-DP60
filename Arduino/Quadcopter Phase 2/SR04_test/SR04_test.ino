#include <HCSR04.h>

int triggerPin = 4;
#define TRIGGER 4
int echoPin = 5;
#define ECHO 5

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
  float time = micros();
  for (int i = 0; i< 1; i++){
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
  // distance/=10;
// Clears the trigPin
  
  // time = micros() - time;
  // Serial.print("Duration = ");
  // Serial.print(time);
  // Serial.println(" µs");
  // Serial.print("Refresh Rate = ");
  // float frequency = 1/time * pow(10, 6);
  // Serial.print(frequency);
  // Serial.println(" Hz");

// Prints the distance on the Serial Monitor
  // Serial.print("Distance: ");
  Serial.println(distance);
//  Serial.print("1: ");
//  Serial.print(distances[0]);
//  Serial.println("cm");
//  
//  Serial.println("---");
//  delay(500);
}

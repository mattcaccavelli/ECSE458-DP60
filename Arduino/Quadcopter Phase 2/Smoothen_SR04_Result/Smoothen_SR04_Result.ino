#include <HCSR04.h>

// Define SMA memory
#define MEMORY 50

//Define DEBUG commands. Uncomment this only if debugging
//#define DEBUG
#define DELAY 1000

// variables for US sensor
int triggerPin = 4;
int echoPin = 5;

// variables for distance calculation
double duration, distance;

// SMA initialization with rotating memories
byte mem = 0;
double averageDistance, distanceArray[MEMORY], totalDistance;
boolean codeInit = true;

int counter = 0;

void setup () {
  Serial.begin(9600);
  Serial.println("1");
  HCSR04.begin(triggerPin, echoPin);
  Serial.println("2");
  // Extra
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  mem = 0;
  totalDistance = 0;
  averageDistance = 0;

//  populate array if first time running the loop
  for (int i = 0; i <= MEMORY; i++)
  {
      /* code */
      distance = computeDistance();
      
      distanceArray[i] = distance;
      totalDistance += distanceArray[i];
  } 
  #ifdef DEBUG
    Serial.println("array filled");
    for (int i = 0; i<MEMORY; i++){
      Serial.print(distanceArray[i]);  
      Serial.print(", ");
    }
  
    delay(5000);
  #endif
  
}

void loop () {
//  double* distances = HCSR04.measureDistanceCm();
 float time = micros();
  // distance = 0;

//   

//   digitalWrite(triggerPin, LOW);
//   delayMicroseconds(2);
// // Sets the trigPin on HIGH state for 10 micro seconds
//   digitalWrite(triggerPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(triggerPin, LOW);
// // Reads the echoPin, returns the sound wave travel time in microseconds
//   duration = pulseIn(echoPin, HIGH);
// // Calculating the distance
//   distance = duration*0.034/2;


  distance = computeDistance();

  //filter values using SMA
  totalDistance -= distanceArray[mem];
  distanceArray[mem] = distance;
  totalDistance += distanceArray[mem];
  mem++;
  if(mem >= MEMORY) mem = 0;
  averageDistance = (double) totalDistance/MEMORY;
  
// Prints the distance on the Serial Monitor
  #ifndef DEBUG
    Serial.println(averageDistance);
  #endif
  
  #ifdef DEBUG
    time = micros() - time;
    float frequency = 1/time * pow(10, 6);
    Serial.print("Refresh Rate: ");
    Serial.print(frequency);
    Serial.println(" Hz");
    Serial.print("Distance: ");
    Serial.println(distance);
    Serial.print("Total Distance: ");
    Serial.println(totalDistance);
    Serial.print("Current Memory Location: ");
    Serial.println(mem);
    Serial.print("Distance Array: ");
    for (int i = 0; i < MEMORY; i++)
    {
      /* code */
      Serial.print(distanceArray[i]);
      if(i<MEMORY-1) Serial.print(", ");
    }
    Serial.println("");
    Serial.print("Average Distance: ");
    Serial.println(averageDistance);
    Serial.println("____________________________");
    delay(DELAY);
    
  #endif

}

double computeDistance(){
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
// Calculating the distance
    distance = duration*0.034/2;
    return distance;
}

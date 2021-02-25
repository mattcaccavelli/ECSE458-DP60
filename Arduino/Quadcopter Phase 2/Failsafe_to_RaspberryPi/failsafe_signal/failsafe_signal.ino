#include <SoftwareSerial.h>

SoftwareSerial failsafe(2, 3); // RX, TX

#define FAILSAFE_SIGNAL 10

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  pinMode(FAILSAFE_SIGNAL, OUTPUT);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.println("BLUETOOTH STARTED!");

  // set the data rate for the SoftwareSerial port
  failsafe.begin(9600);
//  failsafe.println("Hello, world?");
}

void loop() { // run over and over
  if (failsafe.available()){
    char number = failsafe.read();
    int inputSignal = int(number)-'0';
//    Serial.println(inputSignal);
    if(inputSignal){
      digitalWrite(FAILSAFE_SIGNAL, HIGH);
      Serial.print(digitalRead(FAILSAFE_SIGNAL));
      Serial.println("\nGot the data");
      delay(4000);
      
    }
  }
  

//  if (Serial.available()){
//    failsafe.write(Serial.read());
//  }
  Serial.println(digitalRead(FAILSAFE_SIGNAL));
  if(digitalRead(FAILSAFE_SIGNAL)){
    digitalWrite(FAILSAFE_SIGNAL, LOW);
  }
}

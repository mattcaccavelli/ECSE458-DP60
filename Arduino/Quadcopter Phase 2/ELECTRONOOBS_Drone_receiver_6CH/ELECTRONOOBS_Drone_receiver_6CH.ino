/*  
 * Check:  http://www.electronoobs.com/eng_robotica_tut5_2_1.php
 * 
 * 
A basic receiver test for the nRF24L01 module to receive 6 channels send a ppm sum
with all of them on digital pin D2.
Install NRF24 library before you compile
Please, like, share and subscribe on my https://www.youtube.com/c/ELECTRONOOBS
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <HCSR04.h> //Sonar

// Pins for trigger and echo on SR04
int triggerPin = 2;
int echoPin = 3;

// Initialize all variables for PIDs
int prevMillis = 0;
int currentMillis = 0;
int dt = 0;
int setPoint = 0;
int currentDistance = 0;
int error = 0;
int lastError = 0;
int cumError = 0;
int P = 0;
int Kp = 10;
int I = 0;
int Ki = 0.01;
int D = 0;
int Kd = 100;
int c = 0;

////////////////////// PPM CONFIGURATION//////////////////////////
#define channel_number 6  //set the number of channels
#define sigPin 2  //set PPM signal output pin on the arduino
#define PPM_FrLen 27000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 400  //set the pulse length
//////////////////////////////////////////////////////////////////

int ppm[channel_number];

const uint64_t pipeIn =  0xE8E8F0F0E1LL;

RF24 radio(9, 10);

// The sizeof this struct should not exceed 32 bytes
struct MyData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};

MyData data;

void resetData() 
{
  // 'safe' values to use when no radio input is detected
  data.throttle = 0;
  data.yaw = 127;
  data.pitch = 127;
  data.roll = 127;
  data.AUX1 = 0;
  data.AUX2= 0;
  
  setPPMValuesFromData();
}

void setPPMValuesFromData()
{
  ppm[0] = map(data.throttle, 0, 255, 1000, 2000); //Using 5030 props instead of 1045
  ppm[1] = map(data.yaw,      0, 255, 1000, 2000);
  ppm[2] = map(data.pitch,    0, 255, 1000, 2000);
  ppm[3] = map(data.roll,     0, 255, 1000, 2000);
  ppm[4] = map(data.AUX1,     0, 1, 1000, 2000);
  ppm[5] = map(data.AUX2,     0, 1, 1000, 2000);

//  set PIDs here with data from sonar
  int throttle = ppm[0];
  boolean aux1 = data.AUX1;
  if (aux1=0) {
    throttle = 1000;
    c = 0;
    ppm[0] = throttle;
  } else {
    if (c=0) {
      throttle = 1000;
      prevMillis = millis();
      c += 1;
      ppm[0] = throttle;
    } else {
      currentMillis = millis();
      dt = currentMillis - prevMillis;
      setPoint = map(throttle, 1000, 2000, 5, 100);
      double* distances = HCSR04.measureDistanceCm();
      currentDistance = distances[0];
      error = setPoint - currentDistance;
      cumError += error*dt;
      P = Kp*error;
      I = Ki*cumError;
      D = Kd*(error-lastError)/dt;
      throttle = P+I+D;
      throttle = map(throttle, 5, 100, 1000, 2000);
      lastError = error;
      prevMillis = currentMillis;
      ppm[0] = throttle;
    }
  }
  }

/**************************************************/

void setupPPM() {
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, 0);  //set the PPM signal pin to the default state (off)

  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

void setup()
{  
  resetData();
  setupPPM();

  //Setup sonar
  HCSR04.begin(triggerPin, echoPin);
  
  // Set up radio module
  radio.begin();
  radio.setDataRate(RF24_250KBPS); // Both endpoints must have this set the same
  radio.setAutoAck(false);

  radio.openReadingPipe(1,pipeIn);
  radio.startListening();
}

/**************************************************/

unsigned long lastRecvTime = 0;

void recvData()
{  
  while ( radio.available() ) {        
    radio.read(&data, sizeof(MyData));
    lastRecvTime = millis();
  }
}

/**************************************************/

void loop()
{
  recvData();

  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    // signal lost?
    resetData();
  }
  
  setPPMValuesFromData();
}

/**************************************************/

//#error Delete this line befor you cahnge the value (clockMultiplier) below
#define clockMultiplier 2 // set this to 2 if you are using a 16MHz arduino, leave as 1 for an 8MHz arduino

ISR(TIMER1_COMPA_vect){
  static boolean state = true;

  TCNT1 = 0;

  if ( state ) {
    //end pulse
    PORTD = PORTD & ~B00000100; // turn pin 2 off. Could also use: digitalWrite(sigPin,0)
    OCR1A = PPM_PulseLen * clockMultiplier;
    state = false;
  }
  else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B00000100; // turn pin 2 on. Could also use: digitalWrite(sigPin,1)
    state = true;

    if(cur_chan_numb >= channel_number) {
      cur_chan_numb = 0;
      calc_rest += PPM_PulseLen;
      OCR1A = (PPM_FrLen - calc_rest) * clockMultiplier;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * clockMultiplier;
      calc_rest += ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}

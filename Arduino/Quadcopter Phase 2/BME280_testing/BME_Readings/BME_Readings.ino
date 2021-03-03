#include <BME280_MOD-1022.h>
#include <Wire.h>

float currentPressure = 0;
float offset = 0;
float initialPressure = 0;

#define P0 1013.25
#define M 0.02896
#define R 8.3143
#define T 288.15
#define g 9.807
#define e 2.7182818
//#define initialPressure 1004.76

void setup() {
  Serial.begin(115200);
  Wire.begin();
//   uint8_t chipID = BME280.readChipId();
//   Serial.print("BME280 Chip ID: 0x");
//   Serial.println(chipID, HEX);
//  
//   bme280_forcedSample();
  BME__init();

  for (int i = 0; i<80; i++){
    readPressure();
    initialPressure += BME280.getPressureMostAccurate();
  }
  initialPressure /= 80;
  Serial.println(initialPressure);
  offset = log10(initialPressure/P0)/log10(e);
  offset *= R*T/(M*g)*(-1);
}

void BME__init(void){
  // need to read the NVM compensation parameters
  BME280.readCompensationParams();
  
  // Need to turn on 1x oversampling, default is os_skipped, which means it doesn't measure anything
  BME280.writeOversamplingPressure(os1x);  // 1x over sampling (ie, just one sample)
  BME280.writeOversamplingTemperature(os1x);
  BME280.writeOversamplingHumidity(os1x);

  // read out the data - must do this before calling the getxxxxx routines
  BME280.readMeasurements();
}

void readPressure (void){
  BME280.writeStandbyTime(tsb_0p5ms);        // tsb = 0.5ms
  BME280.writeFilterCoefficient(fc_16);      // IIR Filter coefficient 16
  BME280.writeOversamplingPressure(os16x);    // pressure x16
  BME280.writeOversamplingTemperature(os2x);  // temperature x2
  BME280.writeOversamplingHumidity(os1x);     // humidity x1

  while (BME280.isMeasuring());

  BME280.readMeasurements();
}

void loop() {
//  float time = micros();
//   bme280_indoorSample();

 readPressure();
 float currentPressure = BME280.getPressureMostAccurate();
  
//  time = micros() - time;
//  Serial.print("Duration = ");
//  Serial.print(time);
//  Serial.println(" µs");
//  Serial.print("Refresh Rate = ");
//  float frequency = 1/time * pow(10, 6);
//  Serial.print(frequency);
//  Serial.println(" Hz");
  
 float height = log10(currentPressure/P0)/log10(e);
 height *= R*T/(M*g)*(-1);
 height -= offset;
// Serial.print("currentPressure: ");
// Serial.println(currentPressure);
 Serial.println(height*100);
// Serial.println(" cm");
  
//   Serial.println("--------------------------------------");
//   delay(1500);
}

//Numbers printed cleanly
void printFormattedFloat(float x, uint8_t precision) {
  char buffer[10];
  dtostrf(x, 7, precision, buffer);
  Serial.print(buffer);
}

void printCompensatedMeasurements(void) {
  float temp, humidity,  pressure, pressureMoreAccurate;
  double tempMostAccurate, humidityMostAccurate, pressureMostAccurate;
  char buffer[80];

  temp      = BME280.getTemperature();
  humidity  = BME280.getHumidity();
  pressure  = BME280.getPressure();
  
  pressureMoreAccurate = BME280.getPressureMoreAccurate();  // t_fine already calculated from getTemperaure() above
  tempMostAccurate     = BME280.getTemperatureMostAccurate();
  humidityMostAccurate = BME280.getHumidityMostAccurate();
  pressureMostAccurate = BME280.getPressureMostAccurate();
  currentPressure = pressureMostAccurate;

  Serial.println("\t\tGood\t\tBetter\t\tBest");
  Serial.print("Temperature\t");
  printFormattedFloat(temp, 2);
  Serial.print("\t\t-\t\t");
  printFormattedFloat(tempMostAccurate, 2);
  Serial.println();
  
  Serial.print("Humidity\t");
  printFormattedFloat(humidity, 2);
  Serial.print("\t\t-\t\t");
  printFormattedFloat(humidityMostAccurate, 2);
  Serial.println();

  Serial.print("Pressure\t");
  printFormattedFloat(pressure, 2);
  Serial.print("\t\t");
  printFormattedFloat(pressureMoreAccurate, 2);
  Serial.print("\t\t");
  printFormattedFloat(pressureMostAccurate, 2);
  Serial.println();
}

// example of a forced sample.  After taking the measurement the chip goes back to sleep
void bme280_forcedSample() {
  // need to read the NVM compensation parameters
  BME280.readCompensationParams();
  
  // Need to turn on 1x oversampling, default is os_skipped, which means it doesn't measure anything
  BME280.writeOversamplingPressure(os1x);  // 1x over sampling (ie, just one sample)
  BME280.writeOversamplingTemperature(os1x);
  BME280.writeOversamplingHumidity(os1x);
  
  
  BME280.writeMode(smForced);
  Serial.println("BME280 Forced Sample Reading");
  Serial.print("Measuring");
  while (BME280.isMeasuring()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("Done!");
  
  // read out the data - must do this before calling the getxxxxx routines
  BME280.readMeasurements();
  // Serial.print("Temperature =\t");
  // Serial.println(BME280.getTemperature());  // must get temp first
  // Serial.print("Humidity =\t");
  // Serial.println(BME280.getHumidity());
  // Serial.print("Pressure =\t");
  // Serial.println(BME280.getPressure());
  // Serial.println("\tMore Accurate Reading");
  // Serial.print("Pressure =\t");
  // Serial.println(BME280.getPressureMoreAccurate());  // use int64 calculcations
  // Serial.println("\tMost Accuracy Reading");
  // Serial.print("Temperature =\t");
  // Serial.println(BME280.getTemperatureMostAccurate());  // use double calculations
  // Serial.print("Humidity =\t");
  // Serial.println(BME280.getHumidityMostAccurate()); // use double calculations
  // Serial.print("Pressure =\t");
  // Serial.println(BME280.getPressureMostAccurate()); // use double calculations
  // Serial.println();
}

// Example for "indoor navigation"
// We'll switch into normal mode for regular automatic samples
void bme280_indoorSample() {
  
  
  BME280.writeStandbyTime(tsb_0p5ms);        // tsb = 0.5ms
  BME280.writeFilterCoefficient(fc_16);      // IIR Filter coefficient 16
  BME280.writeOversamplingPressure(os16x);    // pressure x16
  BME280.writeOversamplingTemperature(os2x);  // temperature x2
  BME280.writeOversamplingHumidity(os1x);     // humidity x1
  
  BME280.writeMode(smNormal);
  Serial.println("BME280 Normal Mode Reading");
  //Do nothing while measuring
  while (BME280.isMeasuring()) {    }
    
  // read out the data - must do this before calling the getxxxxx routines
  BME280.readMeasurements();
  printCompensatedMeasurements();
  Serial.println();
}

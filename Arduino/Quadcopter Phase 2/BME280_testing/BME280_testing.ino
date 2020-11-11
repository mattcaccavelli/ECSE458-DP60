/*
 * bme280_example.ino
 * Example sketch for bme280
 *
 * Copyright (c) 2016 seeed technology inc.
 * Website    : www.seeedstudio.com
 * Author     : Lambor
 * Create Time:
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "Seeed_BME280.h"
#include <Wire.h>

BME280 bme280;
float offset;
void setup()
{
  Serial.begin(9600);
  if(!bme280.init()){
    Serial.println("Device error!");
  }
  
  for (int i =0;i<100;i++){
    offset+=bme280.getPressure();
  }
  offset/=100;
  Serial.println("Offset done...");
  Serial.println(bme280.calcAltitude(offset));
}

void loop()
{
//  Serial.print("Offset value: ");
//  Serial.println(bme280.calcAltitude(offset));
  float pressure=0;
  for (int i=0;i<100;i++){
    pressure += bme280.getPressure();
  }
  pressure/=100;
  
//  //get and print temperatures
//  Serial.print("Temp: ");
//  Serial.print(bme280.getTemperature());
//  Serial.println("C");//The unit for  Celsius because original arduino don't support speical symbols
  
//  //get and print atmospheric pressure data
//  Serial.print("Pressure: ");
//  Serial.print(pressure = bme280.getPressure());
//  Serial.println("Pa");

  //get and print altitude data
  float altitude = bme280.calcAltitude(pressure)-bme280.calcAltitude(offset);
  altitude *= 100;
  int height = altitude / 10;
  if(height<0) height=0;
  Serial.print("Altitude: ");
  Serial.print(height*10);
  Serial.println(" cm\n");

//  //get and print humidity data
//  Serial.print("Humidity: ");
//  Serial.print(bme280.getHumidity());
//  Serial.println("%\n");

  delay(100);
}

#include <SD.h>
#include "MPL3115A2.h"
#include <Wire.h>

MPL3115A2 myPressure;
File myFile;

void setup(){
  Serial.begin(9600); //to be removed before flight;
  //while (!Serial); //same as above
  pinMode(13, OUTPUT);
  Wire.begin();
  myPressure.begin();
  myPressure.setModeAltimeter();
  myPressure.setOversampleRate(7);
  myPressure.enableEventFlags();
  digitalWrite(13, HIGH);
  delay(500);
  Serial.print("Initializing SD card...");
  digitalWrite(13, LOW);
  pinMode(10, OUTPUT); //chip select pin for sd card
   
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    digitalWrite(13, HIGH);
    return;
  }
  Serial.println("initialization done.");
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile){
    Serial.print("Writing initial time stamp");
    myFile.print("Start time: ");
    myFile.println(millis());
    myFile.close();
  }
   
}

void loop(){
  
  float altitude = myPressure.readAltitudeFt();
  Serial.print(" Altitude(ft):");
  Serial.print(altitude, 2);
  float temperature = myPressure.readTempF();
  
  Serial.print(" Temp(f):");
  Serial.print(temperature, 2);

  Serial.println();
  
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile){
    Serial.print("Writing time stamp");
    myFile.print("Current time: ");
    myFile.println(millis());
    myFile.print("Temperature: ");
    myFile.println(temperature);
    myFile.print("Altitude: ");
    myFile.println(altitude);    
    myFile.close();
  }
  
}

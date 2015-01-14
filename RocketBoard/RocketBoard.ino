#include <SD.h>
#include <Adafruit_VC0706.h>
#include <SPI.h>
#include <SoftwareSerial.h>

SoftwareSerial cameraconnection = SoftwareSerial(8,9);
Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);

void setup(){
  Serial.begin(9600);
  pinMode(10, OUTPUT);
  if(!SD.begin(10)){
    Serial.println("Card failed!");
  }
  else Serial.println("Looks good!");
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } else {
    Serial.println("No camera found?");
    return;
  }
  return;
}

void loop(){
  String dataString = "Hello world!";
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile){
    dataFile.println(dataString);
    Serial.println("Copied into SD card!");
  }
  else Serial.println("We're fucked.");
  dataFile.close();
  delay(500);
  if (cam.begin()) {
    Serial.println("Camera Found:");
    cam.setImageSize(VC0706_640x480);
  } else {
    Serial.println("No camera found?");
    return;
  }
  if (! cam.takePicture()) 
    Serial.println("Failed to snap!");
  else 
    Serial.println("Picture taken!");
}

/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include "PID_v1.h"

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
String readString;

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(PIN_INPUT);
  Setpoint = 100;
  Serial.begin(19200);
  Serial.println("test");

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-90, 90);
  
}

void loop()
{
  readString = "";
  
  if (Serial.available()){
        while (Serial.available()) {
            delay(1);  //delay to allow buffer to fill 
            if (Serial.available() > 0) {
                char c = Serial.read();  //gets one byte from serial buffer
                readString += c; //makes the string readString
            } 
        }
    }
    
    if (readString.length() > 0){
        Serial.println("Input Set to:");
        Serial.println(readString.toFloat());
        Input = readString.toFloat();
    }
    

  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
  delay(500);
  Serial.println("###");
  Serial.println(Output);
  Serial.println(Input);
  Serial.println(Setpoint);
}



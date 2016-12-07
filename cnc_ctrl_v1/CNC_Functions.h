/*This file is part of the Maslow Control Software.
    The Maslow Control Software is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    Maslow Control Software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with the Maslow Control Software.  If not, see <http://www.gnu.org/licenses/>.
    Copyright 2014 Bar Smith*/
    
    
/*Right now this file is a catch all for functions which will be broken out into
libraries*/
    
    
#include "libraries/GearMotor/GearMotor.h"
#include "libraries/Axis/Axis.h"
#include "libraries/Kinematics/Kinematics.h"

#define ZAXIS

#define FORWARD           1
#define BACKWARD         -1

#define CLOCKWISE        -1
#define COUNTERCLOCKWISE  1


#define XDIRECTION BACKWARD
#define YDIRECTION BACKWARD
#define ZDIRECTION BACKWARD


#define MILLIMETERS 1
#define INCHES      25.4

#define DIST_PER_ROTATION 10*6.35//#teeth*pitch of chain
#define Z_DIST_PER_ROTATION 635 //1/8inch in mm

#define ENCODER1A 18
#define ENCODER1B 19
#define ENCODER2A 2
#define ENCODER2B 3
#define ENCODER3A 21
#define ENCODER3B 20

#define IN1 9
#define IN2 8
#define IN3 12
#define IN4 13
#define IN5 14
#define IN6 15

#define ENA 6
#define ENB 7
#define ENC 10


Axis leftAxis (ENB, IN4, IN3, ENCODER2A, ENCODER2B, "Left-axis",   5, DIST_PER_ROTATION);
Axis rightAxis(ENA, IN2, IN1, ENCODER1B, ENCODER1A, "Right-axis", 10, DIST_PER_ROTATION);
Axis zAxis    (ENC, IN5, IN6, ENCODER3B, ENCODER3A, "Z-Axis",     15, DIST_PER_ROTATION/19);


Kinematics kinematics;

float feedrate             =  125;
float _inchesToMMConversion =  1;
String prependString;

void  returnPoz(){
    static unsigned long lastRan = millis();
    int                  timeout = 200;
    
    if (millis() - lastRan > timeout){
        
        float X;
        float Y;
        
        kinematics.forward(leftAxis.read(), rightAxis.read(), &X, &Y);
        
        Serial.print("pz(");
        Serial.print(X/_inchesToMMConversion);
        Serial.print(", ");
        Serial.print(Y/_inchesToMMConversion);
        Serial.print(", ");
        Serial.print(zAxis.read()/_inchesToMMConversion);
        Serial.print(")");
        
        if (_inchesToMMConversion == INCHES){
            Serial.println("in");
        }
        else{
            Serial.println("mm");
        }
        
        kinematics.forward(leftAxis.setpoint(), rightAxis.setpoint(), &X, &Y);
        
        Serial.print("pt(");
        Serial.print(X/_inchesToMMConversion);
        Serial.print(", ");
        Serial.print(Y/_inchesToMMConversion);
        Serial.print(", 0.0)");
        
        if (_inchesToMMConversion == INCHES){
            Serial.println("in");
        }
        else{
            Serial.println("mm");
        }
        
        lastRan = millis();
    }
    
}

void  goAroundInCircle(){
    
    float aChainLength;
    float bChainLength;
    
    float pi = 3.14159;
    float i = 0;
    while(true){
        
        float whereXShouldBeAtThisStep = 100 * cos(i*pi);
        float whereYShouldBeAtThisStep = 100 * sin(i*pi);
        
        kinematics.inverse(whereXShouldBeAtThisStep,whereYShouldBeAtThisStep,&aChainLength,&bChainLength);
        
        
        leftAxis.write(aChainLength);
        rightAxis.write(bChainLength);
        
        delay(15);
        
        returnPoz();
        
        i = i +.0001;
        
        if (i > 2){
            i = 0;
        }
    }
}

int   move(float xEnd, float yEnd, float zEnd, float MMPerSecond){
    
/*The move() function moves the tool in a straight line to the position (xEnd, yEnd, zEnd) at 
the speed moveSpeed. Movements are correlated so that regardless of the distances moved in each 
direction, the tool moves to the target in a straight line. This function is used by the G00 
and G01 commands. The units at this point should all be in rotations or rotations per second*/
    
    float  xStartingLocation;
    float  yStartingLocation;
    float  zStartingLocation;
    int    numberOfStepsPerMM         = 100;
    MMPerSecond = .5;
    
    kinematics.forward(leftAxis.target(), rightAxis.target(), &xStartingLocation, &yStartingLocation);
    
    float  distanceToMoveInMM         = sqrt(  sq(xEnd - xStartingLocation)  +  sq(yEnd - yStartingLocation)  );
    float  xDistanceToMoveInMM        = xEnd - xStartingLocation;
    float  yDistanceToMoveInMM        = yEnd - yStartingLocation;
    float  millisecondsForMove        = numberOfStepsPerMM*(distanceToMoveInMM/MMPerSecond);
    long   finalNumberOfSteps         = abs(distanceToMoveInMM*numberOfStepsPerMM);
    float  timePerStep                = abs(millisecondsForMove/float(finalNumberOfSteps));
    
    float  xStepSize                  = (xDistanceToMoveInMM/distanceToMoveInMM)/float(numberOfStepsPerMM);
    float  yStepSize                  = (yDistanceToMoveInMM/distanceToMoveInMM)/float(numberOfStepsPerMM);
    
    
    long   numberOfStepsTaken         =  0;
    
    leftAxis.attach();
    rightAxis.attach();
    
    float aChainLength;
    float bChainLength;
    
    while(abs(numberOfStepsTaken) < abs(finalNumberOfSteps)){
        
        float whereXShouldBeAtThisStep = xStartingLocation + (numberOfStepsTaken*xStepSize);
        float whereYShouldBeAtThisStep = yStartingLocation + (numberOfStepsTaken*yStepSize);
        
        kinematics.inverse(whereXShouldBeAtThisStep,whereYShouldBeAtThisStep,&aChainLength,&bChainLength);
        
        
        leftAxis.write(aChainLength);
        rightAxis.write(bChainLength);
        numberOfStepsTaken++;
        
        returnPoz();
        
        delay(timePerStep);
    }
    
    kinematics.inverse(xEnd,yEnd,&aChainLength,&bChainLength);
    leftAxis.endMove(aChainLength);
    rightAxis.endMove(bChainLength);
    
    return(1);
    
}

int   rapidMove(float xEnd, float yEnd, float zEnd){
    
    float aChainLength;
    float bChainLength;
    
    kinematics.inverse(xEnd,yEnd,&aChainLength,&bChainLength);
    
    leftAxis.attach();
    rightAxis.attach();
    zAxis.attach();
    
    
    while(true){
        
        leftAxis.write(aChainLength);
        rightAxis.write(bChainLength);
        zAxis.write(zEnd);
        
        returnPoz();
        
        delay(20);

        if (leftAxis.error() < 1 && rightAxis.error() < 1 && zAxis.error() < 1){
            break;
        }
    }
    
    leftAxis.endMove(aChainLength);
    rightAxis.endMove(bChainLength);
    zAxis.endMove(zEnd);
    
}

void  holdPosition(){
    leftAxis.hold();
    rightAxis.hold();
    zAxis.hold();
}
    
float extractGcodeValue(String readString, char target,float defaultReturn){

/*Reads a string and returns the value of number following the target character.
If no number is found, defaultReturn is returned*/

    int begin;
    int end;
    String numberAsString;
    float numberAsFloat;
    
    begin           =  readString.indexOf(target);
    end             =  readString.indexOf(' ', begin);
    numberAsString  =  readString.substring(begin+1,end);
    numberAsFloat   =  numberAsString.toFloat();
    
    if (begin == -1){ //if the character was not found, return error
        return defaultReturn;
    }
    
    return numberAsFloat;
}

int   G1(String readString){
    
/*G1() is the function which is called to process the string if it begins with 
'G01' or 'G00'*/
    
    float xgoto;
    float ygoto;
    float zgoto;
    float gospeed;
    int   isNotRapid;
    
    readString.toUpperCase(); //Make the string all uppercase to remove variability
    
    float currentXPos;
    float currentYPos;
    kinematics.forward(leftAxis.target(), rightAxis.target(), &currentXPos, &currentYPos);
    float currentZPos = zAxis.read();
    
    xgoto      = _inchesToMMConversion*extractGcodeValue(readString, 'X', currentXPos/_inchesToMMConversion);
    ygoto      = _inchesToMMConversion*extractGcodeValue(readString, 'Y', currentYPos/_inchesToMMConversion);
    zgoto      = _inchesToMMConversion*extractGcodeValue(readString, 'Z', currentZPos/_inchesToMMConversion);
    feedrate   = _inchesToMMConversion*extractGcodeValue(readString, 'F', feedrate/_inchesToMMConversion);
    isNotRapid = extractGcodeValue(readString, 'G', 1);
    
    
    #ifndef ZAXIS
    if (zgoto != 0){
        Serial.print("Message: Please adjust Z-Axis to a depth of ");
        if (zgoto > 0){
            Serial.print("+");
        }
        Serial.print(zgoto/_inchesToMMConversion);
        if (_inchesToMMConversion == INCHES){
            Serial.println(" in");
        }
        else{
            Serial.println(" mm");
        }
        
        int    waitTimeMs = 1000;
        double startTime  = millis();
        
        while (millis() - startTime < waitTimeMs){
            delay(1);
            holdPosition();
        } 
    }
    #endif
    
    
    if (isNotRapid){
        move(xgoto, ygoto, zgoto, feedrate); //The XY move is performed
        #ifdef ZAXIS
        if (zgoto != currentZPos/_inchesToMMConversion){
            rapidMove(xgoto, ygoto, zgoto);  //The Z move is performed 
        }
        #endif
    }
    else{
        //if this is a rapid move
        rapidMove(xgoto, ygoto, zgoto);
    }
}

int   arc(float X1, float Y1, float X2, float Y2, float centerX, float centerY, float mmPerSecond, int direction){
    
    mmPerSecond = .5;
    float pi                     =  3.1415;
    float radius                 =  sqrt( sq(centerX - X1) + sq(centerY - Y1) ); 
    float distanceBetweenPoints  =  sqrt( sq(  X2 - X1   ) + sq(    Y2  - Y1) );
    float circumference          =  2.0*pi*radius;
    float theta                  =  acos(  (sq(radius)+sq(radius)-sq(distanceBetweenPoints)) / (2*radius*radius)  ) ;
    
    float arcLengthMM            =  circumference * (theta / (2*pi) );
    float startingAngle          =  atan2(Y1 - centerY, X1 - centerX);
    
    int   numberOfStepsPerMM     =  15;
    int   finalNumberOfSteps     =  arcLengthMM*numberOfStepsPerMM;
    float stepSizeRadians        =  theta/finalNumberOfSteps;
    
    float  millisecondsForMove     = numberOfStepsPerMM*(arcLengthMM/mmPerSecond);
    float stepDelayMs              =  20;
    
    int numberOfStepsTaken         =  0;
    
    float angleNow = startingAngle + direction*stepSizeRadians*numberOfStepsTaken;
    
    float whereXShouldBeAtThisStep = radius * cos(angleNow) + centerX;
    float whereYShouldBeAtThisStep = radius * sin(angleNow) + centerY;
    
    float aChainLength;
    float bChainLength;
    
    while(abs(numberOfStepsTaken) < abs(finalNumberOfSteps)){
        
        angleNow = startingAngle + direction*stepSizeRadians*numberOfStepsTaken;
        
        whereXShouldBeAtThisStep = radius * cos(angleNow) + centerX;
        whereYShouldBeAtThisStep = radius * sin(angleNow) + centerY;
        
        kinematics.inverse(whereXShouldBeAtThisStep,whereYShouldBeAtThisStep,&aChainLength,&bChainLength);
        
        
        leftAxis.write(aChainLength);
        rightAxis.write(bChainLength);
        
        delay(stepDelayMs);
        
        returnPoz();
        
        numberOfStepsTaken = numberOfStepsTaken + 1;
    }
    
    kinematics.inverse(X2,Y2,&aChainLength,&bChainLength);
    leftAxis.endMove(aChainLength);
    rightAxis.endMove(bChainLength);
}

int   G2(String readString){
    
    float X1;
    float Y1;
    kinematics.forward(leftAxis.target(), rightAxis.target(), &X1, &Y1);
    
    float X2      = _inchesToMMConversion*extractGcodeValue(readString, 'X', 0.0);
    float Y2      = _inchesToMMConversion*extractGcodeValue(readString, 'Y', 0.0);
    float I       = _inchesToMMConversion*extractGcodeValue(readString, 'I', 0.0);
    float J       = _inchesToMMConversion*extractGcodeValue(readString, 'J', 0.0);
    float feed    = _inchesToMMConversion*extractGcodeValue(readString, 'F', feedrate/_inchesToMMConversion);
    int   dir     = extractGcodeValue(readString, 'G', 0);
    
    float centerX = X1 + I;
    float centerY = Y1 + J;
    
    if (dir == 2){
        arc(X1, Y1, X2, Y2, centerX, centerY, feed, CLOCKWISE);
    }
    if (dir == 3){
        arc(X1, Y1, X2, Y2, centerX, centerY, feed, COUNTERCLOCKWISE);
    }
}

void  G10(String readString){

/*The G10() function handles the G10 gcode which re-zeroes one or all of the machine's axes.*/
    
    leftAxis.set(0);
    rightAxis.set(0);
    zAxis.set(0);
    
    leftAxis.endMove(0);
    rightAxis.endMove(0);
    zAxis.endMove(0);
    
    leftAxis.attach();
    rightAxis.attach();
    zAxis.attach();
    
    leftAxis.detach();
    rightAxis.detach();
    zAxis.detach();
    
}

void  setInchesToMillimetersConversion(float newConversionFactor){
    _inchesToMMConversion = newConversionFactor;
}

void  interpretCommandString(String readString){
    int i = 0;
    char sect[22];
    
    while (i < 23){
        sect[i] = ' ';
        i++;
    }
    
    Serial.println(readString);
    
    if(readString.substring(0, 3) == "G00" || readString.substring(0, 3) == "G01" || readString.substring(0, 3) == "G02" || readString.substring(0, 3) == "G03" || readString.substring(0, 2) == "G0" || readString.substring(0, 2) == "G1" || readString.substring(0, 2) == "G2" || readString.substring(0, 2) == "G3"){
        prependString = readString.substring(0, 3);
        prependString = prependString + " ";
    }
    
    if(readString[0] == 'X' || readString[0] == 'Y' || readString[0] == 'Z'){
        readString = prependString + readString;
    }
    
    if(readString.substring(0, 3) == "G01" || readString.substring(0, 3) == "G00" || readString.substring(0, 3) == "G0 " || readString.substring(0, 3) == "G1 "){
        
        G1(readString);
        Serial.println("gready");
        Serial.println("ready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G02" || readString.substring(0, 3) == "G2 "){
        G2(readString);
        Serial.println("ready");
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G03" || readString.substring(0, 3) == "G3 "){
        Serial.println("gready");
        G2(readString);
        Serial.println("ready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G10"){
        G10(readString);
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G17"){ //XY plane is the default so no action is taken
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G20"){
        setInchesToMillimetersConversion(INCHES);
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G21"){
        setInchesToMillimetersConversion(MILLIMETERS);
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G90"){ //G90 is the default so no action is taken
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "M06"){ //Tool change are default so no action is taken
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "B01"){
        
        //leftAxis.computeMotorResponse();
        //rightAxis.computeBoost();
        
        Serial.println("Begin motion testing: ");
        
        for(int i = 0; i > -256; i = i - 10){
            Serial.print(i);
            Serial.print("->");
            Serial.println(leftAxis.measureMotorSpeed(i));
        }
        
        readString = "";
        Serial.println("gready");
    }
    
    if(readString.substring(0, 3) == "B05"){
        Serial.println("Firmware Version .59");
        readString = "";
        Serial.println("gready");
    }
    
    if((readString[0] == 'T' || readString[0] == 't') && readString[1] != 'e'){
        if(readString[1] != '1'){
            Serial.print("Please insert tool ");
            Serial.println(readString);
            Serial.println("gready");
        }
        readString = "";
    }
    
    if (readString.length() > 0){
        Serial.println(readString);
        readString = "";
        Serial.println("gready");
    }
} 
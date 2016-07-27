    /*This file is part of the Makesmith Control Software.

    The Makesmith Control Software is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Makesmith Control Software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the Makesmith Control Software.  If not, see <http://www.gnu.org/licenses/>.

    Copyright 2014 Bar Smith*/
    
    /*
    This module contains all of the functions necessary to move the machine. Internally the program operates in units of
    rotation. The conversion ratio between rotations and linear movement is stored by the XPITCH... #defines. When a line
    of Gcode arrives the units are converted into rotations, operated on, and the position is then returned by the machine
    using real world units.
    */

#include "MyTypes.h"
#include "GearMotor.h"
#include "Axis.h"


#define FORWARD 1
#define BACKWARD -1

//these define the number (or fraction there of) of mm moved with each rotation of each axis
#define XPITCH 76.2
#define YPITCH 76.2
#define ZPITCH 1

#define XDIRECTION BACKWARD
#define YDIRECTION BACKWARD
#define ZDIRECTION BACKWARD


Axis xAxis(7, 8, 9, FORWARD, 10, "X-axis", 5);
Axis yAxis(6,12,13, FORWARD, 34, "Y-axis", 10);


float feedrate = 125;
String prependString;

float getAngle(float X,float Y,float centerX,float centerY){

/*This function takes in the ABSOLUTE coordinates of the end of the circle and
 the ABSOLUTE coordinates of the center of the circle and returns the angle between 
 the end and the axis in pi-radians*/

    float theta = 0;
    if ( abs(X - centerX) < .1){
        centerX = X;
    }
    if ( abs(Y - centerY) < .1){
        centerY = Y;
    }

    if (X == centerX) { //this resolves div/0 errors
        if (Y >= centerY) {
            //Serial.println("special case one");
            return(0.5);
        }
        if ( Y <= centerY ){
            //Serial.println("special case two");
            return(1.5);
        }
    }
    if (Y == centerY) { //this resolves div/0 errors
        if ( X >= centerX) {
            //Serial.println("special case three");
            return(0);
        }
        if (X <= centerX) {
            //Serial.println("special case four");
            return(1.0);
        }
    }
    if (X > centerX and Y > centerY) { //quadrant one
        //Serial.println("Quadrant 1");
        theta = atan((centerY - Y)/(X - centerX));
        theta = 2 + theta/3.141593;
    }
    if (X < centerX and Y > centerY) { //#quadrant two
        //Serial.println("Quadrant 2");
        theta = atan((Y - centerY)/(X - centerX));
        theta = 1 - theta/3.141593;
    }
    if (X < centerX and Y < centerY) { //#quadrant three
        //Serial.println("Quadrant 3");
        theta = atan((centerY - Y)/(centerX - X));
        //Serial.println(theta);
        //theta = theta/3.141593 + 1;
        theta = 2 - theta/3.141593;
        theta = theta - 1;
        //Serial.println(theta);
    }
    if (X > centerX and Y < centerY) { //#quadrant four
        //Serial.println("Quadrant 4");
        theta = atan((centerY - Y)/(X - centerX));
        //Serial.println(theta);
        theta = theta/3.141593;
        //Serial.println(theta);
    }

    theta = 2 - theta;

    //Serial.println("Theta: ");
    //Serial.println(theta);
    return(theta);
}

void  chainLengthsToXY(float chainALength, float chainBlength, float* X, float* Y){
    float chainLengthAtCenterInMM       = 1362.45;
    float seperationOfMotorCentersMM    = 2438.40;
    float distFromSpindleToTopAtCenter  = 609;

    //Use the law of cosines to find the angle between the two chains
    float   a   = chainLengthAtCenterInMM + (chainBlength*XPITCH);
    float   b   = chainLengthAtCenterInMM - (chainALength*YPITCH);
    float   c   = seperationOfMotorCentersMM;
    float theta = acos( ( sq(b) + sq(c) - sq(a) ) / (2*b*c) );
    *Y   = distFromSpindleToTopAtCenter - (b*sin(theta));
    *X   = (b*cos(theta)) - seperationOfMotorCentersMM/2;
}

void  returnPoz(){
    static unsigned long lastRan = millis();
    
    if (millis() - lastRan > 200){
        
        float X;
        float Y;
        
        chainLengthsToXY(xAxis.read(), yAxis.read(), &X, &Y);
        
        Serial.print("pz(");
        Serial.print(X);
        Serial.print(", ");
        Serial.print(Y);
        Serial.println(", 0.0)");
        
        lastRan = millis();
    }
    
}

void  xyToChainLengths(float xTarget,float yTarget, float* aChainLength, float* bChainLength){
    Serial.println("conversion function ran");
    
    
    
    *aChainLength = 3.14;
}

int   Move(float xEnd, float yEnd, float zEnd, float rotationsPerSecond){
    
/*The Move() function moves the tool in a straight line to the position (xEnd, yEnd, zEnd) at 
the speed moveSpeed. Movements are correlated so that regardless of the distances moved in each 
direction, the tool moves to the target in a straight line. This function is used by the G00 
and G01 commands. The units at this point should all be in rotations or rotations per second*/
    
    float  xStartingLocation          = xAxis.read();
    float  yStartingLocation          = yAxis.read();
    int    numberOfStepsPerRotation   = 1000;
    
    float aChainLength;
    float bChainLength;
    
    xyToChainLengths(1,2,&aChainLength,&bChainLength);
    
    float  distanceToMoveInRotations  = sqrt(  sq(xEnd - xStartingLocation)  +  sq(yEnd - yStartingLocation)  );
    float  xDistanceToMoveInRotations = xEnd - xStartingLocation;
    float  yDistanceToMoveInRotations = yEnd - yStartingLocation;
    
    float  millisecondsForMove        = numberOfStepsPerRotation*(distanceToMoveInRotations/rotationsPerSecond);
    
    int    finalNumberOfSteps         = distanceToMoveInRotations*numberOfStepsPerRotation;
    
    float  timePerStep                = millisecondsForMove/float(finalNumberOfSteps);
    
    float  xStepSize                  = (xDistanceToMoveInRotations/distanceToMoveInRotations)/float(numberOfStepsPerRotation);
    float  yStepSize                  = (yDistanceToMoveInRotations/distanceToMoveInRotations)/float(numberOfStepsPerRotation);
    
    
    int numberOfStepsTaken            =  0;
    
    xAxis.attach();
    yAxis.attach();
    
    
    while(abs(numberOfStepsTaken) < abs(finalNumberOfSteps)){
        
        float whereXShouldBeAtThisStep = xStartingLocation + (numberOfStepsTaken*xStepSize);
        float whereYShouldBeAtThisStep = yStartingLocation + (numberOfStepsTaken*yStepSize);
        
        delay(timePerStep);
        
        xAxis.updatePositionFromEncoder();
        yAxis.updatePositionFromEncoder();
        
        xAxis.write(whereXShouldBeAtThisStep);
        yAxis.write(whereYShouldBeAtThisStep);
        
        numberOfStepsTaken = numberOfStepsTaken + finalNumberOfSteps/abs(finalNumberOfSteps);
        
        returnPoz();
    }
    
    xAxis.endMove(xEnd);
    yAxis.endMove(yEnd);
    
    return(1);
    
}

void  holdPosition(){
    xAxis.hold();
    yAxis.hold();
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
    
    readString.toUpperCase(); //Make the string all uppercase to remove variability
    
    xgoto   = extractGcodeValue(readString, 'X', xAxis.read());
    ygoto   = extractGcodeValue(readString, 'Y', yAxis.read());
    zgoto   = extractGcodeValue(readString, 'Z', 0.0);
    gospeed = extractGcodeValue(readString, 'F', feedrate);
    
    
    //convert from mm to rotations
    xgoto = xgoto / XPITCH;
    ygoto = ygoto / YPITCH;
    zgoto = zgoto / ZPITCH;
    int secondsPerMinute = 60;
    feedrate = gospeed/(secondsPerMinute*XPITCH); //store the feed rate for later use
    
    Move(xgoto, ygoto, zgoto, feedrate); //The move is performed
}

void  G10(String readString){

/*The G10() function handles the G10 gcode which re-zeroes one or all of the machine's axes.*/
    
    xAxis.set(0);
    yAxis.set(0);
    
}

void interpretCommandString(String readString){
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
        Serial.println("ready");
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G02" || readString.substring(0, 3) == "G2 "){
        Serial.println("ready");
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G03" || readString.substring(0, 3) == "G3 "){
        Serial.println("ready");
        Serial.println("gready");
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
    
    if(readString.substring(0, 3) == "G90"){ //G90 is the default so no action is taken
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "M06"){ //Tool change are default so no action is taken
        Serial.println("gready");
        readString = "";
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

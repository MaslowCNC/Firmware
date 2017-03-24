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
    
    
#include "GearMotor.h"
#include "Axis.h"
#include "Kinematics.h"

#define VERSIONNUMBER 0.60

//#define ZAXIS

#define FORWARD           1
#define BACKWARD         -1

#define CLOCKWISE        -1
#define COUNTERCLOCKWISE  1


#define XDIRECTION BACKWARD
#define YDIRECTION BACKWARD
#define ZDIRECTION BACKWARD

#define LEFT_EEPROM_ADR     5
#define RIGHT_EEPROM_ADR  105
#define Z_EEPROM_ADR      205

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
#define IN3 11
#define IN4 10
#define IN5 12
#define IN6 13

#define ENA 6
#define ENB 7
#define ENC 5


Axis leftAxis (ENC, IN6, IN5, ENCODER3B, ENCODER3A, "Left-axis",   LEFT_EEPROM_ADR, DIST_PER_ROTATION);
Axis rightAxis(ENA, IN1, IN2, ENCODER1A, ENCODER1B, "Right-axis", RIGHT_EEPROM_ADR, DIST_PER_ROTATION);
Axis zAxis    (ENB, IN3, IN4, ENCODER2B, ENCODER2A, "Z-Axis",         Z_EEPROM_ADR, DIST_PER_ROTATION/19);


Kinematics kinematics;

float feedrate             =  125;
float _inchesToMMConversion =  1;
String prependString;       //prefix ('G01' for ex) from the previous command
String readString;          //command being built one character at a time
String readyCommandString;  //next command queued up and ready to send

//These are used in place of a forward kinematic function at the beginning of each move. They should be replaced
//by a call to the forward kinematic function when it is available.
float xTarget = 0;
float yTarget = 0;

void  returnPoz(float x, float y, float z){
    /*
    Causes the machine's position (x,y) to be sent over the serial connection updated on the UI
    in Ground Control. Only executes if hasn't been called in at least timeout ms.
    */
    
    static unsigned long lastRan = millis();
    int                  timeout = 200;
    
    if (millis() - lastRan > timeout){
        float errorTerm = (leftAxis.error() + rightAxis.error() )/2;
        
        Serial.print("pz(");
        Serial.print(x/_inchesToMMConversion);
        Serial.print(", ");
        Serial.print(y/_inchesToMMConversion);
        Serial.print(", ");
        Serial.print(z/_inchesToMMConversion);
        Serial.print(", ");
        Serial.print(errorTerm);
        Serial.print(")");
        
        if (_inchesToMMConversion == INCHES){
            Serial.println("in");
        }
        else{
            Serial.println("mm");
        }
        
        lastRan = millis();
    }
    
}

void readSerialCommands(){
    /*
    Check to see if a new character is available from the serial connection, read it if one is.
    */
    if (Serial.available() > 0) {
        char c = Serial.read();  //gets one byte from serial buffer
        if (c == '\n'){
            readyCommandString = readString;
            readString = "";
        }
        else{
            readString += c; //makes the string readString
        }
    }
}

bool checkForStopCommand(){
    /*
    Check to see if the STOP command has been sent to the machine.
    */
    if(readString.endsWith("STOP")){
        readString = "";
        readyCommandString = "";
        return 1;
    }
    return 0;
}

float calculateDelay(float stepSizeMM, float feedrateMMPerMin){
    /*
    Calculate the time delay between each step for a given feedrate
    */
    
    #define MINUTEINMS 60000.0
    
    // derivation: ms / step = 1 min in ms / dist in one min
    
    float msPerStep = (stepSizeMM*MINUTEINMS)/feedrateMMPerMin;
    
    return msPerStep;
}

int   cordinatedMove(float xEnd, float yEnd, float MMPerMin){
    
/*The move() function moves the tool in a straight line to the position (xEnd, yEnd) at 
the speed moveSpeed. Movements are correlated so that regardless of the distances moved in each 
direction, the tool moves to the target in a straight line. This function is used by the G00 
and G01 commands. The units at this point should all be in mm or mm per minute*/
    
    float  xStartingLocation = xTarget;
    float  yStartingLocation = yTarget;
    float  stepSizeMM         = .5;
    
    //find the total distances to move
    float  distanceToMoveInMM         = sqrt(  sq(xEnd - xStartingLocation)  +  sq(yEnd - yStartingLocation)  );
    float  xDistanceToMoveInMM        = xEnd - xStartingLocation;
    float  yDistanceToMoveInMM        = yEnd - yStartingLocation;
    
    //compute the total  number of steps in the move
    long   finalNumberOfSteps         = abs(distanceToMoveInMM/stepSizeMM);
    
    // (fraction of distance in x direction)* size of step toward target
    float  xStepSize                  = (xDistanceToMoveInMM/distanceToMoveInMM)*stepSizeMM;
    float  yStepSize                  = (yDistanceToMoveInMM/distanceToMoveInMM)*stepSizeMM;
    
    //attach the axes
    leftAxis.attach();
    rightAxis.attach();
    
    float aChainLength;
    float bChainLength;
    long   numberOfStepsTaken         =  0;
    long  beginingOfLastStep          = millis();
    while(abs(numberOfStepsTaken) < abs(finalNumberOfSteps)){
        
        //if enough time has passed to take the next step
        if (millis() - beginingOfLastStep > calculateDelay(stepSizeMM, MMPerMin)){
            
            //reset the counter 
            beginingOfLastStep          = millis();
            
            //find the target point for this step
            float whereXShouldBeAtThisStep = xStartingLocation + (numberOfStepsTaken*xStepSize);
            float whereYShouldBeAtThisStep = yStartingLocation + (numberOfStepsTaken*yStepSize);
            
            //find the chain lengths for this step
            kinematics.inverse(whereXShouldBeAtThisStep,whereYShouldBeAtThisStep,&aChainLength,&bChainLength);
            
            //write to each axis
            leftAxis.write(aChainLength);
            rightAxis.write(bChainLength);
            
            //increment the number of steps taken
            numberOfStepsTaken++;
            
            //update position on display
            returnPoz(whereXShouldBeAtThisStep, whereYShouldBeAtThisStep, zAxis.read());
            
            //check for new serial commands
            readSerialCommands();
            
            //check for a STOP command
            if(checkForStopCommand()){
                
                //set the axis positions to save
                kinematics.inverse(whereXShouldBeAtThisStep,whereYShouldBeAtThisStep,&aChainLength,&bChainLength);
                leftAxis.endMove(aChainLength);
                rightAxis.endMove(bChainLength);
                
                //make sure the positions are displayed correctly after stop
                xTarget = whereXShouldBeAtThisStep;
                yTarget = whereYShouldBeAtThisStep;
                
                return 1;
            }
            
        }
    }
    
    kinematics.inverse(xEnd,yEnd,&aChainLength,&bChainLength);
    leftAxis.endMove(aChainLength);
    rightAxis.endMove(bChainLength);
    
    xTarget = xEnd;
    yTarget = yEnd;
    
    return 1;
    
}

void  singleAxisMove(Axis* axis, float endPos, float MMPerMin){
    /*
    Takes a pointer to an axis object and moves that axis to endPos at speed MMPerMin
    */
    
    float startingPos          = axis->target();
    float moveDist             = startingPos - endPos; //total distance to move
    
    float direction            = -1* moveDist/abs(moveDist); //determine the direction of the move
    
    float stepSizeMM           = 0.01;                    //step size in mm
    long finalNumberOfSteps    = moveDist/stepSizeMM;      //number of steps taken in move
    
    long numberOfStepsTaken    = 0;
    long  beginingOfLastStep   = millis();
    
    axis->attach();
    
    while(abs(numberOfStepsTaken) < abs(finalNumberOfSteps)){
        
        //reset the counter 
        beginingOfLastStep          = millis();
        
        //find the target point for this step
        float whereAxisShouldBeAtThisStep = startingPos + numberOfStepsTaken*stepSizeMM*direction;
        
        //write to each axis
        axis->write(whereAxisShouldBeAtThisStep);
        
        //update position on display
        returnPoz(xTarget, yTarget, zAxis.read());
        
        //calculate the correct delay between steps to set feedrate
        delay(calculateDelay(stepSizeMM, MMPerMin));
        
        //increment the number of steps taken
        numberOfStepsTaken++;
        
        //check for new serial commands
        readSerialCommands();
        
        //check for a STOP command
        if(checkForStopCommand()){
            axis->endMove(whereAxisShouldBeAtThisStep);
            return;
        }
    }
    
    axis->endMove(endPos);
    
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
    
    float currentXPos = xTarget;
    float currentYPos = yTarget;
    //kinematics.forward(leftAxis.target(), rightAxis.target(), &currentXPos, &currentYPos);
    float currentZPos = zAxis.target();
    
    xgoto      = _inchesToMMConversion*extractGcodeValue(readString, 'X', currentXPos/_inchesToMMConversion);
    ygoto      = _inchesToMMConversion*extractGcodeValue(readString, 'Y', currentYPos/_inchesToMMConversion);
    zgoto      = _inchesToMMConversion*extractGcodeValue(readString, 'Z', currentZPos/_inchesToMMConversion);
    feedrate   = _inchesToMMConversion*extractGcodeValue(readString, 'F', feedrate/_inchesToMMConversion);
    isNotRapid = extractGcodeValue(readString, 'G', 1);
    
    #ifndef ZAXIS
    float threshold = .1; //units of mm
    if (abs(currentZPos - zgoto) > threshold){
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
        
        zAxis.set(zgoto);
        
        int    waitTimeMs = 1000;
        double startTime  = millis();
        
        while (millis() - startTime < waitTimeMs){
            delay(1);
            holdPosition();
        } 
    }
    #endif
    
    #ifdef ZAXIS
    if (zgoto != currentZPos/_inchesToMMConversion){
        singleAxisMove(&zAxis, zgoto,40);
    }
    #endif
    
    if (isNotRapid){
        //if this is a regular move
        cordinatedMove(xgoto, ygoto, feedrate); //The XY move is performed
    }
    else{
        //if this is a rapid move
        cordinatedMove(xgoto, ygoto, 1200); //move the same as a regular move, but go fast
    }
}

int   arc(float X1, float Y1, float X2, float Y2, float centerX, float centerY, float MMPerMin, int direction){
    
    float pi                     =  3.1415;
    float radius                 =  sqrt( sq(centerX - X1) + sq(centerY - Y1) ); 
    float distanceBetweenPoints  =  sqrt( sq(  X2 - X1   ) + sq(    Y2  - Y1) );
    float circumference          =  2.0*pi*radius;
    float theta                  =  acos(  (sq(radius)+sq(radius)-sq(distanceBetweenPoints)) / (2*radius*radius)  ) ;
    
    float arcLengthMM            =  circumference * (theta / (2*pi) );
    float startingAngle          =  atan2(Y1 - centerY, X1 - centerX);
    
    int numberOfStepsTaken       =  0;
    
    float stepSizeMM             =  .2;
    int   finalNumberOfSteps     =  arcLengthMM/stepSizeMM;
    float stepSizeRadians        =  theta/finalNumberOfSteps;
    
    float angleNow = startingAngle + direction*stepSizeRadians*numberOfStepsTaken;
    
    float whereXShouldBeAtThisStep = radius * cos(angleNow) + centerX;
    float whereYShouldBeAtThisStep = radius * sin(angleNow) + centerY;
    
    float aChainLength;
    float bChainLength;
    
    //attach the axes
    leftAxis.attach();
    rightAxis.attach();
    
    while(abs(numberOfStepsTaken) < abs(finalNumberOfSteps)){
        
        angleNow = startingAngle + direction*stepSizeRadians*numberOfStepsTaken;
        
        whereXShouldBeAtThisStep = radius * cos(angleNow) + centerX;
        whereYShouldBeAtThisStep = radius * sin(angleNow) + centerY;
        
        kinematics.inverse(whereXShouldBeAtThisStep,whereYShouldBeAtThisStep,&aChainLength,&bChainLength);
        
        
        leftAxis.write(aChainLength);
        rightAxis.write(bChainLength);
        
        delay(calculateDelay(stepSizeMM, MMPerMin));
        
        returnPoz(whereXShouldBeAtThisStep, whereYShouldBeAtThisStep, zAxis.read());
        
        numberOfStepsTaken = numberOfStepsTaken + 1;
        
        //check for new serial commands
        readSerialCommands();
        
        //check for a STOP command
        if(checkForStopCommand()){
            //set the axis positions to save
            kinematics.inverse(whereXShouldBeAtThisStep,whereYShouldBeAtThisStep,&aChainLength,&bChainLength);
            leftAxis.endMove(aChainLength);
            rightAxis.endMove(bChainLength);
            
            //make sure the positions are displayed correctly after stop
            xTarget = whereXShouldBeAtThisStep;
            yTarget = whereYShouldBeAtThisStep;
            
            return 1;
        }
    }
    
    kinematics.inverse(X2,Y2,&aChainLength,&bChainLength);
    leftAxis.endMove(aChainLength);
    rightAxis.endMove(bChainLength);
    
    xTarget = X2;
    yTarget = Y2;
    
    return 1;
}

int   G2(String readString){
    
    float X1 = xTarget; //does this work if units are inches? (It seems to)
    float Y1 = yTarget;
    
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
    /*The G10() function handles the G10 gcode which re-zeros one or all of the machine's axes.*/
    
    float currentXPos = xTarget;
    float currentYPos = yTarget;
    float currentZPos = zAxis.read();
    
    float xgoto      = _inchesToMMConversion*extractGcodeValue(readString, 'X', currentXPos/_inchesToMMConversion);
    float ygoto      = _inchesToMMConversion*extractGcodeValue(readString, 'Y', currentYPos/_inchesToMMConversion);
    float zgoto      = _inchesToMMConversion*extractGcodeValue(readString, 'Z', currentZPos/_inchesToMMConversion);
    
    zAxis.set(zgoto);
    zAxis.endMove(zgoto);
    zAxis.attach();
    leftAxis.detach();
}

void  calibrateChainLengths(){
    /*
    The calibrateChainLengths function lets the machine know that the chains are set to a given length where each chain is ORIGINCHAINLEN
    in length
    */
    
    
    //measure out the left chain
    Serial.println("Measuring out left chain");
    leftAxis.set(0);
    singleAxisMove(&leftAxis, ORIGINCHAINLEN, 500);
    leftAxis.detach();
    
    //measure out the right chain
    Serial.println("Measuring out right chain");
    rightAxis.set(0);
    singleAxisMove(&rightAxis, ORIGINCHAINLEN, 500);
    rightAxis.detach();
    
    xTarget = 0;
    yTarget = 0;
    
}

void  setInchesToMillimetersConversion(float newConversionFactor){
    _inchesToMMConversion = newConversionFactor;
}

void  printBeforeAndAfter(float before, float after){
    Serial.print("Before: ");
    Serial.print(before);
    Serial.print(" After: ");
    Serial.println(after);
}

void  updateSettings(String readString){
    /*
    Updates the machine dimensions from the Ground Control settings
    */
    
    //Extract the settings values
    float bedWidth           = extractGcodeValue(readString, 'A', 0);
    float bedHeight          = extractGcodeValue(readString, 'C', 0);
    float distBetweenMotors  = extractGcodeValue(readString, 'D', 0);
    float motorOffsetX       = (distBetweenMotors - bedWidth)/2;
    float motorOffsetY       = extractGcodeValue(readString, 'E', 0);
    float sledWidth          = extractGcodeValue(readString, 'F', 0);
    float sledHeight         = extractGcodeValue(readString, 'G', 0);
    float sledCG             = extractGcodeValue(readString, 'H', 0);
    
    
    //Change the machine dimensions in the kinematics 
    printBeforeAndAfter(kinematics.l, sledWidth);
    kinematics.l            = sledWidth;
    printBeforeAndAfter(kinematics.s, sledHeight);
    kinematics.s            = sledHeight;
    printBeforeAndAfter(kinematics.h3, sledCG);
    kinematics.h3           = sledCG;
    printBeforeAndAfter(kinematics.D, distBetweenMotors);
    kinematics.D            = distBetweenMotors;
    printBeforeAndAfter(kinematics.motorOffsetX, motorOffsetX);
    kinematics.motorOffsetX = motorOffsetX;
    printBeforeAndAfter(kinematics.motorOffsetY, motorOffsetY);
    kinematics.motorOffsetY = motorOffsetY;
    printBeforeAndAfter(kinematics.machineWidth, bedWidth);
    kinematics.machineWidth = bedWidth;
    printBeforeAndAfter(kinematics.machineHeight, bedHeight);
    kinematics.machineHeight= bedHeight;
    kinematics.recomputeGeometry();
    
    Serial.println("Machine Settings Updated");
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
        G2(readString);
        Serial.println("gready");
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
        
        leftAxis.computeMotorResponse();
        rightAxis.computeMotorResponse();
        
        //Serial.println("Begin motion testing: ");
        
        /*for(int i = 0; i > -256; i = i - 10){
            Serial.print(i);
            Serial.print("->");
            Serial.println(leftAxis.measureMotorSpeed(i));
        }*/
        
        readString = "";
        Serial.println("gready");
    }
    
    if(readString.substring(0, 3) == "B02"){
        calibrateChainLengths();
        readString = "";
        Serial.println("gready");
    }
    
    if(readString.substring(0, 3) == "B03"){
        updateSettings(readString);
        readString = "";
        Serial.println("gready");
    }
    
    if(readString.substring(0, 3) == "B04"){
        //Test each of the axis
        delay(500);
        leftAxis.test();
        delay(500);
        rightAxis.test();
        delay(500);
        zAxis.test();
        Serial.println("Tests complete.");
        readString = "";
        Serial.println("gready");
    }
    
    if(readString.substring(0, 3) == "B05"){
        Serial.print("Firmware Version ");
        Serial.println(VERSIONNUMBER);
        readString = "";
        Serial.println("gready");
    }
    
    if((readString[0] == 'T' || readString[0] == 't') && readString[1] != 'e'){
        Serial.print("Please insert tool ");
        Serial.println(readString);
        Serial.println("gready");
        readString = "";
    }
    
    if (readString.length() > 0){
        Serial.println(readString);
        readString = "";
        Serial.println("gready");
    }
} 

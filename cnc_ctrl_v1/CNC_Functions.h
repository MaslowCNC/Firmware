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
    Copyright 2014-2017 Bar Smith*/
    
    
/*Right now this file is a catch all for functions which will be broken out into
libraries*/
    
    
#include "Axis.h"
#include "Kinematics.h"
#include "RingBuffer.h"

#define VERSIONNUMBER 0.67

bool zAxisAttached = false;

#define FORWARD           1
#define BACKWARD         -1

#define CLOCKWISE        -1
#define COUNTERCLOCKWISE  1

#define LEFT_EEPROM_ADR     5
#define RIGHT_EEPROM_ADR  105
#define Z_EEPROM_ADR      205

#define MILLIMETERS 1
#define INCHES      25.4

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

#define DISTPERROT     10*6.35//#teeth*pitch of chain
#define ZDISTPERROT    3.17//1/8inch in mm
#define ENCODERSTEPS   8148.0
#define ZENCODERSTEPS  7560.0 //7*270*4 --- 7ppr, 270:1 gear ratio, quadrature encoding

Axis leftAxis (ENC, IN6, IN5, ENCODER3B, ENCODER3A, "Left-axis",   LEFT_EEPROM_ADR, DISTPERROT, ENCODERSTEPS);
Axis rightAxis(ENA, IN1, IN2, ENCODER1A, ENCODER1B, "Right-axis", RIGHT_EEPROM_ADR, DISTPERROT, ENCODERSTEPS);
Axis zAxis    (ENB, IN3, IN4, ENCODER2B, ENCODER2A, "Z-Axis",         Z_EEPROM_ADR, ZDISTPERROT, ZENCODERSTEPS);


Kinematics kinematics;
RingBuffer ringBuffer;

float feedrate              =  125;
float _inchesToMMConversion =  1;
bool  useRelativeUnits      =  false;
bool  stopFlag              =  false;
String readString;                        //command being built one character at a time
String readyCommandString;                //next command queued up and ready to send
int   lastCommand           =  0;         //Stores the value of the last command run eg: G01 -> 1

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
        
        Serial.print("<Idle,MPos:");
        Serial.print(x/_inchesToMMConversion);
        Serial.print(",");
        Serial.print(y/_inchesToMMConversion);
        Serial.print(",");
        Serial.print(z/_inchesToMMConversion);
        Serial.println(",WPos:0.000,0.000,0.000>");
        
        Serial.print("[PosError:");
        Serial.print(leftAxis.error());
        Serial.print(',');
        Serial.print(rightAxis.error());
        Serial.println("]");
        
        lastRan = millis();
    }
    
}

void  _signalReady(){
    /*
    
    Signal to the controlling software that the machine has executed the last
    gcode line successfully.
    
    */
    
    Serial.println("ok");
}

void  _watchDog(){
    /*
    Watchdog tells ground control that the machine is ready every second. _watchDog() should only be called when 
    the machine is actually ready.
    
    This fixes the issue where the machine is ready, but Ground Control doesn't know the machine is ready and the system locks up.
    */
    static unsigned long lastRan = millis();
    int                  timeout = 5000;
    
    if (millis() - lastRan > timeout){
        
        if (!leftAxis.attached() and !rightAxis.attached() and !zAxis.attached()){
            _signalReady();
        }
        
        lastRan = millis();
    }
}

void readSerialCommands(){
    /*
    Check to see if a new character is available from the serial connection, read it if one is.
    */
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '!'){
            stopFlag = true;
        }
        else{
            ringBuffer.write(c); //gets one byte from serial buffer, writes it to the internal ring buffer
        }
    }
}

bool checkForStopCommand(){
    /*
    Check to see if the STOP command has been sent to the machine.
    */
    if(stopFlag){
        readString = "";
        readyCommandString = "";
        stopFlag = false;
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
    
int   findEndOfNumber(String textString, int index){
    //Return the index of the last digit of the number beginning at the index passed in
    int i = index;
    
    while (i < textString.length()){
        
        if(isDigit(textString[i]) or isPunct(textString[i])){ //If we're still looking at a number, keep goin
            i++;
        }
        else{
            return i;                                         //If we've reached the end of the number, return the last index
        }
    }
    return i;                                                 //If we've reached the end of the string, return the last number
}
    
float extractGcodeValue(String readString, char target,float defaultReturn){

/*Reads a string and returns the value of number following the target character.
If no number is found, defaultReturn is returned*/

    int begin;
    int end;
    String numberAsString;
    float numberAsFloat;
    
    begin           =  readString.indexOf(target);
    end             =  findEndOfNumber(readString,begin+1);
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
    

    if (useRelativeUnits){ //if we are using a relative coordinate system 
        
        if(readString.indexOf('X') >= 0){ //if there is an X command
            xgoto = currentXPos + xgoto;
        }
        if(readString.indexOf('Y') >= 0){ //if y has moved
            ygoto = currentYPos + ygoto;
        }
        if(readString.indexOf('Z') >= 0){ //if y has moved
            zgoto = currentZPos + zgoto;
        }
    }
    
    feedrate   = constrain(feedrate, 1, 25*_inchesToMMConversion);                                              //constrain the maximum feedrate
    
    //if the zaxis is attached
    if(zAxisAttached){
        if (zgoto != currentZPos/_inchesToMMConversion){
            singleAxisMove(&zAxis, zgoto,45);
        }
    }
    else{
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
    }
    
    
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
    
    //compute geometry 
    float pi                     =  3.1415;
    float radius                 =  sqrt( sq(centerX - X1) + sq(centerY - Y1) ); 
    float distanceBetweenPoints  =  sqrt( sq(  X2 - X1   ) + sq(    Y2  - Y1) );
    float circumference          =  2.0*pi*radius;
    
    //compute angle between lines
    float cosTheta = (sq(radius)+sq(radius)-sq(distanceBetweenPoints)) / (2*radius*radius);
    cosTheta       = constrain(cosTheta, -1.0, 1.0); //when the angle is exactly 180 degrees, rounding errors can push the argument of acos() outside its +-1 range
    float theta                  =  acos(cosTheta);
    
    float arcLengthMM            =  circumference * (theta / (2*pi) );
    float startingAngle          =  atan2(Y1 - centerY, X1 - centerX);
    
    //set up variables for movement
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
    
    float X2      = _inchesToMMConversion*extractGcodeValue(readString, 'X', X1/_inchesToMMConversion);
    float Y2      = _inchesToMMConversion*extractGcodeValue(readString, 'Y', Y1/_inchesToMMConversion);
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
    
    Serial.print(leftAxis.read());
    Serial.println("mm");
    
    //measure out the right chain
    Serial.println("Measuring out right chain");
    rightAxis.set(0);
    singleAxisMove(&rightAxis, ORIGINCHAINLEN, 500);
    rightAxis.detach();
    
    Serial.print(rightAxis.read());
    Serial.println("mm");
    
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

    float bedWidth           = extractGcodeValue(readString, 'A', kinematics.machineWidth);
    float bedHeight          = extractGcodeValue(readString, 'C', kinematics.machineHeight);
    float distBetweenMotors  = extractGcodeValue(readString, 'Q', kinematics.D);
    float motorOffsetX       = extractGcodeValue(readString, 'D', (distBetweenMotors - bedWidth)/2); //read the motor offset X IF it is sent, if it's not sent, compute it from the spacing between the motors
    float motorOffsetY       = extractGcodeValue(readString, 'E', motorOffsetY);
    float sledWidth          = extractGcodeValue(readString, 'F', kinematics.l);
    float sledHeight         = extractGcodeValue(readString, 'R', kinematics.s);
    float sledCG             = extractGcodeValue(readString, 'H', kinematics.h3);
    zAxisAttached            = extractGcodeValue(readString, 'I', zAxisAttached);
    int encoderSteps         = extractGcodeValue(readString, 'J', ENCODERSTEPS);
    float gearTeeth          = extractGcodeValue(readString, 'K', 10);
    float chainPitch         = extractGcodeValue(readString, 'M', 6.35);
    
    float zDistPerRot        = extractGcodeValue(readString, 'N', ZDISTPERROT);
    int zEncoderSteps        = extractGcodeValue(readString, 'P', ZENCODERSTEPS);
    
    //Change the motor properties in cnc_funtions
    float distPerRot = gearTeeth*chainPitch; 
    leftAxis.changePitch(distPerRot);
    rightAxis.changePitch(distPerRot);
    zAxis.changePitch(zDistPerRot);
    
    leftAxis.changeEncoderResolution(encoderSteps);
    rightAxis.changeEncoderResolution(encoderSteps);
    zAxis.changeEncoderResolution(zEncoderSteps);
    
    //Change the machine dimensions in the kinematics 
    kinematics.l            = sledWidth;
    kinematics.s            = sledHeight;
    kinematics.h3           = sledCG;
    kinematics.R            = distPerRot / (2.0*3.14159);
    kinematics.D            = distBetweenMotors;
    kinematics.motorOffsetY = motorOffsetY;
    kinematics.machineWidth = bedWidth;
    kinematics.machineHeight= bedHeight;
    kinematics.recomputeGeometry();


    
    Serial.println("Machine Settings Updated");
}

void  executeGcodeLine(String gcodeLine){
    /*
    
    Executes a single line of gcode beginning with the character 'G' or 'B'. If neither code is
    included on the front of the line, the code from the prevous line will be added.
    
    */
    
    int gNumber = extractGcodeValue(gcodeLine,'G', -1);
    
    if (gNumber != -1){                                     //If the line has a valid G number
        lastCommand = gNumber;                              //remember it for next time
    }
    else{                                                   //If the line does not have a gcommand
        gNumber = lastCommand;                              //apply the last one
    }
    
    switch(gNumber){
        case 0:
            G1(gcodeLine);
            break;
        case 1:
            G1(gcodeLine);
            break;
        case 2:
            G2(gcodeLine);
            break;
        case 3:
            G2(gcodeLine);
            break;
        case 10:
            G10(gcodeLine);
            break;
        case 20:
            setInchesToMillimetersConversion(INCHES);
            break;
        case 21:
            setInchesToMillimetersConversion(MILLIMETERS);
            break;
        case 90:
            useRelativeUnits = false;
            break;
        case 91:
            useRelativeUnits = true;
            break;
    }
    

    if(gcodeLine.substring(0, 3) == "B01"){
        
        leftAxis.computeMotorResponse();
        rightAxis.computeMotorResponse();
        
        gcodeLine = "";
        _signalReady();
        Serial.println("ready");
    }
    
    if(gcodeLine.substring(0, 3) == "B02"){
        calibrateChainLengths();
        gcodeLine = "";
        _signalReady();
        Serial.println("ready");
    }
    
    if(gcodeLine.substring(0, 3) == "B03"){
        updateSettings(gcodeLine);
        gcodeLine = "";
        _signalReady();
        Serial.println("ready");
    }
    
    if(gcodeLine.substring(0, 3) == "B04"){
        //Test each of the axis
        delay(500);
        leftAxis.test();
        delay(500);
        rightAxis.test();
        delay(500);
        zAxis.test();
        Serial.println("Tests complete.");
        gcodeLine = "";
        _signalReady();
        Serial.println("ready");
    }
    
    if(gcodeLine.substring(0, 3) == "B05"){
        Serial.print("Firmware Version ");
        Serial.println(VERSIONNUMBER);
        gcodeLine = "";
        _signalReady();
        Serial.println("ready");
    }
    
    if(gcodeLine.substring(0, 3) == "B06"){
        Serial.println("Manually Setting Chain Lengths To: ");
        float newL = extractGcodeValue(gcodeLine, 'L', 0);
        float newR = extractGcodeValue(gcodeLine, 'R', 0);
        
        leftAxis.set(newL);
        rightAxis.set(newR);
        
        Serial.print("Left: ");
        Serial.print(leftAxis.read());
        Serial.println("mm");
        Serial.print("Right: ");
        Serial.print(rightAxis.read());
        Serial.println("mm");
        
        Serial.println("Message: The machine chains have been manually re-calibrated.");
        
    }
    
    if(gcodeLine.substring(0, 3) == "B07"){
        //erase EEPROM
        leftAxis.wipeEEPROM();
        rightAxis.wipeEEPROM();
        zAxis.wipeEEPROM();
    }
    
    if(gcodeLine.substring(0, 3) == "B08"){
        //Manually recalibrate chain lengths
        leftAxis.set(ORIGINCHAINLEN);
        rightAxis.set(ORIGINCHAINLEN);
        
        Serial.print("Left: ");
        Serial.print(leftAxis.read());
        Serial.println("mm");
        Serial.print("Right: ");
        Serial.print(rightAxis.read());
        Serial.println("mm");
        
        Serial.println("Message: The machine chains have been manually re-calibrated.");
    }
    
    if((gcodeLine[0] == 'T' || gcodeLine[0] == 't') && gcodeLine[1] != 'e'){
        Serial.print("Please insert tool ");
        Serial.println(gcodeLine);
        gcodeLine = "";
    }
    
} 

int   findNextG(String readString, int startingPoint){
    int nextGIndex = readString.indexOf('G', startingPoint);
    if(nextGIndex == -1){
        nextGIndex = readString.length();
    }
    
    return nextGIndex;
}

void  interpretCommandString(String cmdString){
    /*
    
    Splits a string into lines of gcode which begin with 'G'
    
    */
    
    int firstG;  
    int secondG;
    
    if (cmdString[0] == 'B'){                   //If the command is a B command
        executeGcodeLine(cmdString);
        Serial.println(cmdString);
    }
    else{
        while(cmdString.length() > 0){          //Extract each line of gcode from the string
            firstG  = findNextG(cmdString, 0);
            secondG = findNextG(cmdString, firstG + 1);
            
            if(firstG == cmdString.length()){   //If the line contains no G letters
                firstG = 0;                     //send the whole line
            }
            
            String gcodeLine = cmdString.substring(firstG, secondG);
            
            Serial.println(gcodeLine);
            executeGcodeLine(gcodeLine);
            
            cmdString = cmdString.substring(secondG, cmdString.length());
            
        }
    }
    
    _signalReady();
    
}

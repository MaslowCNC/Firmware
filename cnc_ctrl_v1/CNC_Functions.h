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

#define VERSIONNUMBER 0.89

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
#define MAXFEED     900      //The maximum allowable feedrate in mm/min
#define MAXZROTMIN  12.60    // the maximum z rotations per minute


int ENCODER1A;
int ENCODER1B;
int ENCODER2A;
int ENCODER2B;
int ENCODER3A;
int ENCODER3B;

int IN1;
int IN2;
int IN3;
int IN4;
int IN5;
int IN6;

int ENA;
int ENB;
int ENC;

//These are set in Ground Control now
//#define DISTPERROT     10*6.35//#teeth*pitch of chain
//#define ZDISTPERROT    3.17//1/8inch in mm
//#define ENCODERSTEPS   8148.0 //7*291*4 --- 7ppr, 291:1 gear ratio, quadrature encoding
//#define ZENCODERSTEPS  7560.0 //7*270*4 --- 7ppr, 270:1 gear ratio, quadrature encoding

#define AUX1 17
#define AUX2 16
#define AUX3 15
#define AUX4 14
#define Probe AUX4 // use this input for zeroing zAxis with G38.2 gcode

int pcbRevisionIndicator = digitalRead(22);

int   setupPins(){
    /*
    
    Detect the version of the Arduino shield connected, and use the aproprate pins
    
    */
    
    if(pcbRevisionIndicator == 1){
        //Beta PCB v1.0 Detected
        ENCODER1A = 18;
        ENCODER1B = 19;
        ENCODER2A = 2;
        ENCODER2B = 3;
        ENCODER3A = 21;
        ENCODER3B = 20;

        IN1 = 9;
        IN2 = 8;
        IN3 = 11;
        IN4 = 10;
        IN5 = 12;
        IN6 = 13;

        ENA = 6;
        ENB = 7;
        ENC = 5;
        
        return 1;
    }
    else{
        //PCB v1.1 Detected
        ENCODER1A = 20;
        ENCODER1B = 21;
        ENCODER2A = 19;
        ENCODER2B = 18;
        ENCODER3A = 2;
        ENCODER3B = 3;

        IN1 = 6;
        IN2 = 4;
        IN3 = 9;
        IN4 = 7;
        IN5 = 10;
        IN6 = 11;

        ENA = 5;
        ENB = 8;
        ENC = 12;
        
        return 0;
    }
}

int pinsSetup       = setupPins();

Axis leftAxis (ENC, IN6, IN5, ENCODER3B, ENCODER3A, "L",  LEFT_EEPROM_ADR);
Axis rightAxis(ENA, IN1, IN2, ENCODER1A, ENCODER1B, "R", RIGHT_EEPROM_ADR);
Axis zAxis    (ENB, IN3, IN4, ENCODER2B, ENCODER2A, "Z",     Z_EEPROM_ADR);


Kinematics kinematics;
RingBuffer ringBuffer;

float feedrate              =  500;
float _inchesToMMConversion =  1;
bool  useRelativeUnits      =  false;
bool  stopFlag              =  false;
bool  pauseFlag             =  false;
bool  rcvdKinematicSettings =  false;
bool  rcvdMotorSettings     =  false;
bool  encoderStepsChanged   =  false;
bool  zEncoderStepsChanged  =  false;
// Commands that can safely be executed before machineReady
String safeCommands[] = {"B01", "B03", "B04", "B05", "B07", "B12", "G20", "G21", "G90", "G91"};
String readyCommandString;                //next command queued up and ready to send
String gcodeLine;                         //The next individual line of gcode (for example G91 G01 X19 would be run as two lines)

int   lastCommand           =  0;         //Stores the value of the last command run eg: G01 -> 1

float xTarget = 0;
float yTarget = 0;

bool machineReady(){
  bool ret = false;
  if (rcvdMotorSettings && rcvdKinematicSettings){
      ret = true;
  }
  return ret;
}

void finalizeMachineSettings(){
    if(machineReady()){
        if (encoderStepsChanged){
            leftAxis.loadPositionFromMemory();
            rightAxis.loadPositionFromMemory();
            encoderStepsChanged = false;
        }
        if (zEncoderStepsChanged){
            zAxis.loadPositionFromMemory();
            zEncoderStepsChanged = false;
        }
        kinematics.forward(leftAxis.read(), rightAxis.read(), &xTarget, &yTarget);
    }
}

void  returnError(){
    /*
    Prints the machine's positional error and the amount of space available in the 
    gcode buffer
    */
        Serial.print(F("[PE:"));
        Serial.print(leftAxis.error());
        Serial.print(',');
        Serial.print(rightAxis.error());
        Serial.print(',');
        Serial.print(ringBuffer.spaceAvailable());
        Serial.println(F("]"));
}

void  returnPoz(const float& x, const float& y, const float& z){
    /*
    Causes the machine's position (x,y) to be sent over the serial connection updated on the UI
    in Ground Control. Only executes if hasn't been called in at least timeout ms.
    */
    
    static unsigned long lastRan = millis();
    int                  timeout = 200;
    
    if (millis() - lastRan > timeout){
        
        
        Serial.print(F("<Idle,MPos:"));
        Serial.print(x/_inchesToMMConversion);
        Serial.print(F(","));
        Serial.print(y/_inchesToMMConversion);
        Serial.print(F(","));
        Serial.print(z/_inchesToMMConversion);
        Serial.println(F(",WPos:0.000,0.000,0.000>"));
        
        returnError();
        
        lastRan = millis();
    }
    
}

void  _signalReady(){
    /*
    
    Signal to the controlling software that the machine has executed the last
    gcode line successfully.
    
    */
    
    Serial.println(F("ok"));
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
            
            if (ringBuffer.length() > 0){                  //if there is stuff sitting in the buffer, run it
                ringBuffer.write('\n');
                Serial.println(F("watch dog catch"));
            }
            else{
                _signalReady();                          //request new code
                returnError();
            }
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
            pauseFlag = false;
        }
        else if (c == '~'){
            pauseFlag = false;
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
        readyCommandString = "";
        ringBuffer.empty();
        stopFlag = false;
        return 1;
    }
    return 0;
}

void  holdPosition(){
    leftAxis.hold();
    rightAxis.hold();
    zAxis.hold();
}

void pause(){
    /*
    
    The pause command pauses the machine in place without flushing the lines stored in the machine's
    buffer.
    
    When paused the machine enters a while() loop and doesn't exit until the '~' cycle resume command 
    is issued from Ground Control.
    
    */
    
    pauseFlag = true;
    Serial.println(F("Maslow Paused"));
    
    long timeLastPrinted = 0;
    while(1){
        
        holdPosition();
    
        readSerialCommands();
    
        returnPoz(xTarget, yTarget, zAxis.read());
        
        if (!pauseFlag){
            return;
        }
    }    
}

bool checkForProbeTouch(const int& probePin) {
  /*
      Check to see if AUX4 has gone LOW
  */
  if (digitalRead(probePin) == LOW) {
    readyCommandString = "";
    return 1;
  }
  return 0;
}

float calculateDelay(const float& stepSizeMM, const float& feedrateMMPerMin){
    /*
    Calculate the time delay between each step for a given feedrate
    */
    
    #define MINUTEINMS 60000.0
    
    // derivation: ms / step = 1 min in ms / dist in one min
    
    float msPerStep = (stepSizeMM*MINUTEINMS)/feedrateMMPerMin;
    
    return msPerStep;
}

float computeStepSize(const float& MMPerMin){
    /*
    
    Determines the minimum step size which can be taken for the given feed-rate
    and still have there be enough time for the kinematics to run
    
    */
    
    return .0001575*MMPerMin; //value found empirically by running loop until there were not spare cycles
}

int   cordinatedMove(const float& xEnd, const float& yEnd, const float& MMPerMin){
    
/*The move() function moves the tool in a straight line to the position (xEnd, yEnd) at 
the speed moveSpeed. Movements are correlated so that regardless of the distances moved in each 
direction, the tool moves to the target in a straight line. This function is used by the G00 
and G01 commands. The units at this point should all be in mm or mm per minute*/
    
    float  xStartingLocation = xTarget;
    float  yStartingLocation = yTarget;
    float  stepSizeMM         = computeStepSize(MMPerMin);
    
    //find the total distances to move
    float  distanceToMoveInMM         = sqrt(  sq(xEnd - xStartingLocation)  +  sq(yEnd - yStartingLocation)  );
    float  xDistanceToMoveInMM        = xEnd - xStartingLocation;
    float  yDistanceToMoveInMM        = yEnd - yStartingLocation;
    
    //compute the total  number of steps in the move
    //the argument to abs should only be a variable -- splitting calc into 2 lines
    long   finalNumberOfSteps         = distanceToMoveInMM/stepSizeMM;
    finalNumberOfSteps = abs(finalNumberOfSteps);
    
    float delayTime = calculateDelay(stepSizeMM, MMPerMin);
    
    // (fraction of distance in x direction)* size of step toward target
    float  xStepSize                  = (xDistanceToMoveInMM/distanceToMoveInMM)*stepSizeMM;
    float  yStepSize                  = (yDistanceToMoveInMM/distanceToMoveInMM)*stepSizeMM;
    
    //attach the axes
    leftAxis.attach();
    rightAxis.attach();
    
    float aChainLength;
    float bChainLength;
    long   numberOfStepsTaken         =  0;
    
    while(numberOfStepsTaken < finalNumberOfSteps){
            
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
        
        //wait for the step to be completed
        delay(delayTime);
    }
    
    kinematics.inverse(xEnd,yEnd,&aChainLength,&bChainLength);
    leftAxis.endMove(aChainLength);
    rightAxis.endMove(bChainLength);
    
    xTarget = xEnd;
    yTarget = yEnd;
    
    return 1;
    
}

void  singleAxisMove(Axis* axis, const float& endPos, const float& MMPerMin){
    /*
    Takes a pointer to an axis object and moves that axis to endPos at speed MMPerMin
    */
    
    float startingPos          = axis->read();
    float moveDist             = endPos - startingPos; //total distance to move
    
    float direction            = moveDist/abs(moveDist); //determine the direction of the move
    
    float stepSizeMM           = 0.01;                    //step size in mm

    //the argument to abs should only be a variable -- splitting calc into 2 lines
    long finalNumberOfSteps    = moveDist/stepSizeMM;      //number of steps taken in move
    finalNumberOfSteps = abs(finalNumberOfSteps);

    float delayTime = calculateDelay(stepSizeMM, MMPerMin);
    
    long numberOfStepsTaken    = 0;
    
    //attach the axis we want to move
    axis->attach();
    
    while(numberOfStepsTaken < finalNumberOfSteps){
        
        //find the target point for this step
        float whereAxisShouldBeAtThisStep = startingPos + numberOfStepsTaken*stepSizeMM*direction;
        
        //write to axis
        axis->write(whereAxisShouldBeAtThisStep);
        
        //update position on display
        returnPoz(xTarget, yTarget, zAxis.read());
        
        //calculate the correct delay between steps to set feedrate
        delay(delayTime);
        
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
    
int   findEndOfNumber(const String& textString, const int& index){
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
    
float extractGcodeValue(const String& readString, char target, const float& defaultReturn){

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

int   G1(const String& readString, int G0orG1){
    
/*G1() is the function which is called to process the string if it begins with 
'G01' or 'G00'*/
    
    float xgoto;
    float ygoto;
    float zgoto;
        
    float currentXPos = xTarget;
    float currentYPos = yTarget;
    
    float currentZPos = zAxis.target();
    
    xgoto      = _inchesToMMConversion*extractGcodeValue(readString, 'X', currentXPos/_inchesToMMConversion);
    ygoto      = _inchesToMMConversion*extractGcodeValue(readString, 'Y', currentYPos/_inchesToMMConversion);
    zgoto      = _inchesToMMConversion*extractGcodeValue(readString, 'Z', currentZPos/_inchesToMMConversion);
    feedrate   = _inchesToMMConversion*extractGcodeValue(readString, 'F', feedrate/_inchesToMMConversion);
    
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
    
    feedrate = constrain(feedrate, 1, MAXFEED);   //constrain the maximum feedrate, 35ipm = 900 mmpm
    
    //if the zaxis is attached
    if(zAxisAttached){
        float threshold = .01;
        if (abs(zgoto- currentZPos) > threshold){
            float zfeedrate;
            if (G0orG1 == 1) {
                zfeedrate = constrain(feedrate, 1, MAXZROTMIN * 3.17);//distance per rotation shouldn't be hard coded here
            }
            else {
                zfeedrate = MAXZROTMIN * 3.17;
            }
            singleAxisMove(&zAxis, zgoto, zfeedrate);
        }
    }
    else{
        float threshold = .1; //units of mm
        if (abs(currentZPos - zgoto) > threshold){
            Serial.print(F("Message: Please adjust Z-Axis to a depth of "));
            if (zgoto > 0){
                Serial.print(F("+"));
            }
            Serial.print(zgoto/_inchesToMMConversion);
            if (_inchesToMMConversion == INCHES){
                Serial.println(F(" in"));
            }
            else{
                Serial.println(F(" mm"));
            }
            
            pause(); //Wait until the z-axis is adjusted
            
            zAxis.set(zgoto);
            
            int    waitTimeMs = 1000;
            double startTime  = millis();
            
            while (millis() - startTime < waitTimeMs){
                delay(1);
                holdPosition();
            } 
        }
    }
    
    
    if (G0orG1 == 1){
        //if this is a regular move
        cordinatedMove(xgoto, ygoto, feedrate); //The XY move is performed
    }
    else{
        //if this is a rapid move
        cordinatedMove(xgoto, ygoto, 1000); //move the same as a regular move, but go fast
    }
}

int   arc(const float& X1, const float& Y1, const float& X2, const float& Y2, const float& centerX, const float& centerY, const float& MMPerMin, const float& direction){
    /*
    
    Move the machine through an arc from point (X1, Y1) to point (X2, Y2) along the 
    arc defined by center (centerX, centerY) at speed MMPerMin
    
    */
    
    //compute geometry 
    float pi                     =  3.1415;
    float radius                 =  sqrt( sq(centerX - X1) + sq(centerY - Y1) ); 
    float distanceBetweenPoints  =  sqrt( sq(  X2 - X1   ) + sq(    Y2  - Y1) );
    float circumference          =  2.0*pi*radius;
    
    float startingAngle          =  atan2(Y1 - centerY, X1 - centerX);
    float endingAngle            =  atan2(Y2 - centerY, X2 - centerX);
    
    //compute angle between lines
    float theta                  =  endingAngle - startingAngle;
    if (direction == COUNTERCLOCKWISE){
        if (theta <= 0){
            theta += 2*pi;
        }
    }
    else {
        //CLOCKWISE
        if (theta >= 0){
            theta -= 2*pi;
        }
    }
    
    float arcLengthMM            =  circumference * (theta / (2*pi) );
    
    //set up variables for movement
    long numberOfStepsTaken       =  0;
    
    float stepSizeMM             =  computeStepSize(MMPerMin);

    //the argument to abs should only be a variable -- splitting calc into 2 lines
    long   finalNumberOfSteps     =  arcLengthMM/stepSizeMM;
    //finalNumberOfSteps = abs(finalNumberOfSteps);
    
    //Compute the starting position
    float angleNow = startingAngle;
    float whereXShouldBeAtThisStep = radius * cos(angleNow) + centerX;
    float whereYShouldBeAtThisStep = radius * sin(angleNow) + centerY;
    float degreeComplete = 0.0;
    
    float aChainLength;
    float bChainLength;

    float delayTime = calculateDelay(stepSizeMM, MMPerMin);
    
    //attach the axes
    leftAxis.attach();
    rightAxis.attach();
    
    long  beginingOfLastStep          = millis();
    
    while(numberOfStepsTaken < abs(finalNumberOfSteps)){
        
        //if enough time has passed to take the next step
        if (millis() - beginingOfLastStep > delayTime){
            
            //reset the counter 
            beginingOfLastStep          = millis();
            
            degreeComplete = float(numberOfStepsTaken)/float(finalNumberOfSteps);
            
            angleNow = startingAngle + theta*direction*degreeComplete;
            
            whereXShouldBeAtThisStep = radius * cos(angleNow) + centerX;
            whereYShouldBeAtThisStep = radius * sin(angleNow) + centerY;
            
            kinematics.inverse(whereXShouldBeAtThisStep,whereYShouldBeAtThisStep,&aChainLength,&bChainLength);
            
            leftAxis.write(aChainLength);
            rightAxis.write(bChainLength);
            
            returnPoz(whereXShouldBeAtThisStep, whereYShouldBeAtThisStep, zAxis.read());
            
            numberOfStepsTaken++;
            
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
    
    kinematics.inverse(X2,Y2,&aChainLength,&bChainLength);
    leftAxis.endMove(aChainLength);
    rightAxis.endMove(bChainLength);
    
    xTarget = X2;
    yTarget = Y2;
    
    return 1;
}

int   G2(const String& readString, int G2orG3){
    /*
    
    The G2 function handles the processing of the gcode line for both the command G2 and the
    command G3 which cut arcs.
    
    */
    
    
    float X1 = xTarget; //does this work if units are inches? (It seems to)
    float Y1 = yTarget;
    
    float X2      = _inchesToMMConversion*extractGcodeValue(readString, 'X', X1/_inchesToMMConversion);
    float Y2      = _inchesToMMConversion*extractGcodeValue(readString, 'Y', Y1/_inchesToMMConversion);
    float I       = _inchesToMMConversion*extractGcodeValue(readString, 'I', 0.0);
    float J       = _inchesToMMConversion*extractGcodeValue(readString, 'J', 0.0);
    feedrate      = _inchesToMMConversion*extractGcodeValue(readString, 'F', feedrate/_inchesToMMConversion);
    
    float centerX = X1 + I;
    float centerY = Y1 + J;
    
    feedrate = constrain(feedrate, 1, MAXFEED);   //constrain the maximum feedrate, 35ipm = 900 mmpm
    
    if (G2orG3 == 2){
        arc(X1, Y1, X2, Y2, centerX, centerY, feedrate, CLOCKWISE);
    }
    if (G2orG3 == 3){
        arc(X1, Y1, X2, Y2, centerX, centerY, feedrate, COUNTERCLOCKWISE);
    }
}

void  G10(const String& readString){
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

void  G38(const String& readString) {
  //if the zaxis is attached
  if (zAxisAttached) {
    /*
       The G38() function handles the G38 gcode which zeros the machine's z axis.
       Currently ignores X and Y options
    */
    if (readString.substring(3, 5) == ".2") {
      Serial.println(F("probing for z axis zero"));
      float zgoto;


      float currentZPos = zAxis.target();

      zgoto      = _inchesToMMConversion * extractGcodeValue(readString, 'Z', currentZPos / _inchesToMMConversion);
      feedrate   = _inchesToMMConversion * extractGcodeValue(readString, 'F', feedrate / _inchesToMMConversion);
      feedrate = constrain(feedrate, 1, MAXZROTMIN * 3.17);

      if (useRelativeUnits) { //if we are using a relative coordinate system
        if (readString.indexOf('Z') >= 0) { //if z has moved
          zgoto = currentZPos + zgoto;
        }
      }

      Serial.print(F("max depth "));
      Serial.print(zgoto);
      Serial.println(F(" mm."));
      Serial.print(F("feedrate "));
      Serial.print(feedrate);
      Serial.println(F(" mm per min."));


      //set Probe to input with pullup
      pinMode(Probe, INPUT_PULLUP);
      digitalWrite(Probe,   HIGH);

      if (zgoto != currentZPos / _inchesToMMConversion) {
        //        now move z to the Z destination;
        //        Currently ignores X and Y options
        //          we need a version of singleAxisMove that quits if the AUXn input changes (goes LOW)
        //          which will act the same as the checkForStopCommand() found in singleAxisMove (need both?)
        //        singleAxisMove(&zAxis, zgoto, feedrate);

        /*
           Takes a pointer to an axis object and mo ves that axis to endPos at speed MMPerMin
        */

        Axis* axis = &zAxis;
        float MMPerMin             = feedrate;
        float startingPos          = axis->target();
        float endPos               = zgoto;
        float moveDist             = endPos - currentZPos; //total distance to move

        float direction            = moveDist / abs(moveDist); //determine the direction of the move

        float stepSizeMM           = 0.01;                    //step size in mm

        //the argument to abs should only be a variable -- splitting calc into 2 lines
        long finalNumberOfSteps    = moveDist / stepSizeMM;    //number of steps taken in move
        finalNumberOfSteps = abs(finalNumberOfSteps);

        float delayTime = calculateDelay(stepSizeMM, MMPerMin);

        long numberOfStepsTaken    = 0;
        long  beginingOfLastStep   = millis();
  
        axis->attach();
        //  zAxis->attach();

        while (numberOfStepsTaken < finalNumberOfSteps) {

          //reset the counter
          beginingOfLastStep          = millis();

          //find the target point for this step
          float whereAxisShouldBeAtThisStep = startingPos + numberOfStepsTaken * stepSizeMM * direction;

          //write to each axis
          axis->write(whereAxisShouldBeAtThisStep);

          //update position on display
          returnPoz(xTarget, yTarget, zAxis.read());

          //calculate the correct delay between steps to set feedrate
          delay(delayTime);

          //increment the number of steps taken
          numberOfStepsTaken++;

          //check for new serial commands
          readSerialCommands();

          //check for a STOP command
          if (checkForStopCommand()) {
            axis->endMove(whereAxisShouldBeAtThisStep);
            return;
          }

          //check for Probe touchdown
          if (checkForProbeTouch(Probe)) {
            zAxis.set(0);
            zAxis.endMove(0);
            zAxis.attach();
            Serial.println(F("z axis zeroed"));
            return;
          }
        }

        /*
           If wen get here, the probe failed to touch down
            - print error
            - STOP execution
        */
        axis->endMove(endPos);
        Serial.println(F("error: probe did not connect\nprogram stopped\nz axis not set\n"));
        stopFlag = true;
        checkForStopCommand();

      } // end if zgoto != currentZPos / _inchesToMMConversion

    } else {
      Serial.print(F("G38"));
      Serial.print(readString.substring(3, 5));
      Serial.println(F(" is invalid. Only G38.2 recognized."));
    }
  } else {
    Serial.println(F("G38.2 gcode only valid with z-axis attached"));
  }
}

void  calibrateChainLengths(){
    /*
    The calibrateChainLengths function lets the machine know that the chains are set to a given length where each chain is ORIGINCHAINLEN
    in length
    */
    
    
    //measure out the left chain
    Serial.println(F("Measuring out left chain"));
    leftAxis.setPIDAggressiveness(.1);
    singleAxisMove(&leftAxis, ORIGINCHAINLEN, 500);
    
    Serial.print(leftAxis.read());
    Serial.println(F("mm"));
    
    leftAxis.detach();
    
    //measure out the right chain
    Serial.println(F("Measuring out right chain"));
    rightAxis.setPIDAggressiveness(.1);
    singleAxisMove(&rightAxis, ORIGINCHAINLEN, 500);
    
    Serial.print(rightAxis.read());
    Serial.println(F("mm"));
    
    kinematics.forward(leftAxis.read(), rightAxis.read(), &xTarget, &yTarget);
    
    
    leftAxis.setPIDAggressiveness(1);
    rightAxis.setPIDAggressiveness(1);
}

void  setInchesToMillimetersConversion(float newConversionFactor){
    _inchesToMMConversion = newConversionFactor;
}

void  printBeforeAndAfter(const float& before, const float& after){
    Serial.print(F("Before: "));
    Serial.print(before);
    Serial.print(F(" After: "));
    Serial.println(after);
}

void  updateKinematicsSettings(const String& readString){
    /*
    Updates the machine dimensions from the Ground Control settings
    */
    
    //Extract the settings values

    float bedWidth           = extractGcodeValue(readString, 'A', -1);
    float bedHeight          = extractGcodeValue(readString, 'C', -1);
    float distBetweenMotors  = extractGcodeValue(readString, 'Q', -1);
    float motorOffsetY       = extractGcodeValue(readString, 'E', -1);
    float sledWidth          = extractGcodeValue(readString, 'F', -1);
    float sledHeight         = extractGcodeValue(readString, 'R', -1);
    float sledCG             = extractGcodeValue(readString, 'H', -1);

    float kinematicsType     = extractGcodeValue(readString, 'Y', -1);
    float rotationDiskRadius = extractGcodeValue(readString, 'Z', -1);
    
    
    
    //Change the machine dimensions in the kinematics if new values have been received
    if (sledWidth != -1){
        kinematics.l            = sledWidth;
    }
    if (sledHeight != -1){
        kinematics.s            = sledHeight;
    }
    if (sledCG != -1){
        kinematics.h3           = sledCG;
    }
    //if (distPerRot != -1){
    //    kinematics.R            = distPerRot / (2.0*3.14159);
    //}
    if (distBetweenMotors != -1){
        kinematics.D            = distBetweenMotors;
    }
    if (motorOffsetY != -1){
        kinematics.motorOffsetY = motorOffsetY;
    }
    if (bedWidth != -1){
        kinematics.machineWidth = bedWidth;
    }
    if (bedHeight != -1){
        kinematics.machineHeight= bedHeight;
    }
    if (kinematicsType != -1){
        kinematics.kinematicsType = kinematicsType;
    }
    if (rotationDiskRadius != -1){
        kinematics.rotationDiskRadius = rotationDiskRadius;
    }
    
    //propagate the new values
    rcvdKinematicSettings = true;
    kinematics.recomputeGeometry();
    finalizeMachineSettings();
    
    Serial.println(F("Kinematics Settings Loaded"));
}

void updateMotorSettings(const String& readString){
    /*
    
    Update settings related to the motor configurations
    
    */
    
    if (extractGcodeValue(readString, 'I', -1) != -1){
        zAxisAttached            = extractGcodeValue(readString, 'I', -1);
    }
    int encoderSteps         = extractGcodeValue(readString, 'J', -1);
    float gearTeeth          = extractGcodeValue(readString, 'K', -1);
    float chainPitch         = extractGcodeValue(readString, 'M', -1);
    
    float zDistPerRot        = extractGcodeValue(readString, 'N', -1);
    int zEncoderSteps        = extractGcodeValue(readString, 'P', -1);
    
    float KpPos              = extractGcodeValue(readString, 'S', -1);
    float KiPos              = extractGcodeValue(readString, 'T', -1);
    float KdPos              = extractGcodeValue(readString, 'U', -1);
    float KpV                = extractGcodeValue(readString, 'V', -1);
    float KiV                = extractGcodeValue(readString, 'W', -1);
    float KdV                = extractGcodeValue(readString, 'X', -1);
      
    //Write the PID values to the axis if new ones have been received
    if (KpPos != -1){
        leftAxis.setPIDValues(KpPos, KiPos, KdPos, KpV, KiV, KdV);
        rightAxis.setPIDValues(KpPos, KiPos, KdPos, KpV, KiV, KdV);
        zAxis.setPIDValues(KpPos, KiPos, KdPos, KpV, KiV, KdV);
    }
    
    //Change the motor properties in cnc_funtions if new values have been sent
    float distPerRot = -1;
    if (gearTeeth != -1 and chainPitch != -1){
        float distPerRot = gearTeeth*chainPitch; 
        leftAxis.changePitch(distPerRot);
        rightAxis.changePitch(distPerRot);
        zAxis.changePitch(zDistPerRot);
    }
    
    //update the number of encoder steps if new values have been received
    if (encoderSteps != -1){
        leftAxis.changeEncoderResolution(encoderSteps);
        rightAxis.changeEncoderResolution(encoderSteps);
        encoderStepsChanged = true;
    }
    if (zEncoderSteps != -1){
        zAxis.changeEncoderResolution(zEncoderSteps);
        zEncoderStepsChanged = true;
    }
    
    rcvdMotorSettings = true;
    finalizeMachineSettings(); 
    Serial.println(F("Motor Settings Loaded"));
}

bool isSafeCommand(const String& readString){
    bool ret = false;
    String command = readString.substring(0, 3);
    for(int i = 0; i < sizeof(safeCommands); i++){
       if(safeCommands[i] == command){
           ret = true;
           break;
       }
    }
    return ret;
}

void  executeGcodeLine(const String& gcodeLine){
    /*
    
    Executes a single line of gcode beginning with the character 'G' or 'B'. If neither code is
    included on the front of the line, the code from the prevous line will be added.
    
    */
    
    if (!machineReady() && !isSafeCommand(gcodeLine)){
        Serial.println(F("Unable to execute command, machine settings not yet received"));
        return;
    }
    
    //Handle B-codes
    
    if(gcodeLine.substring(0, 3) == "B01"){
        
        Serial.println(F("Motor Calibration Not Needed"));
        
        return;
    }
    
    if(gcodeLine.substring(0, 3) == "B02"){
        calibrateChainLengths();
        return;
    }
    
    if(gcodeLine.substring(0, 3) == "B03"){
        updateKinematicsSettings(gcodeLine);
        return;
    }
    
    if(gcodeLine.substring(0, 3) == "B04"){
        //Test each of the axis
        delay(500);
        leftAxis.test();
        delay(500);
        rightAxis.test();
        delay(500);
        zAxis.test();
        Serial.println(F("Tests complete."));
        return;
    }
    
    if(gcodeLine.substring(0, 3) == "B05"){
        Serial.print(F("Firmware Version "));
        Serial.println(VERSIONNUMBER);
        return;
    }
    
    if(gcodeLine.substring(0, 3) == "B06"){
        Serial.println(F("Setting Chain Lengths To: "));
        float newL = extractGcodeValue(gcodeLine, 'L', 0);
        float newR = extractGcodeValue(gcodeLine, 'R', 0);
        
        leftAxis.set(newL);
        rightAxis.set(newR);
        
        Serial.print(F("Left: "));
        Serial.print(leftAxis.read());
        Serial.println(F("mm"));
        Serial.print(F("Right: "));
        Serial.print(rightAxis.read());
        Serial.println(F("mm"));
        
        return;
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
        
        Serial.print(F("Left: "));
        Serial.print(leftAxis.read());
        Serial.println(F("mm"));
        Serial.print(F("Right: "));
        Serial.print(rightAxis.read());
        Serial.println(F("mm"));
        
        kinematics.forward(leftAxis.read(), rightAxis.read(), &xTarget, &yTarget);
        
        Serial.println(F("Message: The machine chains have been manually re-calibrated."));
        
        return;
    }
    
    if(gcodeLine.substring(0, 3) == "B09"){
        //Directly command each axis to move to a given distance
        float lDist = extractGcodeValue(gcodeLine, 'L', 0);
        float rDist = extractGcodeValue(gcodeLine, 'R', 0);
		float speed = extractGcodeValue(gcodeLine, 'F', 500);
        
        leftAxis.setPIDAggressiveness(.1);
        rightAxis.setPIDAggressiveness(.1);
        
        if(useRelativeUnits){
            if(abs(lDist) > 0){
                singleAxisMove(&leftAxis,  leftAxis.read()  + lDist, speed);
            }
            if(abs(rDist) > 0){
                singleAxisMove(&rightAxis, rightAxis.read() + rDist, speed);
            }
        }
        else{
            singleAxisMove(&leftAxis,  lDist, speed);
            singleAxisMove(&rightAxis, rDist, speed);
        }
        
        leftAxis.setPIDAggressiveness(1);
        rightAxis.setPIDAggressiveness(1);
        
        return;
    }
    
    if(gcodeLine.substring(0, 3) == "B10"){
        //measure the left axis chain length
        Serial.print(F("[Measure: "));
        Serial.print(leftAxis.read());
        Serial.println(F("]"));
        return;
    }
    
    if(gcodeLine.substring(0, 3) == "B11"){
        //run right motor in the given direction at the given speed for the given time
        float  speed      = extractGcodeValue(gcodeLine, 'S', 100);
        float  time       = extractGcodeValue(gcodeLine, 'T', 1);
        
        double ms    = 1000*time;
        double begin = millis();
        
        int i = 0;
        while (millis() - begin < ms){
            leftAxis.motorGearboxEncoder.motor.directWrite(speed);
            if (i % 10000 == 0){
                Serial.println(F("pulling"));                              //Keep the connection from timing out
            }
            i++;
        }
        leftAxis.set(leftAxis.read());
        return;
    }
    
    if(gcodeLine.substring(0, 3) == "B12"){
        //Update the motor characteristics
        updateMotorSettings(gcodeLine);
        return;
    }
    
    //Handle G-Codes
   
    int gNumber = extractGcodeValue(gcodeLine,'G', -1);
    
    if (gNumber != -1){                                     //If the line has a valid G number
        lastCommand = gNumber;                              //remember it for next time
    }
    else{                                                   //If the line does not have a gcommand
        gNumber = lastCommand;                              //apply the last one
    }
    
    switch(gNumber){
        case 0:
            G1(gcodeLine, gNumber);
            break;
        case 1:
            G1(gcodeLine, gNumber);
            break;
        case 2:
            G2(gcodeLine, gNumber);
            break;
        case 3:
            G2(gcodeLine, gNumber);
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
        case 38:
            G38(gcodeLine);
            break;
        case 90:
            useRelativeUnits = false;
            break;
        case 91:
            useRelativeUnits = true;
            break;
        default:
            if(gcodeLine[0] != 'B'){
                Serial.print(F("Command G"));
                Serial.print(gNumber);
                Serial.println(F(" unsupported and ignored."));
            }
    }
    
    if((gcodeLine[0] == 'T' || gcodeLine[0] == 't') && gcodeLine[1] != 'e'){
        Serial.print(F("Please insert tool "));
        Serial.println(gcodeLine);
        gcodeLine = "";
    }
    
} 

int   findNextG(const String& readString, const int& startingPoint){
    int nextGIndex = readString.indexOf('G', startingPoint);
    if(nextGIndex == -1){
        nextGIndex = readString.length();
    }
    
    return nextGIndex;
}

void  interpretCommandString(const String& cmdString){
    /*
    
    Splits a string into lines of gcode which begin with 'G'
    
    */
    
    returnError();  //Cue up sending the next line
    
    int firstG;  
    int secondG;
    String cmdStringTrim = cmdString;
    cmdStringTrim.trim();
    if (cmdStringTrim.length() <= 0){
        // Nothing to process, likely a blank startup line
    }
    else if (cmdString[0] == 'B'){                   //If the command is a B command
        Serial.print(cmdString);
        executeGcodeLine(cmdString);
    }
    else{
        while(cmdString.length() > 0){          //Extract each line of gcode from the string
            firstG  = findNextG(cmdString, 0);
            secondG = findNextG(cmdString, firstG + 1);
            
            if(firstG == cmdString.length()){   //If the line contains no G letters
                firstG = 0;                     //send the whole line
            }
            
            gcodeLine = cmdString.substring(firstG, secondG);
            
            if (gcodeLine.length() > 1){
                Serial.println(gcodeLine);
                executeGcodeLine(gcodeLine);
            }
            
            cmdString = cmdString.substring(secondG, cmdString.length());
            
        }
    }
    
    _signalReady();
    
}

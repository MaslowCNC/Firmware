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

#define VERSIONNUMBER 1.00

#define verboseDebug 0    // set to 0 for no debug messages, 1 for single-line messages, 2 to also output ring buffer contents
#define misloopDebug 0    // set to 1 for a warning every time the movement loop fails 
                          // to complete before being interrupted, helpful for loop
                          // LOOPINTERVAL tuning

#include <Servo.h>
Servo myservo;  // create servo object to control a servo 

bool zAxisAttached = false;
bool zAxisAuto = false;

#define FORWARD           1
#define BACKWARD         -1

#define CLOCKWISE        -1
#define COUNTERCLOCKWISE  1

#define LEFT_EEPROM_ADR     5
#define RIGHT_EEPROM_ADR  105
#define Z_EEPROM_ADR      205

#define MILLIMETERS 1
#define INCHES      25.4
#define MAXFEED     1000      //The maximum allowable feedrate in mm/min
#define MAXZROTMIN  12.60    // the maximum z rotations per minute
#define LOOPINTERVAL 10000     // What is the frequency of the PID loop in microseconds

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
//#define ENCODERSTEPS   8148.0 //7*291*4 --- 7ppr, 291:1 gear ratio, quadrature encoding
//#define ZENCODERSTEPS  7560.0 //7*270*4 --- 7ppr, 270:1 gear ratio, quadrature encoding

#define AUX1 17
#define AUX2 16
#define AUX3 15
#define AUX4 14
#define SpindlePowerControlPin AUX1 // output for controlling spindle power
#define ProbePin AUX4 // use this input for zeroing zAxis with G38.2 gcode

int pcbVersion = -1;


int   setupPins(){
    /*
    
    Detect the version of the Arduino shield connected, and use the appropriate pins
    
    This function runs before the serial port is open so the version is not printed here
    
    */
    
    //read the pins which indicate the PCB version
    pcbVersion = (8*digitalRead(53) + 4*digitalRead(52) + 2*digitalRead(23) + 1*digitalRead(22)) - 1;
    
    if(pcbVersion == 0){
        //Beta PCB v1.0 Detected
        //MP1 - Right Motor
        ENCODER1A = 18; // INPUT
        ENCODER1B = 19; // INPUT
        IN1 = 9;        // OUTPUT
        IN2 = 8;        // OUTPUT
        ENA = 6;        // PWM
        
        //MP2 - Z-axis
        ENCODER2A = 2;  // INPUT
        ENCODER2B = 3;  // INPUT
        IN3 = 11;       // OUTPUT
        IN4 = 10;       // OUTPUT
        ENB = 7;        // PWM
        
        //MP3 - Left Motor
        ENCODER3A = 21; // INPUT
        ENCODER3B = 20; // INPUT
        IN5 = 12;       // OUTPUT
        IN6 = 13;       // OUTPUT
        ENC = 5;        // PWM
        
        return 1;
    }
    else if(pcbVersion == 1){
        //PCB v1.1 Detected
        //MP1 - Right Motor
        ENCODER1A = 20; // INPUT
        ENCODER1B = 21; // INPUT
        IN1 = 6;        // OUTPUT
        IN2 = 4;        // OUTPUT
        ENA = 5;        // PWM
        
        //MP2 - Z-axis
        ENCODER2A = 19; // INPUT
        ENCODER2B = 18; // INPUT
        IN3 = 9;        // OUTPUT
        IN4 = 7;        // OUTPUT
        ENB = 8;        // PWM
        
        //MP3 - Left Motor
        ENCODER3A = 2;   // INPUT
        ENCODER3B = 3;   // INPUT
        IN5 = 10;        // OUTPUT
        IN6 = 11;        // OUTPUT
        ENC = 12;        // PWM
        
        return 0;
    }
    else if(pcbVersion == 2){
        //PCB v1.2 Detected
        
        //MP1 - Right Motor
        ENCODER1A = 20;  // INPUT
        ENCODER1B = 21;  // INPUT
        IN1 = 4;         // OUTPUT
        IN2 = 6;         // OUTPUT
        ENA = 5;         // PWM
        
        //MP2 - Z-axis
        ENCODER2A = 19;  // INPUT
        ENCODER2B = 18;  // INPUT
        IN3 = 7;         // OUTPUT
        IN4 = 9;         // OUTPUT
        ENB = 8;         // PWM
        
        //MP3 - Left Motor
        ENCODER3A = 2;   // INPUT
        ENCODER3B = 3;   // INPUT
        IN5 = 11;        // OUTPUT
        IN6 = 12;        // OUTPUT
        ENC = 10;        // PWM
        
        return 0;
    }
}

int pinsSetup       = setupPins();

Axis leftAxis (ENC, IN6, IN5, ENCODER3B, ENCODER3A, 'L',  LEFT_EEPROM_ADR, LOOPINTERVAL);
Axis rightAxis(ENA, IN1, IN2, ENCODER1A, ENCODER1B, 'R', RIGHT_EEPROM_ADR, LOOPINTERVAL);
Axis zAxis    (ENB, IN3, IN4, ENCODER2B, ENCODER2A, 'Z',     Z_EEPROM_ADR, LOOPINTERVAL);


Kinematics kinematics;
RingBuffer ringBuffer;

int expectedMaxLineLength   = 60;   // expected maximum Gcode line length in characters, including line ending character(s)

float feedrate              =  500;
float _inchesToMMConversion =  1;
bool  useRelativeUnits      =  false;
bool  stopFlag              =  false;
bool  pauseFlag             =  false;
bool  rcvdKinematicSettings =  false;
bool  rcvdMotorSettings     =  false;
bool  encoderStepsChanged   =  false;
bool  zEncoderStepsChanged  =  false;
volatile bool  movementUpdated  =  false;

// Global variables for misloop tracking
#if misloopDebug > 0
volatile bool  inMovementLoop   =  false;
volatile bool  movementFail     =  false;
#endif

// Commands that can safely be executed before machineReady
String safeCommands[] = {"B01", "B03", "B04", "B05", "B07", "B12", "G20", "G21", "G90", "G91"};
String readyCommandString;                //next command queued up and ready to send
String gcodeLine;                         //The next individual line of gcode (for example G91 G01 X19 would be run as two lines)

int   lastCommand           =  0;         //Stores the value of the last command run eg: G01 -> 1
int   lastTool              =  0;         //Stores the value of the last tool number eg: T4 -> 4
int   nextTool              =  0;         //Stores the value of the next tool number eg: T4 -> 4

float xTarget = 0;
float yTarget = 0;


bool checkForStopCommand();

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
    unsigned int         timeout = 200;
    
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
    
    if ( (ringBuffer.spaceAvailable() > expectedMaxLineLength)    // if there is space in the buffer to accept the expected maximum line length
          && (ringBuffer.numberOfLines() < 4) ) {                 // and if there are fewer than 4 lines in the buffer
        Serial.println(F("ok"));                                  // then request new code
    }
}

void  _watchDog(){
    /*
    Watchdog tells ground control that the machine is ready every second. _watchDog() should only be called when 
    the machine is actually ready.
    
    This fixes the issue where the machine is ready, but Ground Control doesn't know the machine is ready and the system locks up.
    */
    static unsigned long lastRan = millis();
    unsigned long        timeout = 5000;
    
    if ((millis() - lastRan) > timeout){
        
        if (!leftAxis.attached() and !rightAxis.attached() and !zAxis.attached()){
            
            if (ringBuffer.length() == 0) {       // if the buffer is empty
                #if defined (verboseDebug) && verboseDebug > 0              
                Serial.println(F("_watchDog requesting new code"));
                #endif
                _signalReady();                   // request new code
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
    if (Serial.available() > 0) {
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
                int bufferOverflow = ringBuffer.write(c); //gets one byte from serial buffer, writes it to the internal ring buffer
                if (bufferOverflow != 0) {
                  stopFlag = true;
                  checkForStopCommand();
                }
            }
        }
        #if defined (verboseDebug) && verboseDebug > 1              
        // print ring buffer contents
        Serial.println(F("rSC added to ring buffer"));
        ringBuffer.print();        
        #endif
    }
}

bool checkForStopCommand(){
    /*
    Check to see if the STOP command has been sent to the machine.
    If it has, empty the buffer, stop all axes, set target position to current 
    position and return true.
    */
    if(stopFlag){
        readyCommandString = "";
        ringBuffer.empty();
        leftAxis.stop();
        rightAxis.stop();
        if(zAxisAttached){
          zAxis.stop();
        }
        kinematics.forward(leftAxis.read(), rightAxis.read(), &xTarget, &yTarget);
        stopFlag = false;
        return true;
    }
    return false;
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
    
    while(1){
        
        holdPosition();
    
        readSerialCommands();
    
        returnPoz(xTarget, yTarget, zAxis.read());
        
        if (!pauseFlag){
            return;
        }
    }    
}

void maslowDelay(unsigned long waitTimeMs) {
  /*
   * Provides a time delay while holding the machine position, reading serial commands,
   * and periodically sending the machine position to Ground Control.  This prevents
   * Ground Control from thinking that the connection is lost.
   * 
   * This is similar to the pause() command above, but provides a time delay rather than
   * waiting for the user (through Ground Control) to tell the machine to continue.
   */
   
    unsigned long startTime  = millis();
    
    while ((millis() - startTime) < waitTimeMs){
        delay(1);
        holdPosition();
        readSerialCommands();
        returnPoz(xTarget, yTarget, zAxis.read());
    }
}

bool checkForProbeTouch(const int& probePin) {
  /*
      Check to see if ProbePin has gone LOW
  */
  if (digitalRead(probePin) == LOW) {
    readyCommandString = "";
    return 1;
  }
  return 0;
}

float calculateDelay(const float& stepSizeMM, const float& feedrateMMPerMin){
    /*
    Calculate the time delay in microseconds between each step for a given feedrate
    */
    
    return LOOPINTERVAL;
}

float calculateFeedrate(const float& stepSizeMM, const float& usPerStep){
    /*
    Calculate the time delay between each step for a given feedrate
    */
    
    #define MINUTEINUS 60000000.0
    
    // derivation: ms / step = 1 min in ms / dist in one min
    
    float feedrate = (stepSizeMM*MINUTEINUS)/usPerStep;
    
    return feedrate;
}

float computeStepSize(const float& MMPerMin){
    /*
    
    Determines the minimum step size which can be taken for the given feed-rate
    based on the loop interval frequency.  Converts to MM per microsecond first,
    then mutiplies by the number of microseconds in each loop interval
    
    */
    return LOOPINTERVAL*(MMPerMin/(60 * 1000000));
}
 
void movementUpdate(){
  #if misloopDebug > 0
  if (movementFail){
    Serial.println("Movement loop failed to complete before interrupt.");
    movementFail = false;
  }
  #endif
  movementUpdated = true;
}

int   coordinatedMove(const float& xEnd, const float& yEnd, const float& zEnd, float MMPerMin){
    
    /*The move() function moves the tool in a straight line to the position (xEnd, yEnd) at 
    the speed moveSpeed. Movements are correlated so that regardless of the distances moved in each 
    direction, the tool moves to the target in a straight line. This function is used by the G00 
    and G01 commands. The units at this point should all be in mm or mm per minute*/
    
    float  xStartingLocation = xTarget;
    float  yStartingLocation = yTarget;
    float  zStartingLocation = zAxis.read();  // I don't know why we treat the zaxis differently
    float  zMAXFEED          = MAXZROTMIN * abs(zAxis.getPitch());
    
    //find the total distances to move
    float  distanceToMoveInMM         = sqrt(  sq(xEnd - xStartingLocation)  +  sq(yEnd - yStartingLocation)  + sq(zEnd - zStartingLocation));
    float  xDistanceToMoveInMM        = xEnd - xStartingLocation;
    float  yDistanceToMoveInMM        = yEnd - yStartingLocation;
    float  zDistanceToMoveInMM        = zEnd - zStartingLocation;
    
    //compute feed details
    MMPerMin = constrain(MMPerMin, 1, MAXFEED);   //constrain the maximum feedrate, 35ipm = 900 mmpm
    float  stepSizeMM           = computeStepSize(MMPerMin);
    float  finalNumberOfSteps   = abs(distanceToMoveInMM/stepSizeMM);
    float  delayTime            = calculateDelay(stepSizeMM, MMPerMin);
    float  zFeedrate            = calculateFeedrate(abs(zDistanceToMoveInMM/finalNumberOfSteps), delayTime);
    
    //throttle back federate if it exceeds zaxis max
    if (zFeedrate > zMAXFEED){
      float  zStepSizeMM        = computeStepSize(zMAXFEED);
      finalNumberOfSteps        = abs(zDistanceToMoveInMM/zStepSizeMM);
      stepSizeMM                = (distanceToMoveInMM/finalNumberOfSteps);
      MMPerMin                  = calculateFeedrate(stepSizeMM, delayTime);
    }
    
    // (fraction of distance in x direction)* size of step toward target
    float  xStepSize            = (xDistanceToMoveInMM/finalNumberOfSteps);
    float  yStepSize            = (yDistanceToMoveInMM/finalNumberOfSteps);
    float  zStepSize            = (zDistanceToMoveInMM/finalNumberOfSteps);
    
    //attach the axes
    leftAxis.attach();
    rightAxis.attach();
    if(zAxisAttached){
      zAxis.attach();
    }
    
    float aChainLength;
    float bChainLength;
    float zPosition                   = zStartingLocation;
    float whereXShouldBeAtThisStep    = xStartingLocation;
    float whereYShouldBeAtThisStep    = yStartingLocation;
    long   numberOfStepsTaken         =  0;
    
    while(numberOfStepsTaken < finalNumberOfSteps){
      
        #if misloopDebug > 0
        inMovementLoop = true;
        #endif
        //if last movment was performed start the next
        if (!movementUpdated) {
            //find the target point for this step
            // This section ~20us
            whereXShouldBeAtThisStep +=  xStepSize;
            whereYShouldBeAtThisStep +=  yStepSize;
            zPosition += zStepSize;
            
            //find the chain lengths for this step
            // This section ~180us
            kinematics.inverse(whereXShouldBeAtThisStep,whereYShouldBeAtThisStep,&aChainLength,&bChainLength);
            
            //write to each axis
            // This section ~180us
            leftAxis.write(aChainLength);
            rightAxis.write(bChainLength);
            if(zAxisAttached){
              zAxis.write(zPosition);
            }
            
            movementUpdate();
            
            //increment the number of steps taken
            numberOfStepsTaken++;
            
            //update position on display
            returnPoz(whereXShouldBeAtThisStep, whereYShouldBeAtThisStep, zPosition);
            
            // This section ~10us
            //check for new serial commands
            readSerialCommands();
            
            //check for a STOP command
            if(checkForStopCommand()){
                return 1;
            }
        }
    }
    #if misloopDebug > 0
    inMovementLoop = false;
    #endif
    
    kinematics.inverse(xEnd,yEnd,&aChainLength,&bChainLength);
    leftAxis.endMove(aChainLength);
    rightAxis.endMove(bChainLength);
    if(zAxisAttached){
      zAxis.endMove(zPosition);
    }
    
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
    
    float stepSizeMM           = computeStepSize(MMPerMin);                    //step size in mm

    //the argument to abs should only be a variable -- splitting calc into 2 lines
    long finalNumberOfSteps    = abs(moveDist/stepSizeMM);      //number of steps taken in move
    finalNumberOfSteps = abs(finalNumberOfSteps);
    stepSizeMM = stepSizeMM*direction;
    
    long numberOfStepsTaken    = 0;
    
    //attach the axis we want to move
    axis->attach();
    
    float whereAxisShouldBeAtThisStep = startingPos;
    #if misloopDebug > 0
    inMovementLoop = true;
    #endif
    while(numberOfStepsTaken < finalNumberOfSteps){
        if (!movementUpdated) {
          //find the target point for this step
          whereAxisShouldBeAtThisStep += stepSizeMM;
          
          //write to axis
          axis->write(whereAxisShouldBeAtThisStep);
          movementUpdate();
          
          //update position on display
          returnPoz(xTarget, yTarget, zAxis.read());
          
          //increment the number of steps taken
          numberOfStepsTaken++;
        }
        
        //check for new serial commands
        readSerialCommands();
        
        //check for a STOP command
        if(checkForStopCommand()){
            return;
        }
    }
    #if misloopDebug > 0
    inMovementLoop = false;
    #endif
    
    axis->endMove(endPos);
    
}
    
int   findEndOfNumber(const String& textString, const int& index){
    //Return the index of the last digit of the number beginning at the index passed in
    unsigned int i = index;
    
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
    if(!zAxisAttached){
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

            maslowDelay(1000);
        }
    }
    
    
    if (G0orG1 == 1){
        //if this is a regular move
        coordinatedMove(xgoto, ygoto, zgoto, feedrate); //The XY move is performed
    }
    else{
        //if this is a rapid move
        coordinatedMove(xgoto, ygoto, zgoto, 1000); //move the same as a regular move, but go fast
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
    
    //attach the axes
    leftAxis.attach();
    rightAxis.attach();
    
    while(numberOfStepsTaken < abs(finalNumberOfSteps)){
        #if misloopDebug > 0
        inMovementLoop = true;
        #endif
        
        //if last movement was performed start the next one
        if (!movementUpdated){
            
            degreeComplete = float(numberOfStepsTaken)/float(finalNumberOfSteps);
            
            angleNow = startingAngle + theta*direction*degreeComplete;
            
            whereXShouldBeAtThisStep = radius * cos(angleNow) + centerX;
            whereYShouldBeAtThisStep = radius * sin(angleNow) + centerY;
            
            kinematics.inverse(whereXShouldBeAtThisStep,whereYShouldBeAtThisStep,&aChainLength,&bChainLength);
            
            leftAxis.write(aChainLength);
            rightAxis.write(bChainLength); 
            movementUpdate();
            
            returnPoz(whereXShouldBeAtThisStep, whereYShouldBeAtThisStep, zAxis.read());
            
            numberOfStepsTaken++;
            
            //check for new serial commands
            readSerialCommands();
            
            //check for a STOP command
            if(checkForStopCommand()){
                return 1;
            }
        }
    }
    #if misloopDebug > 0
    inMovementLoop = false;
    #endif
    
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
        return arc(X1, Y1, X2, Y2, centerX, centerY, feedrate, CLOCKWISE);
    }
    else {
        return arc(X1, Y1, X2, Y2, centerX, centerY, feedrate, COUNTERCLOCKWISE);
    }
}

void  G10(const String& readString){
    /*The G10() function handles the G10 gcode which re-zeros one or all of the machine's axes.*/
    float currentZPos = zAxis.read();
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
      feedrate = constrain(feedrate, 1, MAXZROTMIN * abs(zAxis.getPitch()));

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
      pinMode(ProbePin, INPUT_PULLUP);
      digitalWrite(ProbePin, HIGH);

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

        long numberOfStepsTaken    = 0;
        float whereAxisShouldBeAtThisStep = startingPos;
  
        axis->attach();
        //  zAxis->attach();

        while (numberOfStepsTaken < finalNumberOfSteps) {
          if (!movementUpdated){
              //find the target point for this step
              whereAxisShouldBeAtThisStep += stepSizeMM * direction;

              //write to each axis
              axis->write(whereAxisShouldBeAtThisStep);
              movementUpdate();

              //update position on display
              returnPoz(xTarget, yTarget, zAxis.read());

              //increment the number of steps taken
              numberOfStepsTaken++;
          }

          //check for new serial commands
          readSerialCommands();

          //check for a STOP command
          if (checkForStopCommand()) {
            return;
          }

          //check for Probe touchdown
          if (checkForProbeTouch(ProbePin)) {
            zAxis.set(0);
            zAxis.endMove(0);
            zAxis.attach();
            Serial.println(F("z axis zeroed"));
            return;
          }
        }

        /*
           If we get here, the probe failed to touch down
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

void  calibrateChainLengths(String gcodeLine){
    /*
    The calibrateChainLengths function lets the machine know that the chains are set to a given length where each chain is ORIGINCHAINLEN
    in length
    */
    
    if (extractGcodeValue(gcodeLine, 'L', 0)){
        //measure out the left chain
        Serial.println(F("Measuring out left chain"));
        singleAxisMove(&leftAxis, ORIGINCHAINLEN, 800);
        
        Serial.print(leftAxis.read());
        Serial.println(F("mm"));
        
        leftAxis.detach();
    }
    else if(extractGcodeValue(gcodeLine, 'R', 0)){
        //measure out the right chain
        Serial.println(F("Measuring out right chain"));
        singleAxisMove(&rightAxis, ORIGINCHAINLEN, 800);
        
        Serial.print(rightAxis.read());
        Serial.println(F("mm"));
        
        rightAxis.detach();
    }
    
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
    float encoderSteps         = extractGcodeValue(readString, 'J', -1);
    float gearTeeth          = extractGcodeValue(readString, 'K', -1);
    float chainPitch         = extractGcodeValue(readString, 'M', -1);
    
    float zDistPerRot        = extractGcodeValue(readString, 'N', -1);
    float zEncoderSteps        = extractGcodeValue(readString, 'P', -1);
    
    float propWeight         = extractGcodeValue(readString, 'R', -1);
    float KpPos              = extractGcodeValue(readString, 'S', -1);
    float KiPos              = extractGcodeValue(readString, 'T', -1);
    float KdPos              = extractGcodeValue(readString, 'U', -1);
    float KpV                = extractGcodeValue(readString, 'V', -1);
    float KiV                = extractGcodeValue(readString, 'W', -1);
    float KdV                = extractGcodeValue(readString, 'X', -1);
    if (extractGcodeValue(readString, 'Y', -1) != -1) {
	zAxisAuto            = extractGcodeValue(readString, 'Y', -1);
    }
      
    //Write the PID values to the axis if new ones have been received
    if (KpPos != -1){
        leftAxis.setPIDValues(KpPos, KiPos, KdPos, propWeight, KpV, KiV, KdV);
        rightAxis.setPIDValues(KpPos, KiPos, KdPos, propWeight, KpV, KiV, KdV);
        zAxis.setPIDValues(KpPos, KiPos, KdPos, propWeight, KpV, KiV, KdV);
    }
    
    //Change the motor properties in cnc_funtions if new values have been sent
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

// *** There is a more elegant way to do this - put the machineReady check at the beginning of each non-safe code!
//     This will reduce the overhead of looking through safeCommands[] using isSafeCommand()
//     and eliminate the 76 bytes of dynamic memory consumed by the safeCommands[] array in global variables!
bool isSafeCommand(const String& readString){
    bool ret = false;
    String command = readString.substring(0, 3);
    for(byte i = 0; i < sizeof(safeCommands); i++){
       if(safeCommands[i] == command){
           ret = true;
           break;
       }
    }
    return ret;
}

void  setSpindlePower(boolean powerState) {
    /*
     * Turn spindle on or off depending on powerState
     */ 
    boolean useServo = !zAxisAuto;
    boolean activeHigh = true;
    int delayAfterChange = 1000;  // milliseconds
    int servoIdle =  90;  // degrees
    int servoOn   = 180;  // degrees
    int servoOff  =   0;  // degrees
    int servoDelay = 2000;  // milliseconds
  
    // Now for the main code
    #if defined (verboseDebug) && verboseDebug > 1              
    Serial.print(F("Spindle control uses pin "));
    Serial.print(SpindlePowerControlPin);
    #endif
    if (useServo) {   // use a servo to control a standard wall switch
        #if defined (verboseDebug) && verboseDebug > 1              
        Serial.print(F(" with servo (idle="));
        Serial.print(servoIdle);
        Serial.print(F(", on="));
        Serial.print(servoOn);
        Serial.print(F(", off="));
        Serial.print(servoOff);
        Serial.println(F(")"));
        #endif
        myservo.attach(SpindlePowerControlPin); // start servo control
        myservo.write(servoIdle);   // move servo to idle position
        maslowDelay(servoDelay);    // wait for move to complete
        if (powerState) { // turn on spindle
            Serial.println(F("Turning Spindle On"));
            myservo.write(servoOn); // move servo to turn on switch
        }
        else {            // turn off spindle
            Serial.println(F("Turning Spindle Off"));
            myservo.write(servoOff); // move servo to turn off switch
        }
        maslowDelay(servoDelay);    // wait for move to complete
        myservo.write(servoIdle);   // return servo to idle position
        maslowDelay(servoDelay);    // wait for move to complete
        myservo.detach();           // stop servo control
    }
    else {            // use a digital I/O pin to control a relay
        #if defined (verboseDebug) && verboseDebug > 1              
        Serial.print(F(" as digital output, active "));
        if (activeHigh) Serial.println(F("high"));
        else Serial.println(F("low"));
        #endif
        pinMode(SpindlePowerControlPin, OUTPUT);
        if (powerState) { // turn on spindle
            Serial.println(F("Turning Spindle On"));
            if (activeHigh) digitalWrite(SpindlePowerControlPin, HIGH);
            else digitalWrite(SpindlePowerControlPin, LOW);
        }
        else {            // turn off spindle
            Serial.println(F("Turning Spindle Off"));
            if (activeHigh) digitalWrite(SpindlePowerControlPin, LOW);
            else digitalWrite(SpindlePowerControlPin, HIGH);
        }
    }
    maslowDelay(delayAfterChange);
}

void PIDTestVelocity(Axis* axis, const float start, const float stop, const float steps, const float version){
    // Moves the defined Axis at series of speed steps for PID tuning
    // Start Log
    Serial.println(F("--PID Velocity Test Start--"));
    Serial.println(axis->motorGearboxEncoder.getPIDString());
    if (version == 2) {
      Serial.println(F("setpoint,input,output"));
    }

    double startTime;
    double print = micros();
    double current = micros();
    float error;
    float reportedSpeed;
    float span = stop - start;
    float speed;
    
    // Start the steps
    axis->disablePositionPID();
    axis->attach();
    for(int i = 0; i < steps; i++){
        // 1 step = start, 2 step = start & finish, 3 = start, start + 1/2 span...
        speed = start;
        if (i > 0){
            speed = start + (span * (i/(steps-1)));
        }
        startTime = micros();
        axis->motorGearboxEncoder.write(speed);
        while (startTime + 2000000 > current){
          if (current - print > LOOPINTERVAL){
            if (version == 2) {
              Serial.println(axis->motorGearboxEncoder.pidState());
            }
            else {
              reportedSpeed= axis->motorGearboxEncoder.cachedSpeed();
              error =  reportedSpeed - speed;
              print = current;
              Serial.println(error);
            }
          }
          current = micros();
        }
    }
    axis->motorGearboxEncoder.write(0);
    
    // Print end of log, and update axis for use again
    Serial.println(F("--PID Velocity Test Stop--\n"));
    axis->write(axis->read());
    axis->detach();
    axis->enablePositionPID();
    kinematics.forward(leftAxis.read(), rightAxis.read(), &xTarget, &yTarget);
}

void positionPIDOutput (Axis* axis, float setpoint, float startingPoint){
  Serial.print((setpoint - startingPoint), 4);
  Serial.print(F(","));
  Serial.print((axis->pidInput() - startingPoint),4);
  Serial.print(F(","));  
  Serial.print(axis->pidOutput(),4);
  Serial.print(F(","));
  Serial.print(axis->motorGearboxEncoder.cachedSpeed(), 4);
  Serial.print(F(","));
  Serial.println(axis->motorGearboxEncoder.motor.lastSpeed());
}

void PIDTestPosition(Axis* axis, float start, float stop, const float steps, const float stepTime, const float version){
    // Moves the defined Axis at series of chain distance steps for PID tuning
    // Start Log
    Serial.println(F("--PID Position Test Start--"));
    Serial.println(axis->getPIDString());
    if (version == 2) {
      Serial.println(F("setpoint,input,output,rpminput,voltage"));
    }

    unsigned long startTime;
    unsigned long print = micros();
    unsigned long current = micros();
    float error;
    float startingPoint = axis->read();
    start = startingPoint + start;
    stop  = startingPoint + stop;
    float span = stop - start;
    float location;
    
    // Start the steps
    axis->attach();
    for(int i = 0; i < steps; i++){
        // 1 step = start, 2 step = start & finish, 3 = start, start + 1/2 span...
        location = start;
        if (i > 0){
            location = start + (span * (i/(steps-1)));
        }
        startTime = micros();
        current = micros();
        axis->write(location);
        while (startTime + (stepTime * 1000) > current){
          if (current - print > LOOPINTERVAL){
            if (version == 2) {
              positionPIDOutput(axis, location, startingPoint);
            }
            else {
              error   =  axis->read() - location;
              Serial.println(error);
            }
            print = current;
          }
          current = micros();
        }
    }
    startTime = micros();
    current = micros();
    //Allow 1 seccond to settle out
    while (startTime + 1000000 > current){
      if (current - print > LOOPINTERVAL){
        if (version == 2) {
          positionPIDOutput(axis, location, startingPoint);
        }            
        else {
          error   =  axis->read() - location;
          Serial.println(error);
        }
        print = current;
      }
      current = micros();
    }
    // Print end of log, and update axis for use again
    Serial.println(F("--PID Position Test Stop--\n"));
    axis->write(axis->read());
    axis->detach();
    kinematics.forward(leftAxis.read(), rightAxis.read(), &xTarget, &yTarget);
}

void voltageTest(Axis* axis, int start, int stop){
    // Moves the defined Axis at a series of voltages and reports the resulting
    // RPM
    Serial.println(F("--Voltage Test Start--"));
    int direction = 1;
    if (stop < start){ direction = -1;}
    int steps = abs(start - stop);
    unsigned long startTime = millis() + 200;
    unsigned long currentTime = millis();
    unsigned long printTime = 0;
    
    for (int i = 0; i <= steps; i++){
        axis->motorGearboxEncoder.motor.directWrite((start + (i*direction)));
        while (startTime > currentTime - (i * 200)){
            currentTime = millis();
            if ((printTime + 50) <= currentTime){
                Serial.print((start + (i*direction)));
                Serial.print(F(","));
                Serial.print(axis->motorGearboxEncoder.computeSpeed(),4);
                Serial.print(F("\n"));
                printTime = millis();
            }
        }
    }
    
    // Print end of log, and update axis for use again
    axis->motorGearboxEncoder.motor.directWrite(0);
    Serial.println(F("--Voltage Test Stop--\n"));
    axis->write(axis->read());
    kinematics.forward(leftAxis.read(), rightAxis.read(), &xTarget, &yTarget);
}

void  executeBcodeLine(const String& gcodeLine){
    /*
    
    Executes a single line of gcode beginning with the character 'B'.
    
    */
    
    // *** There is a more elegant way to do this - put the machineReady check at the beginning of each non-safe code!
    //     This will reduce the overhead of looking through safe commands using isSafeCommand()
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
        calibrateChainLengths(gcodeLine);
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
        float speed = extractGcodeValue(gcodeLine, 'F', 800);
        
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
    
    if(gcodeLine.substring(0, 3) == "B13"){
        //PID Testing of Velocity
        float  left       = extractGcodeValue(gcodeLine, 'L', 0);
        float  useZ       = extractGcodeValue(gcodeLine, 'Z', 0);
        float  start      = extractGcodeValue(gcodeLine, 'S', 1);
        float  stop       = extractGcodeValue(gcodeLine, 'F', 1);
        float  steps      = extractGcodeValue(gcodeLine, 'I', 1);
        float  version    = extractGcodeValue(gcodeLine, 'V', 1);
        
        Axis* axis = &rightAxis;
        if (left > 0) axis = &leftAxis;
        if (useZ > 0) axis = &zAxis;
        PIDTestVelocity(axis, start, stop, steps, version);
        return;
    }
    
    if(gcodeLine.substring(0, 3) == "B14"){
        //PID Testing of Position
        float  left       = extractGcodeValue(gcodeLine, 'L', 0);
        float  useZ       = extractGcodeValue(gcodeLine, 'Z', 0);
        float  start      = extractGcodeValue(gcodeLine, 'S', 1);
        float  stop       = extractGcodeValue(gcodeLine, 'F', 1);
        float  steps      = extractGcodeValue(gcodeLine, 'I', 1);
        float  stepTime   = extractGcodeValue(gcodeLine, 'T', 2000);
        float  version    = extractGcodeValue(gcodeLine, 'V', 1);
        
        Axis* axis = &rightAxis;
        if (left > 0) axis = &leftAxis;
        if (useZ > 0) axis = &zAxis;
        PIDTestPosition(axis, start, stop, steps, stepTime, version);
        return;
    }
    
    if(gcodeLine.substring(0, 3) == "B16"){
        //Incrementally tests voltages to see what RPMs they produce
        float  left       = extractGcodeValue(gcodeLine, 'L', 0);
        float  useZ       = extractGcodeValue(gcodeLine, 'Z', 0);
        float  start      = extractGcodeValue(gcodeLine, 'S', 1);
        float  stop       = extractGcodeValue(gcodeLine, 'F', 1);
        
        Axis* axis = &rightAxis;
        if (left > 0) axis = &leftAxis;
        if (useZ > 0) axis = &zAxis;
        voltageTest(axis, start, stop);
        return;
    }
    
    if(gcodeLine.substring(0, 3) == "B15"){
        //The B15 command moves the chains to the length which will put the sled in the center of the sheet
        
        //Compute chain length for position 0,0
        float chainLengthAtMiddle;
        kinematics.inverse(0,0,&chainLengthAtMiddle,&chainLengthAtMiddle);
        
        //Adjust left chain length
        singleAxisMove(&leftAxis,  chainLengthAtMiddle, 800);
        
        //Adjust right chain length
        singleAxisMove(&rightAxis, chainLengthAtMiddle, 800);
        
        //Reload the position
        kinematics.forward(leftAxis.read(), rightAxis.read(), &xTarget, &yTarget);
    }
}
    
void  executeGcodeLine(const String& gcodeLine){
    /*
    
    Executes a single line of gcode beginning with the character 'G'.  If the G code is
    not included on the front of the line, the code from the previous line will be added.
    
    */

    // *** There is a more elegant way to do this - put the machineReady check at the beginning of each non-safe code!
    //     This will reduce the overhead of looking through safe commands using isSafeCommand()
    if (!machineReady() && !isSafeCommand(gcodeLine)){
        Serial.println(F("Unable to execute command, machine settings not yet received"));
        return;
    }
      
    //Handle G-Codes
   
    int gNumber = extractGcodeValue(gcodeLine,'G', -1);
    
    if (gNumber == -1){               // If the line does not have a G command
        gNumber = lastCommand;        // apply the last one
    }
    
    switch(gNumber){
        case 0:   // Rapid positioning
        case 1:   // Linear interpolation
            G1(gcodeLine, gNumber);
            lastCommand = gNumber;    // remember G number for next time
            break;
        case 2:   // Circular interpolation, clockwise
        case 3:   // Circular interpolation, counterclockwise
            G2(gcodeLine, gNumber);
            lastCommand = gNumber;    // remember G number for next time
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
            Serial.print(F("Command G"));
            Serial.print(gNumber);
            Serial.println(F(" unsupported and ignored."));
    }

}
    
void  executeMcodeLine(const String& gcodeLine){
    /*
    
    Executes a single line of gcode beginning with the character 'M'.
    
    */
    
    //Handle M-Codes
   
    int mNumber = extractGcodeValue(gcodeLine,'M', -1);
    
    switch(mNumber){
        case 0:   // Program Pause / Unconditional Halt / Stop
        case 1:   // Optional Pause / Halt / Sleep
            pause();
            break;
        case 2:   // Program End
        case 30:  // Program End with return to program top
        case 5:   // Spindle Off
            setSpindlePower(false); // turn off spindle
            break;
        case 3:   // Spindle On - clockwise
        case 4:   // Spindle On - counterclockwise
            // Maslow spindle runs only one direction, but turn spindle on for either code
            setSpindlePower(true);  // turn on spindle
            break;
        case 6:   // Tool Change
            if (nextTool > 0) {
                setSpindlePower(false); // first, turn off spindle
                Serial.print(F("Tool Change: Please insert tool "));   // prompt user to change tool
                Serial.println(nextTool);
                lastTool = nextTool;
                pause();
            }
            break;
        default:
            Serial.print(F("Command M"));
            Serial.print(mNumber);
            Serial.println(F(" unsupported and ignored."));
    }
    
}

void  executeOtherCodeLine(const String& gcodeLine){
    /*
    
    Executes a single line of gcode beginning with a character other than 'G', 'B', or 'M'.
    
    */

    if (gcodeLine.length() > 1) {
        if (gcodeLine[0] == 'T') {
            int tNumber = extractGcodeValue(gcodeLine,'T', 0);    // get tool number
            Serial.print(F("Tool change to tool "));
            Serial.println(tNumber);
            if ((tNumber > 0) && (tNumber != lastTool)) {         // if tool number is greater than 0 and not the same as the last tool
                nextTool = tNumber;                               // remember tool number to prompt user when G06 is received
            }
            else {
                nextTool = 0;                                     // tool is 0 or same as last change - don't prompt user on next G06
            }
        }
        else {  // try it as a 'G' command without the leading 'G' code
            executeGcodeLine(gcodeLine);
        }
    }
    else {
        Serial.print(F("Command "));
        Serial.print(gcodeLine);
        Serial.println(F(" too short - ignored."));      
    }
        
} 

int   findNextGM(const String& readString, const int& startingPoint){
    int nextGIndex = readString.indexOf('G', startingPoint);
    int nextMIndex = readString.indexOf('M', startingPoint);
    if (nextMIndex != -1) {           // if 'M' was found
        if ((nextGIndex == -1) || (nextMIndex < nextGIndex)) { // and 'G' was not found, or if 'M' is before 'G'
            nextGIndex = nextMIndex;  // then use 'M'
        }
    }
    if (nextGIndex == -1) {           // if 'G' was not found (and therefore 'M' was not found)
        nextGIndex = readString.length();   // then use the whole string
    }
    
    return nextGIndex;
}

void  interpretCommandString(String& cmdString){
    /*
    
    Splits a string into lines of gcode which begin with 'G' or 'M', executing each in order
    Also executes full lines for 'B' codes, and handles 'T' at beginning of line

    Assumptions:
        Leading and trailing white space has already been removed from cmdString
        cmdString has been converted to upper case
    
    */
    
    returnError();  //Cue up sending the next line
    
    int firstG;  
    int secondG;

    if (cmdString.length() > 0) {
        if (cmdString[0] == 'B'){                   //If the command is a B command
            #if defined (verboseDebug) && verboseDebug > 0
            Serial.print(F("iCS executing B code line: "));
            #endif
            Serial.println(cmdString);
            executeBcodeLine(cmdString);
        }
        else{
            while(cmdString.length() > 0){          //Extract each line of gcode from the string
                firstG  = findNextGM(cmdString, 0);
                secondG = findNextGM(cmdString, firstG + 1);
                
                if(firstG == cmdString.length()){   //If the line contains no G or M letters
                    firstG = 0;                     //send the whole line
                }
    
                if (firstG > 0) {                   //If there is something before the first 'G' or 'M'
                    gcodeLine = cmdString.substring(0, firstG);
                    #if defined (verboseDebug) && verboseDebug > 0
                    Serial.print(F("iCS executing other code: "));
                    #endif
                    Serial.println(gcodeLine);
                    executeOtherCodeLine(gcodeLine);  // execute it first
                }
                
                gcodeLine = cmdString.substring(firstG, secondG);
                
                if (gcodeLine.length() > 0){
                    if (gcodeLine[0] == 'M') {
                        #if defined (verboseDebug) && verboseDebug > 0
                        Serial.print(F("iCS executing M code: "));
                        #endif
                        Serial.println(gcodeLine);
                        executeMcodeLine(gcodeLine);
                    }
                    else {
                        #if defined (verboseDebug) && verboseDebug > 0
                        Serial.print(F("iCS executing G code: "));
                        #endif
                        Serial.println(gcodeLine);
                        executeGcodeLine(gcodeLine);
                    }
                }
                
                cmdString = cmdString.substring(secondG, cmdString.length());
                
            }
        }
    }
    
    #if defined (verboseDebug) && verboseDebug > 1              
    // print ring buffer contents
    Serial.println(F("iCS execution complete"));
    ringBuffer.print();
    #endif
    
    _signalReady();
    
}

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
    
    
#include "Maslow.h"

Servo myservo;  // create servo object to control a servo 

bool zAxisAttached = false;
bool zAxisAuto = false;

#define FORWARD           1
#define BACKWARD         -1

#define CLOCKWISE        -1
#define COUNTERCLOCKWISE  1

#define MILLIMETERS 1
#define INCHES      25.4
#define MAXFEED     1000      //The maximum allowable feedrate in mm/min
#define MAXZROTMIN  12.60    // the maximum z rotations per minute

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

float feedrate              =  500;
float _inchesToMMConversion =  1;
bool  useRelativeUnits      =  false;
bool  encoderStepsChanged   =  false;
bool  zEncoderStepsChanged  =  false;
volatile bool  movementUpdated  =  false;

// Global variables for misloop tracking
#if misloopDebug > 0
volatile bool  inMovementLoop   =  false;
volatile bool  movementFail     =  false;
#endif

String gcodeLine;                         //The next individual line of gcode (for example G91 G01 X19 would be run as two lines)

int   lastCommand           =  0;         //Stores the value of the last command run eg: G01 -> 1
int   lastTool              =  0;         //Stores the value of the last tool number eg: T4 -> 4
int   nextTool              =  0;         //Stores the value of the next tool number eg: T4 -> 4

bool checkForStopCommand();

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
        Serial.print(incSerialBuffer.spaceAvailable());
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
            
            if (incSerialBuffer.length() == 0) {       // if the buffer is empty
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

bool checkForStopCommand(){
    /*
    Check to see if the STOP command has been sent to the machine.
    If it has, empty the buffer, stop all axes, set target position to current 
    position and return true.
    */
    if(sys.stop){
        readyCommandString = "";
        incSerialBuffer.empty();
        leftAxis.stop();
        rightAxis.stop();
        if(zAxisAttached){
          zAxis.stop();
        }
        kinematics.forward(leftAxis.read(), rightAxis.read(), &sys.xPosition, &sys.yPosition);
        sys.stop = false;
        return true;
    }
    return false;
}

void  holdPosition(){
    /*
    
    This function is called every time the main loop runs. When the machine is executing a move it is not called, but when the machine is
    not executing a line it is called regularly and causes the motors to hold their positions.
    
    */
    checkForStopCommand();
    
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
    
    sys.pause = true;
    Serial.println(F("Maslow Paused"));
    
    while(1){
        
        holdPosition();
    
        readSerialCommands();
    
        returnPoz(sys.xPosition, sys.yPosition, zAxis.read());
        
        if (!sys.pause){
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
        returnPoz(sys.xPosition, sys.yPosition, zAxis.read());
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
    
    float  xStartingLocation = sys.xPosition;
    float  yStartingLocation = sys.yPosition;
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
    
    sys.xPosition = xEnd;
    sys.yPosition = yEnd;
    
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
          returnPoz(sys.xPosition, sys.yPosition, zAxis.read());
          
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
    
int   G1(const String& readString, int G0orG1){
    
    /*G1() is the function which is called to process the string if it begins with 
    'G01' or 'G00'*/
    
    float xgoto;
    float ygoto;
    float zgoto;
        
    float currentXPos = sys.xPosition;
    float currentYPos = sys.yPosition;
    
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
    
    sys.xPosition = X2;
    sys.yPosition = Y2;
    
    return 1;
}

int   G2(const String& readString, int G2orG3){
    /*
    
    The G2 function handles the processing of the gcode line for both the command G2 and the
    command G3 which cut arcs.
    
    */
    
    
    float X1 = sys.xPosition; //does this work if units are inches? (It seems to)
    float Y1 = sys.yPosition;
    
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
              returnPoz(sys.xPosition, sys.yPosition, zAxis.read());

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
        sys.stop = true;
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

void  setInchesToMillimetersConversion(float newConversionFactor){
    _inchesToMMConversion = newConversionFactor;
}

void  printBeforeAndAfter(const float& before, const float& after){
    Serial.print(F("Before: "));
    Serial.print(before);
    Serial.print(F(" After: "));
    Serial.println(after);
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
    
    sys.rcvdMotorSettings = 1;
    finalizeMachineSettings(); 
    Serial.println(F("Motor Settings Loaded"));
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

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

// This file contains all the functions used to receive and parse the gcode
// commands

#include <Arduino.h>
#include "system.h"
#include "RingBuffer.h"

RingBuffer ringBuffer;
int expectedMaxLineLength   = 60;   // expected maximum Gcode line length in characters, including line ending character(s)
String readyCommandString = "";  //KRK why is this a global?

void readSerialCommands(){
    /*
    Check to see if a new character is available from the serial connection, 
    if this is a necessary character write to the ringBuffer otherwise discard
    it.
    */
    if (Serial.available() > 0) {
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '!'){
                sys.stop = true;
                sys.pause = false;
            }
            else if (c == '~'){
                sys.pause = false;
            }
            else{
                int bufferOverflow = ringBuffer.write(c); //gets one byte from serial buffer, writes it to the internal ring buffer
                if (bufferOverflow != 0) {
                  sys.stop = true;
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

int gcodeSpaceAvailable(){
    return ringBuffer.spaceAvailable();
}

bool gcodeIsBufferEmpty(){
    if (ringBuffer.length() == 0){
        return true;
    }
    else {
        return false;
        
        
        
    }
}

void gcodeClearBuffer(){
    ringBuffer.empty();
}

void gcodePrintBuffer(){
    ringBuffer.print();
}

String gcodeBufferReadline(){
    String gcodeline = ringBuffer.readLine();
    gcodeline.trim();  // remove leading and trailing white space
    gcodeline.toUpperCase();
    return gcodeline;
}

void  _signalReady(){
    /*
    
    Signal to the controlling software that the machine has executed the last
    gcode line successfully.
    
    */
    
    if ( (gcodeSpaceAvailable() > expectedMaxLineLength)    // if there is space in the buffer to accept the expected maximum line length
          && (ringBuffer.numberOfLines() < 4) ) {                 // and if there are fewer than 4 lines in the buffer
        Serial.println(F("ok"));                                  // then request new code
    }
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
    gcodePrintBuffer();
    #endif
    
    _signalReady();
    
}

void gcodeExecuteLoop(){
  readyCommandString = gcodeBufferReadline();
  
  if (readyCommandString.length() > 0){
      interpretCommandString(readyCommandString);
      readyCommandString = "";
  }
}


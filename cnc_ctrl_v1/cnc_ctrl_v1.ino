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
    
#include "Maslow.h"

// Define system global state structure
system_t sys;

// Define the global settings storage - treat as readonly
settings_t sysSettings;

// Global realtime executor bitflag variable for setting various alarms.
byte systemRtExecAlarm;  

// Define axes, it might be tighter to define these within the sys struct
Axis leftAxis;
Axis rightAxis;
Axis zAxis;

// Define kinematics, is it necessary for this to be a class?  Is this really
// going to be reused?
Kinematics kinematics;

void setup(){
    Serial.begin(57600);
    Serial.print(F("PCB v1."));
    Serial.print(getPCBVersion());
    Serial.println(F(" Detected"));
    sys.inchesToMMConversion = 1;
    setupAxes();
    settingsInit();
    // TODO This seems wrong, if the encoder steps are changed, axis position
    // will be in the wrong place.  Would be better if we stored positions as
    // steps 
    // Set initial desired position of the machine to its current position
    leftAxis.write(leftAxis.read());
    rightAxis.write(leftAxis.read());
    zAxis.write(leftAxis.read());
    readyCommandString.reserve(INCBUFFERLENGTH);           //Allocate memory so that this string doesn't fragment the heap as it grows and shrinks
    gcodeLine.reserve(INCBUFFERLENGTH);
    Timer1.initialize(LOOPINTERVAL);
    Timer1.attachInterrupt(runsOnATimer);
    
    Serial.println(F("Grbl v1.00"));  // Why GRBL?  Apparenlty because some programs are silly and look for this as an initailization command
    Serial.println(F("ready"));
}

void runsOnATimer(){
    #if misloopDebug > 0
    if (inMovementLoop && !movementUpdated){
        movementFail = true;
    }
    #endif
    movementUpdated = false;
    leftAxis.computePID();
    rightAxis.computePID();
    zAxis.computePID();
}

void loop(){
    // This section is called on startup and whenever a stop command is issued
    initGCode();
    if (sys.stop){               // only called on sys.stop to prevent stopping
        initMotion();            // on USB disconnect.  Might consider removing 
        setSpindlePower(false);  // this restriction for safety if we are 
    }                            // comfortable that USB disconnects are
                                 // not a common occurence anymore
    kinematics.init();
    
    // Let's go!
    reportStatusMessage(STATUS_OK);
    sys.stop = false;            // We should consider an abort option which
                                 // is not reset automatically such as a software
                                 // limit
    while (!sys.stop){
        gcodeExecuteLoop();
        
        execSystemRealtime();
    
        _watchDog();
    }
}

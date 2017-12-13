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

// Define axes, it might be tighter to define these within the sys struct
Axis leftAxis;
Axis rightAxis;
Axis zAxis;

// Define kinematics, is it necessary for this to be a class?  Is this really
// going to be reused?
Kinematics kinematics;

void setup(){
    sys.inchesToMMConversion = 1;
    setupAxes();
    Serial.begin(57600);
    readyCommandString.reserve(128);           //Allocate memory so that this string doesn't fragment the heap as it grows and shrinks
    gcodeLine.reserve(128);
    
    Serial.print(F("PCB v1."));
    Serial.print(getPCBVersion());
    Serial.println(F(" Detected"));
    
    Serial.println(F("ready"));
    _signalReady();
    
    Timer1.initialize(LOOPINTERVAL);
    Timer1.attachInterrupt(runsOnATimer);
    
    Serial.println(F("Grbl v1.00"));
    
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
    
    gcodeExecuteLoop();
    
    holdPosition();
    
    readSerialCommands();
    
    returnPoz(sys.xPosition, sys.yPosition, zAxis.read());
    
    _watchDog();
}

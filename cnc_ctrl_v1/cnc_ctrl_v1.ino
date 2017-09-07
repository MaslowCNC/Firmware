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
    
#include "CNC_Functions.h"
#include "TimerOne.h"

void setup(){
    Serial.begin(57600);
    
    readyCommandString.reserve(128);           //Allocate memory so that this string doesn't fragment the heap as it grows and shrinks
    gcodeLine.reserve(128);
    
    if(pcbRevisionIndicator == 0){
        Serial.println(F("PCB v1.1 Detected"));
    } 
    if(pcbRevisionIndicator == 1){
        Serial.println(F("Beta PCB v1.0 Detected"));
    }
    
    Serial.println(F("ready"));
    _signalReady();
    
    
    Timer1.initialize(10000);
    Timer1.attachInterrupt(runsOnATimer);
    
    Serial.println(F("Grbl v1.00"));
    
}

void runsOnATimer(){
    leftAxis.computePID();
    rightAxis.computePID();
    zAxis.computePID();
}

void loop(){
    
    readyCommandString = ringBuffer.readLine();
    
    if (readyCommandString.length() > 0){
        readyCommandString.trim();  // remove leading and trailing white space
        readyCommandString.toUpperCase();
        interpretCommandString(readyCommandString);
        readyCommandString = "";
    }
    
    holdPosition();
    
    readSerialCommands();
    
    returnPoz(xTarget, yTarget, zAxis.read());
    
    _watchDog();
}

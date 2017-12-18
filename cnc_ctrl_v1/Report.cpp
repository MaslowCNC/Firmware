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

// This file contains the functions for outgoing Serial responses

#include "Maslow.h"

void  _signalReady(){
    /*
    
    Signal to the controlling software that the machine has executed the last
    gcode line successfully.
    
    */
    // if there is space in the buffer to accept the expected maximum line length
    // and if there are fewer than max allowed lines in the buffer, request new code
    if ( (incSerialBuffer.spaceAvailable() > EXPGCODELINE)    
          && (incSerialBuffer.numberOfLines() < MAXBUFFERLINES) ) {  
        Serial.println(F("ok"));
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
        Serial.print(incSerialBuffer.spaceAvailable());
        Serial.println(F("]"));
}

void  returnPoz(){
    /*
    Causes the machine's position (x,y) to be sent over the serial connection updated on the UI
    in Ground Control. Also causes the error report to be sent. Only executes 
    if hasn't been called in at least POSITIONTIMEOUT ms.
    */
    
    static unsigned long lastRan = millis();
    
    if (millis() - lastRan > POSITIONTIMEOUT){
        
        
        Serial.print(F("<Idle,MPos:"));
        Serial.print(sys.xPosition/sys.inchesToMMConversion);
        Serial.print(F(","));
        Serial.print(sys.yPosition/sys.inchesToMMConversion);
        Serial.print(F(","));
        Serial.print(zAxis.read()/sys.inchesToMMConversion);
        Serial.println(F(",WPos:0.000,0.000,0.000>"));
        
        returnError();
        
        lastRan = millis();
    }
    
}

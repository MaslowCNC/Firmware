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
    ringBuffer.readLine();
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
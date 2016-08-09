    /*This file is part of the Makesmith Control Software.

    The Makesmith Control Software is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Makesmith Control Software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the Makesmith Control Software.  If not, see <http://www.gnu.org/licenses/>.
    
    Copyright 2014 Bar Smith*/
    
    

#include "MyTypes.h"
#include "CNC_Functions.h"


String readString;

void setup(){
    Serial.begin(19200);
    
    Serial.println("ready");
    Serial.println("gready");
    
    xAxis.initializePID();
    yAxis.initializePID();
    
}

void loop(){
    readString = "";
    if (Serial.available()){
        while (true) {
            delay(1);  //delay to allow buffer to fill 
            if (Serial.available() > 0) {
                char c = Serial.read();  //gets one byte from serial buffer
                if (c == '#'){
                    Serial.println("newline");
                    break;
                }
                readString += c; //makes the string readString
            } 
        }
    }
    if (readString.length() > 0){
        interpretCommandString(readString);
    }
    
    holdPosition();
    
    returnPoz();
}
    
    
    

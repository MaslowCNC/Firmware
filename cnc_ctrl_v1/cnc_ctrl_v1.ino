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
    Serial.begin(19200);
    
    kinematics.forward(leftAxis.read(), rightAxis.read(), &xTarget, &yTarget);
    
    Serial.println("ready");
    Serial.println("ok");
    
    Timer1.initialize(10000);
    Timer1.attachInterrupt(runsOnATimer);
    
    Serial.println("Grbl v1.00");
    
    
    
    int i = 0;
    String testGcode = "G20 \nG1 X-1 Y-1 F25 \nG1 X1 \nG3 Y2 J1.5\nG1 X-1 \nG1 Y-1\nG1 X0 Y0G20 \nG1 X-1 Y-1 F25 \nG1 X1 \nG3 Y2 J1.5\nG1 X-1 \nG1 Y-1\nG1 X0 Y0";
    
    Serial.println(testGcode.length());
    while (i < testGcode.length()){
        ringBuffer.write(testGcode[i]);
        i++;
    }
    
}

void runsOnATimer(){
    leftAxis.computePID();
    rightAxis.computePID();
    zAxis.computePID();
}

void loop(){
    
    readyCommandString = ringBuffer.readLine();
    
    if (readyCommandString.length() > 0){
        readyCommandString.toUpperCase();
        interpretCommandString(readyCommandString);
        readyCommandString = "";
    }
    
    holdPosition();
    
    readSerialCommands();
    
    returnPoz(xTarget, yTarget, zAxis.read());
    
    _watchDog();
}

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

/*
The Motor module imitates the behavior of the Arduino servo module. It allows a gear motor (or any electric motor)
to be a drop in replacement for a continuous rotation servo.

*/

#include "Arduino.h"
#include "MotorGearboxEncoder.h"

MotorGearboxEncoder::MotorGearboxEncoder(int pwmPin, int directionPin1, int directionPin2, int encoderPin1, int encoderPin2)
:
encoder(encoderPin1,encoderPin2)
{
    
    //initialize motor
    motor.setupMotor(pwmPin, directionPin1, directionPin2);
    
}

float MotorGearboxEncoder::computeSpeed(){
    /*Serial.println(micros() - _lastTimeStamp);
    
    _lastTimeStamp = micros();*/
    
    return 0;
}

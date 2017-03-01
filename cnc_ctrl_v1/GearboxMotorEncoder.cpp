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


#include "Arduino.h"
#include "GearboxMotorEncoder.h"

GearboxMotorEncoder::GearboxMotorEncoder(int pwmPin, int directionPin1, int directionPin2, int encoderPin1, int encoderPin2, int eepromAdr)
:
_encoder(encoderPin1,encoderPin2)
{
    
    Serial.println("begin gearbox motor setup");
    
    //initialize motor
    _motor.setupMotor(pwmPin, directionPin1, directionPin2);
    
    
    //initialize variables
    _eepromAdr    = eepromAdr;
    
    //_readAllLinSegs(_eepromAdr);
}
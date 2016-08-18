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
    
    Copyright 2014-2016 Bar Smith*/


#include "Arduino.h"
#include "Axis.h"

#define FORWARD 1
#define BACKWARD -1

#define EEPROMVALIDDATA 56
#define EEPROMFLAG 18

#define NUMBER_OF_ENCODER_STEPS 4096.0



Axis::Axis(int pwmPin, int directionPin1, int directionPin2, int encoderDirection, int encoderPin1, int encoderPin2, String axisName, int eepromAdr, float mmPerRotation)
:
_encoder(encoderPin1,encoderPin2)
{
    
    //initialize motor
    _motor.setupMotor(pwmPin, directionPin1, directionPin2);
    
    _pidController.setup(&_pidInput, &_pidOutput, &_pidSetpoint, _Kp, _Ki, _Kd, REVERSE);
    
    //initialize variables
    _direction    = encoderDirection;
    _axisName     = axisName;
    _axisPosition = 0.0;
    _axisTarget   = 0.0;
    _direction    = BACKWARD;
    _eepromAdr    = eepromAdr;
    _mmPerRotation= mmPerRotation;
    
    //load position
    if (EEPROM.read(EEPROMFLAG) == EEPROMVALIDDATA){
        set(_readFloat(_eepromAdr));
    }
    
}

void   Axis::initializePID(){
    _pidController.SetMode(AUTOMATIC);
    _pidController.SetOutputLimits(-90, 90);
}

int    Axis::write(float targetPosition){
    
    _pidInput      =  _axisPosition;
    _pidSetpoint   =  targetPosition/_mmPerRotation;
    
    _pidController.Compute();
    
    _motor.write(90 + _pidOutput);
    
    while(true){
        Serial.println("help! I'm stuck");
        _motor.write(90 + 90);
    }
    
    return 1;
}

float  Axis::read(){
    
    if (_motor.attached()){
        return _axisPosition*_mmPerRotation;
    }
    else{
        return _axisPosition*_mmPerRotation;
    }
}

float  Axis::target(){
    return _axisTarget*_mmPerRotation;
}

float Axis::setpoint(){
    return _pidSetpoint*_mmPerRotation;
}

int    Axis::set(float newAxisPosition){
    _axisPosition =  newAxisPosition/_mmPerRotation;
    _axisTarget   =  newAxisPosition/_mmPerRotation;
    _encoder.write((_direction*newAxisPosition*NUMBER_OF_ENCODER_STEPS)/_mmPerRotation);
}

int    Axis::updatePositionFromEncoder(){
    
    _axisPosition = _direction * _encoder.read()/NUMBER_OF_ENCODER_STEPS;

}

float  Axis::error(){
    return abs(_axisPosition - _pidSetpoint)*_mmPerRotation;
}

int    Axis::detach(){
    
    if (_motor.attached()){
        _writeFloat(_eepromAdr, read());
        EEPROM.write(EEPROMFLAG, EEPROMVALIDDATA);
    }
    
    _motor.detach();
    
    return 1;
}

int    Axis::attach(){
     _motor.attach(1);
     return 1;
}

void   Axis::hold(){
    int timeout   = 2000;
    
    if (millis() - _timeLastMoved < timeout){
        updatePositionFromEncoder();
        write(_axisTarget*_mmPerRotation);
    }
    else{
        detach();
    }
    
}

void   Axis::endMove(float finalTarget){
    
    _timeLastMoved = millis();
    _axisTarget    = finalTarget/_mmPerRotation;
    
}

int    Axis::returnPidMode(){
    return _pidController.GetMode();
}

float  Axis::_readFloat(unsigned int addr){

//readFloat and writeFloat functions courtesy of http://www.alexenglish.info/2014/05/saving-floats-longs-ints-eeprom-arduino-using-unions/


    union{
        byte b[4];
        float f;
    } data;
    for(int i = 0; i < 4; i++)
    {
        data.b[i] = EEPROM.read(addr+i);
    }
    return data.f;
}

void   Axis::_writeFloat(unsigned int addr, float x){
    union{
        byte b[4];
        float f;
    } data;
    data.f = x;
    for(int i = 0; i < 4; i++){
        EEPROM.write(addr+i, data.b[i]);
    }
}

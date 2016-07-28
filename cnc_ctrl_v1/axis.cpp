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



Axis::Axis(int pwmPin, int directionPin1, int directionPin2, int encoderDirection, int encoderPin, String axisName, int eepromAdr, float mmPerRotation){
    
    //initialize motor
    _motor.setupMotor(pwmPin, directionPin1, directionPin2);
    
    //initialize pins
    pinMode(encoderPin, INPUT);
    
    _pidController.setup(&_pidInput, &_pidOutput, &_pidSetpoint, _Kp, _Ki, _Kd, REVERSE);
    
    //initialize variables
    _direction    = encoderDirection;
    _encoderPin   = encoderPin;
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
    
    return 1;
}

float  Axis::read(){
    if (_motor.attached()){
        return _axisPosition*_mmPerRotation;
    }
    else{
        return _axisTarget*_mmPerRotation;
    }
}

float Axis::target(){
    return _axisTarget*_mmPerRotation;
}

int    Axis::set(float newAxisPosition){
    _axisPosition =  newAxisPosition/_mmPerRotation;
    _axisTarget   =  newAxisPosition/_mmPerRotation;
}

int    Axis::updatePositionFromEncoder(){

/*The SetPos() function updates the machine's position by essentially integrating 
the input from the encoder*/

    int maxJump = 400;

    if(abs(_currentAngle - _previousAngle) <= maxJump){ //The encoder did not just transition from 0 to 360 degrees
        _axisPosition = _axisPosition + (_currentAngle - _previousAngle)/1023.0; //The position is incremented by the change in position since the last update.
    }
    else{//The transition from 0 to 360 (10-bit value 1023) or vice versa has just taken place
        if(_previousAngle < 200 && _currentAngle > 850){ //Add back in any dropped position
            _currentAngle = 1023;
            _axisPosition = _axisPosition + (0 - _previousAngle)/1023.0;
        }
        if(_previousAngle > 850 && _currentAngle < 200){
            _currentAngle = 0;
            _axisPosition = _axisPosition + (1023 - _previousAngle)/1023.0;
        }
    }
    

    _previousAngle = _currentAngle; //Reset the previous angle variables

    if(_direction == FORWARD){ //Update the current angle variable. Direction is set at compile time depending on which side of the rod the encoder is positioned on.
        _currentAngle = _PWMread(_encoderPin);
    }
    else{
        _currentAngle = 1023 - _PWMread(_encoderPin);
    }
    
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
    
    if (millis() - _timeLastMoved < 2000){
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
    Serial.println("ft set to");
    Serial.println(_axisTarget);
    Serial.println(finalTarget);
    
}

int    Axis::_PWMread(int pin){

/*PWMread() measures the duty cycle of a PWM signal on the provided pin. It then
takes this duration and converts it to a ten bit number.*/

    int duration = 0;
    int numberOfSamplesToAverage = 1;
    int i = 0;
    
    while (i < numberOfSamplesToAverage){
        duration = duration + pulseIn(pin, HIGH, 2000); //This returns the pulse duration
        i++;
    }
    
    duration = duration/numberOfSamplesToAverage;
    
    duration = (int)((float)duration*1.23); //1.23 scales it to a ten bit number
    
    
    if (duration >= 1023){
        duration = 1023;
    }
    

    
    if (duration < 10){
        duration = 0;
    }

    return duration;

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

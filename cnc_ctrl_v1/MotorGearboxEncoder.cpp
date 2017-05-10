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

void  MotorGearboxEncoder::write(float speed){
    /*
    Command the motor to turn at the given speed. Should be RPM is PWM right now.
    */
    
    _targetSpeed = 15.0*speed; //15 is 255 (max pwm) / 17 (max RPM)
    
}

void  MotorGearboxEncoder::computePID(){
    /*
    Recompute the speed control PID loop and command the motor to move.
    */
    computeSpeed();
    
    motor.write(_targetSpeed);
}

float MotorGearboxEncoder::computeSpeed(){
    /*
    
    Returns the motors speed in RPM since the last time this function was called
    
    */
    double timeElapsed =  micros() - _lastTimeStamp;
    float    distMoved   =  _runningAverage(encoder.read() - _lastPosition);     //because of quantization noise it helps to average these
    
    //Compute the speed in RPM
    float RPM = (7364.0*distMoved)/float(timeElapsed);  //6*10^7 us per minute, 8148 steps per revolution
    
    //Store values for next time
    _lastTimeStamp = micros();
    _lastPosition  = encoder.read();
    
    return RPM;
}

float MotorGearboxEncoder::_runningAverage(int newValue){
    /*
    
    Compute a running average from the number passed in.
    
    */
    
    int sum = newValue + _oldValue1 + _oldValue2 + _oldValue3 + _oldValue4 + _oldValue5 + _oldValue6 + _oldValue7 + _oldValue8 + _oldValue9 + _oldValue10;
    float runningAverage = (float(sum))/11.0;
    
    _oldValue10 = _oldValue9;
    _oldValue9 = _oldValue8;
    _oldValue8 = _oldValue7;
    _oldValue7 = _oldValue6;
    _oldValue6 = _oldValue5;
    _oldValue5 = _oldValue4;
    _oldValue4 = _oldValue3;
    _oldValue3 = _oldValue2;
    _oldValue2 = _oldValue1;
    _oldValue1 = newValue;
    
    return runningAverage;
}

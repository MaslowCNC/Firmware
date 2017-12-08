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

#include "maslow.h"

Motor::Motor(){
  
  _attachedState = 0;
  
  
}

int  Motor::setupMotor(const int& pwmPin, const int& pin1, const int& pin2){
  
  //store pin numbers as private variables
  _pwmPin = pwmPin;
  _pin1  = pin1;
  _pin2  = pin2;
  _attachedState = 0;
  
  //set pinmodes
  pinMode(_pwmPin,   OUTPUT); 
  pinMode(_pin1,     OUTPUT); 
  pinMode(_pin2,     OUTPUT);
  
  //stop the motor
  digitalWrite(_pin1,    HIGH);
  digitalWrite(_pin2,    LOW) ;
  digitalWrite(_pwmPin,  LOW);
  
  return 1;
}

void Motor::attach(){
    _attachedState = 1;
}

void Motor::detach(){
    _attachedState = 0;
    
    //stop the motor
    digitalWrite(_pin1,    HIGH);
    digitalWrite(_pin2,    LOW) ;
    digitalWrite(_pwmPin,  LOW);
}

int Motor::lastSpeed(){
    /*
    Returns the last speed(voltage) value sent to the pwm
    */
    return _lastSpeed;
}

void Motor::additiveWrite(int speed){
    /*
    Increases/decreases the motor speed by the passed speed amount
    */
    write(_lastSpeed + speed);
}

void Motor::write(int speed){
    /*
    Sets motor speed from input. Speed = 0 is stopped, -255 is full reverse, 255 is full ahead.
    */
    if (_attachedState == 1){
        
        //linearize the motor
        //speed = _convolve(speed);
        
        //set direction range is 0-180
        if (speed > 0){
            digitalWrite(_pin1 , HIGH);
            digitalWrite(_pin2 , LOW );
            speed = speed;
        }
        else if (speed == 0){
            speed = speed;
        }
        else{
            digitalWrite(_pin1 , LOW);
            digitalWrite(_pin2 , HIGH );
            speed = speed;
        }
        
        //enforce range
        speed = constrain(speed, -255, 255);
        
        _lastSpeed = speed; //saves speed for use in additive write
        
        speed = abs(speed); //remove sign from input because direction is set by control pins on H-bridge
        
        int pwmFrequency = round(speed);
        
        if(_pwmPin == 12){
            pwmFrequency = map(pwmFrequency, 0, 255, 0, 1023);  //Scales 0-255 to 0-1023
            Timer1.pwm(2, pwmFrequency);  //Special case for pin 12 due to timer blocking analogWrite()
        }
        else{
            analogWrite(_pwmPin, pwmFrequency);
        }
        
    }
}

void Motor::directWrite(int voltage){
    /*
    Write directly to the motor, ignoring if the axis is attached or any applied calibration.
    */
    
    if (voltage > 0){
        digitalWrite(_pin1 , HIGH);
        digitalWrite(_pin2 , LOW );
    }
    else if (voltage == 0){
        voltage = voltage;
    }
    else{
        digitalWrite(_pin1 , LOW);
        digitalWrite(_pin2 , HIGH );
    }
    
    if(_pwmPin == 12){
        voltage = abs(voltage);
        voltage = map(voltage, 0, 255, 0, 1023);  //Scales 0-255 to 0-1023
        Timer1.pwm(2, voltage);  //Special case for pin 12 due to timer blocking analogWrite()
    }
    else{
        analogWrite(_pwmPin, abs(voltage));
    }
}

int  Motor::attached(){
    
    return _attachedState;
}

int  Motor::_convolve(const int& input){
    /*
    This function distorts the input signal in a manner which is the inverse of the way
    the mechanics of the motor distort it to give a linear response.
    */
    
    int output = input;
    
    int arrayLen = sizeof(_linSegments)/sizeof(_linSegments[1]);
    for (int i = 0; i <= arrayLen - 1; i++){
        if (input > _linSegments[i].negativeBound and input < _linSegments[i].positiveBound){
            output = (input + _linSegments[i].intercept)/_linSegments[i].slope;
            break;
        }
    }
    
    return output;
}

void Motor::setSegment(const int& index, const float& slope, const float& intercept, const int& negativeBound, const int& positiveBound){
    
    //Adds a linearizion segment to the linSegments object in location index
    
    _linSegments[index].slope          =          slope;
    _linSegments[index].intercept      =      intercept;
    _linSegments[index].positiveBound  =  positiveBound;
    _linSegments[index].negativeBound  =  negativeBound;
    
}

LinSegment Motor::getSegment(const int& index){
    return _linSegments[index];
}
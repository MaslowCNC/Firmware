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
    
    Copyright 2014-2016 Bar Smith*/

/*
The GearMotor module imitates the behavior of the Arduino servo module. It allows a gear motor (or any electric motor)
to be a drop in replacement for a continuous rotation servo.

*/

#include "Arduino.h"
#include "GearMotor.h"

int positivePoint =  162;
int negativePoint = -115;

GearMotor::GearMotor(){
  //Serial.println("created gear motor");
  
  _attachedState = 0;
  
  
  //segment 0 less than -115 and more than -256
  
  _linSegments[0].slope          =   0.7;
  _linSegments[0].intercept      =  23.1;
  _linSegments[0].positiveBound  =  -115;
  _linSegments[0].negativeBound  =  -256;
  
  //segment 1 less than   0  and more than -114
  
  _linSegments[1].slope          =    1.9;
  _linSegments[1].intercept      = -137.0;
  _linSegments[1].positiveBound  =      0;
  _linSegments[1].negativeBound  =   -114;
  
  //segment 2 less than  162 and more than   0
   
  _linSegments[2].slope          =   2.32;
  _linSegments[2].intercept      =  46.68;
  _linSegments[2].positiveBound  =    162;
  _linSegments[2].negativeBound  =      0;
  
  //segment 3 less than  256 and more than   161 
  
  _linSegments[3].slope          =   0.54;
  _linSegments[3].intercept      =  113.4;
  _linSegments[3].positiveBound  =    256;
  _linSegments[3].negativeBound  =    161;
}

int GearMotor::setupMotor(int pwmPin, int pin1, int pin2){
  
  //store pin numbers as private variables
  _pwmPin = pwmPin;
  _pin1  = pin1;
  _pin2  = pin2;
  _attachedState = 1;
  
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

void GearMotor::attach(int pin){
    _attachedState = 1;
}

void GearMotor::detach(){
    _attachedState = 0;
    
    //stop the motor
    digitalWrite(_pin1,    HIGH);
    digitalWrite(_pin2,    LOW) ;
    digitalWrite(_pwmPin,  LOW);
}

void GearMotor::write(int speed){
    /*
    Sets motor speed from input. Speed = 0 is stopped, -255 is full reverse, 255 is full ahead.
    */
    if (_attachedState == 1){
        
        //linearize the motor
        speed = _convolve(speed);
        
        //set direction range is 0-180
        if (speed > 0){
            digitalWrite(_pin1 , HIGH);
            digitalWrite(_pin2 , LOW );
            speed = speed + _posBoost;
        }
        else if (speed == 0){
            speed = speed;
        }
        else{
            digitalWrite(_pin1 , LOW);
            digitalWrite(_pin2 , HIGH );
            speed = speed - _negBoost;
        }
        
        //enforce range
        if (speed > 255){speed = 255;}
        
        if (speed < -255)  {speed = -255;  }
        
        speed = abs(speed); //remove sign from input because direction is set by control pins on H-bridge
        
        int pwmFrequency = round(speed);
        
        analogWrite(_pwmPin, pwmFrequency);
        
    }
}

int GearMotor::attached(){
    
    return _attachedState;
}

int GearMotor::_convolve(int input){
    /*
    This function distorts the input signal in a manner which is the inverse of the way
    the mechanics of the motor distort it to give a linear response.
    */
    
    int output = input;
    
    
    //|-255-------|-90---------|0---------|90-----------|255
    
    int arrayLen = sizeof(_linSegments)/sizeof(_linSegments[1]);
    for (int i = 0; i <= arrayLen; i++){
        if (input > _linSegments[i].negativeBound and input < _linSegments[i].positiveBound){
            Serial.println('T');
        }
    }
    
    /*if (input < negativePoint){
        //do most negative thing
        output = (input +23.1)/0.7;
        return output;
    }
    else if(input < 0){
        //do less negative conversion
        output = (input-137.0)/1.9;
        return output;
    }
    else if(input > positivePoint){
        //do more positive thing
        output = (input - 113.4)/0.54;
        return output;
    }
    else if(input > 0){
        //do less positive thing
        output = (input + 46.68)/2.32;
        return output;
    }*/
    
    
    return output;
}

void GearMotor::setBoost(int negBoost, int posBoost){
    
    _negBoost = negBoost;
    _posBoost = posBoost;
}


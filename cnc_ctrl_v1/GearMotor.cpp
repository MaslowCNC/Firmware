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

GearMotor::GearMotor(){
  //Serial.println("created gear motor");
  
  _attachedState = 0;
  
}

int GearMotor::setupMotor(int pwmPin, int pin1, int pin2){
  
  //store pin numbers as private variables
  _pwmPin = pwmPin;
  _pin1  = pin1;
  _pin2  = pin2;
  _attachedState = 1;
  
  //set pinmodes
  pinMode(_pwmPin,   OUTPUT); 
  pinMode(_pin1,   OUTPUT); 
  pinMode(_pin2, OUTPUT);
  
  //stop the motor
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, LOW) ;
  digitalWrite(_pwmPin, LOW);
  
  return 1;
}

void GearMotor::attach(int pin){
    _attachedState = 1;
}

void GearMotor::detach(){
    _attachedState = 0;
    
    //stop the motor
    digitalWrite(_pin1, HIGH);
    digitalWrite(_pin2, LOW) ;
    digitalWrite(_pwmPin, LOW);
}

void GearMotor::write(int speed){
    /*
    Mirrors the behavior of the servo.write() function. Speed = 90 is stopped, 0 is full reverse, 180 is full ahead.
    */
    if (_attachedState == 1){
        
        if (speed > 180){
            speed = 180;
        }
        
        if (speed < 0){
            speed = 0;
        }
        
        //set direction range is 0-180
        if (speed > 90){
            digitalWrite(_pin1 , HIGH);
            digitalWrite(_pin2 , LOW );
        }
        else{
            digitalWrite(_pin1 , LOW);
            digitalWrite(_pin2 , HIGH );
        }
        
        
        speed = (speed - 90); //range is +-0-90
        speed = abs(speed); //remove sign from input
        
        int pwmFrequency;
        
        float scalor = (255.0-_deadBand)/90.0;
        pwmFrequency = round((scalor*speed) + _deadBand);
        
        analogWrite(_pwmPin, pwmFrequency);
        
        
    }
    
}

int GearMotor::attached(){
    
    return _attachedState;
}


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
#include "GearMotor.h"

GearMotor::GearMotor(int pwmPin, int pin1, int pin2)
{
  Serial.println("created gear motor");
  Serial.println(pwmPin);
  Serial.println(pin1);
  Serial.println(pin2);
  
  //store pin numbers as private variables
  _pwmPin = pwmPin;
  _pin1  = pin1;
  _pin2  = pin2;
  
  //set pinmodes
  pinMode(_pwmPin,   OUTPUT); 
  pinMode(_pin1,   OUTPUT); 
  pinMode(_pin2, OUTPUT);
  
  //stop the motor
  digitalWrite(_pin1, HIGH);
  digitalWrite(_pin2, LOW) ;
  digitalWrite(_pwmPin, LOW);
  
}

void GearMotor::attach(int pin){
    //Serial.println("gear motor attached");
}

void GearMotor::detach(){
    //Serial.println("gear motor detached");
}

void GearMotor::write(int speed){
    //Serial.print("gm: ");
    
    int pwmFrequency = (speed - 90)*(2034/90);
    
    //Serial.println(speed);
    //Serial.println(pwmFrequency);
    
    analogWrite(_pwmPin, abs(pwmFrequency));
    
    if (pwmFrequency > 0){
        digitalWrite(_pin1 , LOW);
        digitalWrite(_pin2 , HIGH );
    }
    else{
        digitalWrite(_pin1 , HIGH);
        digitalWrite(_pin2 , LOW );
    }
    
}

int GearMotor::attached(){
    Serial.println("gear motor attached");
    return 1;
}


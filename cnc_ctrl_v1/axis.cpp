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
#include "PID_v1.h"

#define FORWARD 1
#define BACKWARD -1

//PID setup 
double _pidSetpoint, _pidInput, _pidOutput;
double Kp=300, Ki=0, Kd=10;
PID _pidController(&_pidInput, &_pidOutput, &_pidSetpoint, Kp, Ki, Kd, DIRECT);

Axis::Axis(int pwmPin, int directionPin1, int directionPin2, int encoderDirection, int encoderPin, String axisName){
    
    //initialize motor
    _motor      = GearMotor();
    _motor.setupMotor(pwmPin, directionPin1, directionPin2);
    //_motor.write(0);
    
    //initialize pins
    pinMode(encoderPin, INPUT);
    
    //initialize PID controller
    double _pidSetpoint, _pidInput, _pidOutput;
    double Kp=300, Ki=0, Kd=10;
    PID _pidController(&_pidInput, &_pidOutput, &_pidSetpoint, Kp, Ki, Kd, DIRECT);
    
    //initialize variables
    _direction    = encoderDirection;
    _encoderPin   = encoderPin;
    _axisName     = axisName;
    _axisPosition = 0.0;
    _axisTarget   = 0.0;
    _direction    = BACKWARD;
    
}

void   Axis::initializePID(){
    _pidController.SetMode(AUTOMATIC);
    _pidController.SetOutputLimits(-90, 90);
}

int    Axis::write(float targetPosition){
    
    _pidInput      =  _axisPosition;
    _pidSetpoint   =  targetPosition;
    
    
    
    bool pidreturn = _pidController.Compute();
    
    //Serial.println(targetPosition);
    
    _motor.write(90 + _pidOutput);
    
    return 1;
}

float  Axis::read(){
    return _axisPosition;
}

int    Axis::set(float newAxisPosition){
    Serial.println (" would manually reset axis pos here");
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
    _motor.detach();
    return 1;
}

int    Axis::attach(){
     _motor.attach(1);
     return 1;
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
    
    //if (duration == 0){
    //    Serial.print(_axisName);
    //    Serial.println(" timed out");
    //}
    
    if (duration < 10){
        duration = 0;
    }

    return duration;
}

int    Axis::returnPidMode(){
    return _pidController.GetMode();
}
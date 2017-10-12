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

MotorGearboxEncoder::MotorGearboxEncoder(const int& pwmPin, const int& directionPin1, const int& directionPin2, const int& encoderPin1, const int& encoderPin2)
:
encoder(encoderPin1,encoderPin2)
{
    
    //initialize motor
    motor.setupMotor(pwmPin, directionPin1, directionPin2);
    motor.write(0);
    
    //initialize the PID
    _PIDController.setup(&_currentSpeed, &_pidOutput, &_targetSpeed, _Kp, _Ki, _Kd, P_ON_E, DIRECT);
    initializePID();
    
    
}

void  MotorGearboxEncoder::write(const float& speed){
    /*
    Command the motor to turn at the given speed. Should be RPM is PWM right now.
    */
    
    _targetSpeed = speed;
    
}

void   MotorGearboxEncoder::initializePID(){
    //setup positive PID controller
    _PIDController.SetMode(AUTOMATIC);
    _PIDController.SetOutputLimits(-255, 255);
    _PIDController.SetSampleTime(10);
}

void  MotorGearboxEncoder::computePID(){
    /*
    Recompute the speed control PID loop and command the motor to move.
    */
    _currentSpeed = computeSpeed();
    
    /*if (millis() < 8000){
        _targetSpeed = 0;
    }
    else if (millis() < 12000){
        _targetSpeed = 10;
    }
    else if (millis() < 18000){
         _targetSpeed = 0;
    }
    else if(millis() < 24000){
        _targetSpeed = float((millis() - 18000))/400.0;
    }
    else if (millis() < 32000){
        _targetSpeed = 0;
    }
    else if (millis() < 40000){
        _targetSpeed = 10;
    }
    else if (millis() < 48000){
        _targetSpeed = 0;
    }
    else if (millis() < 56000){
        _targetSpeed = -10;
    }
    else if (millis() < 64000){
        _targetSpeed = 0;
    }
    else if (millis() < 72000){
        _targetSpeed = 10;
    }
    else if (millis() < 80000){
        _targetSpeed = 0;
    }
    else{
        _targetSpeed = 0;
    }*/
    
    // Between these speeds the motor is incapable of turning and it only
    // causes the Iterm in the PID calculation to wind up

    _PIDController.Compute();
        
    /*if(_motorName[0] == 'R'){
        //Serial.print(_currentSpeed);
        //Serial.print(" ");
        Serial.println(_targetSpeed);
    }*/
    
    //motor.attach();
    motor.write(_pidOutput);
}

void  MotorGearboxEncoder::setPIDValues(float KpV, float KiV, float KdV){
    /*
    
    Set PID tuning values
    
    */
    
    _Kp = KpV;
    _Ki = KiV;
    _Kd = KdV;
    
    _PIDController.SetTunings(_Kp, _Ki, _Kd, P_ON_E);
}

String  MotorGearboxEncoder::getPIDString(){
    /*
    
    Get PID tuning values
    
    */
    String PIDString = "Kp=";
    return PIDString + _Kp + ",Ki=" + _Ki + ",Kd=" + _Kd;
}

void MotorGearboxEncoder::setPIDAggressiveness(float aggressiveness){
    /*
    
    The setPIDAggressiveness() function sets the aggressiveness of the PID controller to
    compensate for a change in the load on the motor.
    
    */
    
    _PIDController.SetTunings(aggressiveness*_Kp, _Ki, _Kd, P_ON_E);
    
}

void MotorGearboxEncoder::setEncoderResolution(float resolution){
    /*
    
    Change the encoder resolution
    
    */
    
    _encoderStepsToRPMScaleFactor = 60000000.0/resolution; //6*10^7 us per minute divided by 8148 steps per revolution
    
}

float MotorGearboxEncoder::computeSpeed(){
    /*
    
    Returns the motors speed in RPM since the last time this function was called
    
    */
    
    float currentPosition = encoder.read();
    float currentMicros = micros();
    
    float distMoved   =  currentPosition - _lastPosition;
    float RPM = 0;
    if (distMoved > 3 || distMoved < -3){ 
    
        unsigned long timeElapsed =  currentMicros - _lastTimeStamp;
        //Compute the speed in RPM
        RPM = (_encoderStepsToRPMScaleFactor*distMoved)/float(timeElapsed);
    
    }
    else {
        float elapsedTime = encoder.elapsedTime();

        RPM = 0 ;
        if (elapsedTime != 0){
          RPM = _encoderStepsToRPMScaleFactor / elapsedTime;
        }
    }
    RPM = RPM * -1.0;
    
    //Store values for next time
    _lastTimeStamp = currentMicros;
    _lastPosition  = currentPosition;
    _lastRPM = RPM;
    
     // This dampens some of the effects of quantization without having 
     // a big effect on other changes 
     if (RPM - _lastRPM < -1){ RPM + .5;}
     else if (RPM - _lastRPM > 1){RPM - .5;}
    
    return RPM;
}

void MotorGearboxEncoder::setName(const char& newName){
    /*
    Set the name for the object
    */
    _motorName = newName;
}

char MotorGearboxEncoder::name(){
    /*
    Get the name for the object
    */
    return _motorName;
}

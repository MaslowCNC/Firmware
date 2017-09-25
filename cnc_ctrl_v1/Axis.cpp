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
#include "Axis.h"

#define FORWARD 1
#define BACKWARD -1

#define EEPROMVALIDDATA 56
#define SIZEOFFLOAT      4
#define SIZEOFLINSEG    17

Axis::Axis(const int& pwmPin, const int& directionPin1, const int& directionPin2, const int& encoderPin1, const int& encoderPin2, const String& axisName, const int& eepromAdr)
:
motorGearboxEncoder(pwmPin, directionPin1, directionPin2, encoderPin1, encoderPin2)
{
    
    _pidController.setup(&_pidInput, &_pidOutput, &_pidSetpoint, 0, 0, 0, P_ON_E, REVERSE);
    
    //initialize variables
    _direction    = FORWARD;
    _axisName     = axisName;
    _eepromAdr    = eepromAdr;
    
    initializePID();
    
    motorGearboxEncoder.setName(_axisName);
}

void   Axis::loadPositionFromMemory(){
        /*
        
        Reload the last known position for the axis
        
        */
        
        //If a valid position has been stored
        if (EEPROM.read(_eepromAdr) == EEPROMVALIDDATA){
            float f = 0.00f;
            f = EEPROM.get(_eepromAdr + SIZEOFFLOAT, f );
            set(f);
        }
        
}

void   Axis::initializePID(){
    _pidController.SetMode(AUTOMATIC);
    _pidController.SetOutputLimits(-17, 17);
    _pidController.SetSampleTime(10);
}

void    Axis::write(const float& targetPosition){
    
    _pidSetpoint   =  targetPosition/_mmPerRotation;
    return;
}

float  Axis::read(){
    //returns the true axis position
    
    return (motorGearboxEncoder.encoder.read()/_encoderSteps)*_mmPerRotation;
    
}

float  Axis::target(){
    //returns the axis target
    return _axisTarget*_mmPerRotation;
}

float  Axis::setpoint(){
    return _pidSetpoint*_mmPerRotation;
}

int    Axis::set(const float& newAxisPosition){
    
    //reset everything to the new value
    _axisTarget   =  newAxisPosition/_mmPerRotation;
    _pidSetpoint  =  newAxisPosition/_mmPerRotation;
    motorGearboxEncoder.encoder.write((newAxisPosition*_encoderSteps)/_mmPerRotation);
    
}

void   Axis::computePID(){
    
    
    if (_disableAxisForTesting){
        return;
    }
    
    if (_detectDirectionChange(_pidSetpoint)){ //this determines if the axis has changed direction of movement and flushes the accumulator in the PID if it has
        _pidController.FlipIntegrator();
    }
    
    _pidInput      =  motorGearboxEncoder.encoder.read()/_encoderSteps;
    
    _pidController.Compute();
    
    motorGearboxEncoder.write(_pidOutput);
    
    /*if(_axisName[0] == 'L'){
        Serial.println("*");
        Serial.println(_pidInput);
        Serial.println(_pidSetpoint);
        Serial.println(_pidOutput);
    }*/
    
    
    
    motorGearboxEncoder.computePID();
    
}

void   Axis::setPIDValues(float KpPos, float KiPos, float KdPos, float propWeight, float KpV, float KiV, float KdV){
    /*
    
    Sets the positional PID values for the axis
    
    */
    _Kp = KpPos;
    _Ki = KiPos;
    _Kd = KdPos;
    
    _pidController.SetTunings(_Kp, _Ki, _Kd, propWeight);
    
    motorGearboxEncoder.setPIDValues(KpV, KiV, KdV);
}

void   Axis::setPIDAggressiveness(float aggressiveness){
    /*
    
    The setPIDAggressiveness() function sets the aggressiveness of the PID controller to
    compensate for a change in the load on the motor.
    
    */
    
    motorGearboxEncoder.setPIDAggressiveness(aggressiveness);
}

float  Axis::error(){

    float encoderErr = (motorGearboxEncoder.encoder.read()/_encoderSteps) - _pidSetpoint;

    return encoderErr *_mmPerRotation;
}

void   Axis::changePitch(const float& newPitch){
    /*
    Reassign the distance moved per-rotation for the axis.
    */
    _mmPerRotation = newPitch;
}

void   Axis::changeEncoderResolution(const int& newResolution){
    /*
    Reassign the encoder resolution for the axis.
    */
    _encoderSteps = newResolution;
    
    //push to the gearbox for calculating RPM
    motorGearboxEncoder.setEncoderResolution(newResolution);
    
}

int    Axis::detach(){
    
    if (motorGearboxEncoder.motor.attached()){
        float f = read();  //Store the axis position
        EEPROM.put(_eepromAdr + SIZEOFFLOAT, f);
        EEPROM.update(_eepromAdr, EEPROMVALIDDATA);
    }
    
    motorGearboxEncoder.motor.detach();
    
    return 1;
}

int    Axis::attach(){
     motorGearboxEncoder.motor.attach();
     return 1;
}

bool   Axis::attached(){
    /*
    
    Returns true if the axis is attached, false if it is not.
    
    */
    
    return motorGearboxEncoder.motor.attached();
}

void   Axis::hold(){
    int timeout   = 2000;
    
    if (millis() - _timeLastMoved < timeout){
        write(_axisTarget*_mmPerRotation);
    }
    else{
        detach();
    }
    
}

void   Axis::endMove(const float& finalTarget){
    
    _timeLastMoved = millis();
    _axisTarget    = finalTarget/_mmPerRotation;
    
}

void   Axis::wipeEEPROM(){
    /*
    
    Over-write all the values stored in EEPROM to return the machine to a known state.
    
    */
    
    int i = 0;
    while(i < 50){
        EEPROM.update(_eepromAdr + i, 0);
        i++;
    }
    
    Serial.print(_axisName);
    Serial.println(F(" EEPROM erased"));
}

int    Axis::_detectDirectionChange(const float& _pidSetpoint){
    
    float difference = _pidSetpoint - _oldSetpoint;
    
    if(difference == 0){
        return 0;
    }
    
    int direction;
    if(difference > 0){
        direction = 1;
    }
    else{
        direction = 0;
    }
    
    int retVal = 0;
    if(direction != _oldDir){
        retVal = 1;
    }
    
    _oldSetpoint = _pidSetpoint;
    _oldDir = direction;
    
    return retVal;
}

void   Axis::test(){
    /*
    Test the axis by directly commanding the motor and observing if the encoder moves
    */
    
    Serial.print(F("Testing "));
    Serial.print(_axisName);
    Serial.println(F(" motor:"));
    
    //print something to prevent the connection from timing out
    Serial.print(F("<Idle,MPos:0,0,0,WPos:0.000,0.000,0.000>"));
    
    int i = 0;
    double encoderPos = motorGearboxEncoder.encoder.read(); //record the position now
    
    //move the motor
    while (i < 1000){
        motorGearboxEncoder.motor.directWrite(255);
        i++;
        delay(1);
    }
    
    //check to see if it moved
    if(encoderPos - motorGearboxEncoder.encoder.read() > 500){
        Serial.println(F("Direction 1 - Pass"));
    }
    else{
        Serial.println(F("Direction 1 - Fail"));
    }
    
    //record the position again
    encoderPos = motorGearboxEncoder.encoder.read();
    Serial.print(F("<Idle,MPos:0,0,0,WPos:0.000,0.000,0.000>"));
    
    //move the motor in the other direction
    i = 0;
    while (i < 1000){
        motorGearboxEncoder.motor.directWrite(-255);
        i++;
        delay(1);
    }
    
    //check to see if it moved
    if(encoderPos - motorGearboxEncoder.encoder.read() < -500){
        Serial.println(F("Direction 2 - Pass"));
    }
    else{
        Serial.println(F("Direction 2 - Fail"));
    }
    
    //stop the motor
    motorGearboxEncoder.motor.directWrite(0);
    Serial.print(F("<Idle,MPos:0,0,0,WPos:0.000,0.000,0.000>"));
}

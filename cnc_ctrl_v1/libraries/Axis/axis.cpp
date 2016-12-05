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

#define NUMBER_OF_ENCODER_STEPS 8148.0 



Axis::Axis(int pwmPin, int directionPin1, int directionPin2, int encoderPin1, int encoderPin2, String axisName, int eepromAdr, float mmPerRotation)
:
_encoder(encoderPin1,encoderPin2)
{
    
    //initialize motor
    _motor.setupMotor(pwmPin, directionPin1, directionPin2);
    
    _pidController.setup(&_pidInput, &_pidOutput, &_pidSetpoint, _Kp, _KiFar, _Kd, REVERSE);
    
    //initialize variables
    _direction    = FORWARD;
    _axisName     = axisName;
    _axisTarget   = 0.0;
    _eepromAdr    = eepromAdr;
    _mmPerRotation= mmPerRotation;
    
    //load position
    if (EEPROM.read(EEPROMFLAG) == EEPROMVALIDDATA){
        set(_readFloat(_eepromAdr));
    }
    
    if (_axisName == "Left-axis"){
        _motor.setSegment(0 ,  0.7,    23.1,  -256,  -115);
        _motor.setSegment(1 ,  1.9,  -137.0,  -114,     0);
        _motor.setSegment(2 , 2.32,   46.68,     0,   162);
        _motor.setSegment(3 , 0.54,  -113.4,   161,   256);
    }
    
    if (_axisName == "Right-axis"){
        _motor.setSegment(0 ,  .48,   131.9,  -256,  -174);
        _motor.setSegment(1 ,  2.4,   -39.8,  -175,     0);
        _motor.setSegment(3 ,  1.9,   117.2,     0,   134);
        _motor.setSegment(2 ,  .69,   -44.2,   133,   256);
    }
}

void   Axis::initializePID(){
    _pidController.SetMode(AUTOMATIC);
    _pidController.SetOutputLimits(-255, 255);
}

int    Axis::write(float targetPosition){
    
    _pidSetpoint   =  targetPosition/_mmPerRotation;
    
    int acceptableError = 20;
    if (abs( ((_encoder.read()/NUMBER_OF_ENCODER_STEPS) - _pidSetpoint)*1000 ) < acceptableError){
        return 1;
    }
    else{
        return 0;
    }
}

float  Axis::read(){
    
    if (_motor.attached()){
        return (_encoder.read()/NUMBER_OF_ENCODER_STEPS)*_mmPerRotation;
    }
    else{
        return (_encoder.read()/NUMBER_OF_ENCODER_STEPS)*_mmPerRotation;
    }
}

float  Axis::target(){
    return _axisTarget*_mmPerRotation;
}

float  Axis::setpoint(){
    return _pidSetpoint*_mmPerRotation;
}

int    Axis::set(float newAxisPosition){
    _axisTarget   =  newAxisPosition/_mmPerRotation;
    _encoder.write((newAxisPosition*NUMBER_OF_ENCODER_STEPS)/_mmPerRotation);
}

void   Axis::computePID(){
    
    if (_disableAxisForTesting){
        return;
    }
    
    if (_change(_sign(_oldSetpoint - _pidSetpoint))){ //this determines if the axis has changed direction of movement and flushes the acumulator in the PID if it has
        _pidController.FlipIntegrator();
    }
    _oldSetpoint = _pidSetpoint;
    
    //antiWindup code
    if (abs(_pidOutput) > 20){ //if the actuator is saturated
        _pidController.SetTunings(_Kp, _KiFar, _Kd); //disable the integration term
    }
    else{
        if (abs(_pidInput - _pidSetpoint) < .02){
            //This second check catches the corner case where the setpoint has just jumped, but compute has not been run yet
            _pidController.SetTunings(_Kp, _KiClose, _Kd);
        }
        if (abs(_pidInput - _pidSetpoint) < .06){
            _pidController.SetTunings(_Kp, _KiMid, _Kd);
        }
    }
    
    _pidInput      =  _encoder.read()/NUMBER_OF_ENCODER_STEPS;
    _pidController.Compute();
    
    _motor.write(_pidOutput);
    
}

float  Axis::error(){
    return abs((_encoder.read()/NUMBER_OF_ENCODER_STEPS) - _pidSetpoint)*_mmPerRotation;
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

int    Axis::_sign(float val){
    if (val < 0) return -1;
    if (val==0) return 0;
    return 1;
}

int    Axis::_change(float val){
    if (val != _oldVal){
        _oldVal = val;
        return true;
    }
    else{
        _oldVal = val;
        return false;
    }
}

void   Axis::printBoost(){
    
    _disableAxisForTesting = true;
    
    for(int i = -255; i < 255; i = i+10){
        Serial.print(i);
        Serial.print(" -> ");
        Serial.println(_motor._convolve(i));
    }
     
    _disableAxisForTesting = false;
}

void   Axis::computeMotorResponse(){
    Serial.println("Compute motor response");
    
    float scale = 255/measureMotorSpeed(255); //Y3*scale = 255 -> scale = 255/Y3
    
    
    int i = 15;
    float motorSpeed;                                                                                                                                     
    while (i < 255){
        motorSpeed = measureMotorSpeed(i);
        
        if (motorSpeed > 0){
            break;
        }
        
        i++;
    }
    
    float X1 = i;
    i = 0;
    float Y1 = scale*motorSpeed;
    
    float X2 = (255 - X1)/2;
    float Y2 = scale*measureMotorSpeed(X2);
    
    float X3 = 255;
    float Y3 = 255;
    
    float M1 = (Y1 - Y2)/(X1 - X2);
    float M2 = (Y2 - Y3)/(X2 - X3);
    
    float I1 = Y1 - (M1*X1);
    float I2 = Y2 - (M2*X2);
    
    Serial.print("First point: (");
    Serial.print(X1);
    Serial.print(", ");
    Serial.print(Y1);
    Serial.println(")");
    
    Serial.print("Second point: (");
    Serial.print(X2);
    Serial.print(", ");
    Serial.print(Y2);
    Serial.println(")");
    
    Serial.print("Third point: (");
    Serial.print(X3);
    Serial.print(", ");
    Serial.print(Y3);
    Serial.println(")");
    
    Serial.print("Slope 1: ");
    Serial.println(M1);
    
    Serial.print("Slope 2: ");
    Serial.println(M2);
    
    Serial.print("Intercept 1: ");
    Serial.println(I1);
    
    Serial.print("Intercept 2: ");
    Serial.println(I2);
    
    _motor.setSegment(2 , M1, I1,    0, Y2);
    _motor.setSegment(3 , M2, I2, Y2-1, Y3);
}

float  Axis::measureMotorSpeed(int speed){
    /*
    Returns the motors speed in RPM at a given input value
    */
    
    _disableAxisForTesting = true;
    attach();
    
    int numberOfStepsToTest = 2000;
    int timeOutMS           = 30000;
    
    //Serial.print(_axisName);
    //Serial.print(" cmd ");
    //Serial.print(speed);
    //Serial.print("->");
    
    //run the motor for numberOfStepsToTest steps positive and record the time taken
    long originalEncoderPos  = _encoder.read();
    long startTime = millis();
    while (abs(originalEncoderPos - _encoder.read()) < numberOfStepsToTest){
        _motor.write(speed);
        if (millis() - startTime > timeOutMS ){break;}
    }
    int posTime = millis() - startTime;
    
    float RPM = 60.0*1000.0 * 1.0/(4.0*float(posTime));
    
    if (posTime > timeOutMS){
        RPM = 0;
    }
    
    //Serial.println(RPM);
    
    //pause
    _motor.write(0);
    delay(200);
    
    //reset to start point
    _disableAxisForTesting = false;
    _timeLastMoved = millis();
    for (long startTime = millis(); millis() - startTime < 2000; millis()){
        hold();
        delay(10);
    }
    
    return RPM;
}
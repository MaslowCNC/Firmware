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
#define SIZEOFFLOAT      4
#define SIZEOFLINSEG    17

Axis::Axis(int pwmPin, int directionPin1, int directionPin2, int encoderPin1, int encoderPin2, String axisName, int eepromAdr, float mmPerRotation, float encoderSteps)
:
motorGearboxEncoder(pwmPin, directionPin1, directionPin2, encoderPin1, encoderPin2)
{
    
    _pidController.setup(&_pidInput, &_pidOutput, &_pidSetpoint, _Kp, _KiFar, _Kd, REVERSE);
    
    //initialize variables
    _direction    = FORWARD;
    _axisName     = axisName;
    _axisTarget   = 0.0;
    _eepromAdr    = eepromAdr;
    _mmPerRotation= mmPerRotation;
    _encoderSteps = encoderSteps;
    
    //load position
    if (EEPROM.read(_eepromAdr) == EEPROMVALIDDATA){
        set(_readFloat(_eepromAdr + SIZEOFFLOAT));
    }
    
    _readAllLinSegs(_eepromAdr);
    
    initializePID();
}

void   Axis::initializePID(){
    _pidController.SetMode(AUTOMATIC);
    _pidController.SetOutputLimits(-255, 255);
    _pidController.SetSampleTime(10);
}

int    Axis::write(float targetPosition){
    
    _pidSetpoint   =  targetPosition/_mmPerRotation;
    return 1;
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

int    Axis::set(float newAxisPosition){
    
    //reset everything to the new value
    _axisTarget   =  newAxisPosition/_mmPerRotation;
    _pidSetpoint  =  newAxisPosition/_mmPerRotation;
    motorGearboxEncoder.encoder.write((newAxisPosition*_encoderSteps)/_mmPerRotation);
    
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
    
    _pidInput      =  motorGearboxEncoder.encoder.read()/_encoderSteps;
    
    _pidController.Compute();
    
    motorGearboxEncoder.motor.write(_pidOutput);
    
    motorGearboxEncoder.computeSpeed();
    
}

float  Axis::error(){
    return abs((motorGearboxEncoder.encoder.read()/_encoderSteps) - _pidSetpoint)*_mmPerRotation;
}

void   Axis::changePitch(float newPitch){
    /*
    Reassign the distance moved per-rotation for the axis.
    */
    _mmPerRotation = newPitch;
}

void   Axis::changeEncoderResolution(int newResolution){
    /*
    Reassign the encoder resolution for the axis.
    */
    _encoderSteps = newResolution;
    
}

int    Axis::detach(){
    
    if (motorGearboxEncoder.motor.attached()){
        _writeFloat (_eepromAdr+SIZEOFFLOAT, read());
        EEPROM.write(_eepromAdr, EEPROMVALIDDATA);
        
        _writeAllLinSegs(_eepromAdr);
        
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
    
    //Writes a floating point number into the eeprom memory by splitting it into four one byte chunks and saving them
    
    union{
        byte b[4];
        float f;
    } data;
    data.f = x;
    for(int i = 0; i < 4; i++){
        EEPROM.write(addr+i, data.b[i]);
    }
}

void   Axis::_writeLinSeg(unsigned int addr, LinSegment linSeg){
    
    //flag that data is good
    EEPROM.write(addr, EEPROMVALIDDATA);
    
    _writeFloat(addr + 1                , linSeg.slope);
    _writeFloat(addr + 1 + 1*SIZEOFFLOAT, linSeg.intercept);
    _writeFloat(addr + 1 + 2*SIZEOFFLOAT, linSeg.positiveBound);
    _writeFloat(addr + 1 + 3*SIZEOFFLOAT, linSeg.negativeBound);
}

void   Axis::_writeAllLinSegs(unsigned int addr){
    /*
    Write all of the LinSegment objects which define the motors response into the EEPROM
    */
    
    addr = addr + 2*SIZEOFFLOAT;
    
    //Write into memory
    _writeLinSeg(addr                 , motorGearboxEncoder.motor.getSegment(0));
    _writeLinSeg(addr + 1*SIZEOFLINSEG, motorGearboxEncoder.motor.getSegment(1));
    _writeLinSeg(addr + 2*SIZEOFLINSEG, motorGearboxEncoder.motor.getSegment(2));
    _writeLinSeg(addr + 3*SIZEOFLINSEG, motorGearboxEncoder.motor.getSegment(3));
    
}

LinSegment   Axis::_readLinSeg(unsigned int addr){
    /*
    Read a LinSegment object from the EEPROM
    */
    
    LinSegment linSeg;
    
    //check that data is good
    if (EEPROM.read(addr) == EEPROMVALIDDATA){
        linSeg.slope         =  _readFloat(addr + 1                );
        linSeg.intercept     =  _readFloat(addr + 1 + 1*SIZEOFFLOAT);
        linSeg.positiveBound =  _readFloat(addr + 1 + 2*SIZEOFFLOAT);
        linSeg.negativeBound =  _readFloat(addr + 1 + 3*SIZEOFFLOAT);
    }
    
    return linSeg;
}

void   Axis::_readAllLinSegs(unsigned int addr){
   /*
   Read back all the LinSegment objects which define the motors behavior from the EEPROM
   */
    
    addr = addr + 8;
    
    LinSegment linSeg;
    
    for (int i = 0; i < 4; i++){
        linSeg = _readLinSeg (addr + i*SIZEOFLINSEG);
        
        motorGearboxEncoder.motor.setSegment(i, linSeg.slope, linSeg.intercept,  linSeg.negativeBound, linSeg.positiveBound);
    }
}

void   Axis::wipeEEPROM(){
    /*
    
    Over-write all the values stored in EEPROM to return the machine to a known state.
    
    */
    
    int i = 0;
    while(i < 50){
        EEPROM.write(_eepromAdr + i, 0);
        i++;
    }
    
    Serial.print(_axisName);
    Serial.println(" EEPROM erased");
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
        Serial.println(motorGearboxEncoder.motor._convolve(i));
    }
     
    _disableAxisForTesting = false;
}

void   Axis::test(){
    /*
    Test the axis by directly commanding the motor and observing if the encoder moves
    */
    
    Serial.print("Testing ");
    Serial.print(_axisName);
    Serial.println(" motor:");
    
    //print something to prevent the connection from timing out
    Serial.print("<Idle,MPos:0,0,0,WPos:0.000,0.000,0.000>");
    
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
        Serial.println("Direction 1 - Pass");
    }
    else{
        Serial.println("Direction 1 - Fail");
    }
    
    //record the position again
    encoderPos = motorGearboxEncoder.encoder.read();
    Serial.print("<Idle,MPos:0,0,0,WPos:0.000,0.000,0.000>");
    
    //move the motor in the other direction
    i = 0;
    while (i < 1000){
        motorGearboxEncoder.motor.directWrite(-255);
        i++;
        delay(1);
    }
    
    //check to see if it moved
    if(encoderPos - motorGearboxEncoder.encoder.read() < -500){
        Serial.println("Direction 2 - Pass");
    }
    else{
        Serial.println("Direction 2 - Fail");
    }
    
    //stop the motor
    motorGearboxEncoder.motor.directWrite(0);
    Serial.print("<Idle,MPos:0,0,0,WPos:0.000,0.000,0.000>");
}

void   Axis::computeMotorResponse(){
    
    //remove whatever transform is applied
    motorGearboxEncoder.motor.setSegment(0 , 1, 0, 0, 0);
    motorGearboxEncoder.motor.setSegment(1 , 1, 0, 0, 0);
    motorGearboxEncoder.motor.setSegment(2 , 1, 0, 0, 0);
    motorGearboxEncoder.motor.setSegment(3 , 1, 0, 0, 0);
    
    //In the positive direction
    //-----------------------------------------------------------------------------------
    
    float scale = 255/measureMotorSpeed(255); //Y3*scale = 255 -> scale = 255/Y3
    
    int stallPoint;
    float motorSpeed;
    
    int upperBound = 255; //the whole range is valid
    int lowerBound =   0;
    
    while (true){ //until a value is found
        Serial.print("Testing: ");
        Serial.println((upperBound + lowerBound)/2);
        
        motorSpeed = measureMotorSpeed((upperBound + lowerBound)/2);
        
        if (motorSpeed == 0){                               //if the motor stalled
            lowerBound = (upperBound + lowerBound)/2;           //shift lower bound to be the guess
            Serial.println("- stall");
        }
        else{                                               //if the motor didn't stall
            upperBound = (upperBound + lowerBound)/2;           //shift upper bound to be the guess
            Serial.println("- good");
        }
        
        if (upperBound - lowerBound <= 1){                  //when we've converged on the first point which doesn't stall
            break;                                              //exit loop
        }
    }
    
    stallPoint = upperBound;
    
    Serial.print("decided on final value of: ");
    Serial.println(stallPoint);
    
    
    //compute a model of the motor from the given data points
    float X1 = stallPoint;
    float Y1 = scale*motorSpeed;
    
    float X2 = (255 - X1)/2;
    float Y2 = scale*measureMotorSpeed(X2);
    
    float X3 = 255;
    float Y3 = 255;
    
    float M1 = (Y1 - Y2)/(X1 - X2);
    float M2 = (Y2 - Y3)/(X2 - X3);
    
    float I1 = -1*(Y1 - (M1*X1));
    float I2 = -1*(Y2 - (M2*X2));
    
    
    //Apply the model to the motor
    motorGearboxEncoder.motor.setSegment(2 , M1, I1,    0,   Y2);
    motorGearboxEncoder.motor.setSegment(3 , M2, I2, Y2-1, Y3+1);
    
    
    //In the negative direction
    //-----------------------------------------------------------------------------
    
    scale = -255/measureMotorSpeed(-255); //Y3*scale = 255 -> scale = 255/Y3
    
    upperBound =      0; //the whole range is valid
    lowerBound =   -255;
    
    while (true){ //until a value is found
        
        Serial.print("Testing: ");
        Serial.println((upperBound + lowerBound)/2);
        
        motorSpeed = measureMotorSpeed((upperBound + lowerBound)/2);
        if (motorSpeed == 0){                               //if the motor stalled
            upperBound = (upperBound + lowerBound)/2;           //shift lower bound to be the guess
            Serial.println("-stall");
        }
        else{                                               //if the motor didn't stall
            lowerBound = (upperBound + lowerBound)/2;           //shift upper bound to be the guess
            Serial.println("-good");
        }
        
        if (upperBound - lowerBound <= 1){                  //when we've converged on the first point which doesn't stall
            break;                                              //exit loop
        }
    }
    
    stallPoint = lowerBound;
    
    Serial.print("decided on a final value of: ");
    Serial.println(stallPoint);
    
    //At this point motorSpeed is the speed in RPM at the value i which is just above the stall speed
    
    
    //Compute a model for the motor's behavior using the given data-points
    X1 = stallPoint;
    Y1 = scale*motorSpeed;
    
    X2 = (-255 - X1)/3 + X1;
    Y2 = scale*measureMotorSpeed(X2);
    
    X3 = -255;
    Y3 = -255;
    
    M1 = (Y1 - Y2)/(X1 - X2);
    M2 = (Y2 - Y3)/(X2 - X3);
    
    I1 = -1*(Y1 - (M1*X1));
    I2 = -1*(Y2 - (M2*X2));
    
    
    //Apply the model to the motor
    motorGearboxEncoder.motor.setSegment(0 , M1, I1,   Y2,    0);
    motorGearboxEncoder.motor.setSegment(1 , M2, I2, Y3-1, Y2+1);
    
    Serial.println("Calibration complete.");
    
}

float  Axis::_speedSinceLastCall(){
    //static variables to persist between calls
    static long time = millis();
    static long prevEncoderValue = motorGearboxEncoder.encoder.read();
    
    //compute dist moved
    int elapsedTime = millis() - time;
    int distMoved   = motorGearboxEncoder.encoder.read() - prevEncoderValue;
    float speed = float(distMoved)/float(elapsedTime);
    
    //catch if time is zero
    if (elapsedTime < 10){
        speed = 0;
    }
    
    //set values for next call
    time = millis();
    prevEncoderValue = motorGearboxEncoder.encoder.read();
    
    //return the absolute value because speed is not a vector
    return abs(speed);
}

float  Axis::measureMotorSpeed(int speed){
    /*
    Returns the motors speed in RPM at a given input value
    */
    
    int sign = speed/abs(speed);
    
    _disableAxisForTesting = true;
    attach();
    
    int numberOfStepsToTest = 2000;
    int timeOutMS           = 30*1000; //30 seconds
    bool stall              = false;
    
    //run the motor for numberOfStepsToTest steps positive and record the time taken
    
    
    long originalEncoderPos  = motorGearboxEncoder.encoder.read();
    long startTime = millis();
    
    //until the motor has moved the target distance
    while (abs(originalEncoderPos - motorGearboxEncoder.encoder.read()) < numberOfStepsToTest){
        //establish baseline for speed measurement
        _speedSinceLastCall();
        
        //command motor to spin at speed
        motorGearboxEncoder.motor.write(speed);
        
        //wait
        delay(200);
        
        //print to prevent connection timeout
        Serial.println("<Idle,MPos:0,0,0>");
        
        //check to see if motor is moving
        if (_speedSinceLastCall() < .01){
            stall = true;
            break;
        }
    }
    int posTime = millis() - startTime;
    
    //rotations = number of steps taken / steps per rotation
    float rotations = (originalEncoderPos - motorGearboxEncoder.encoder.read())/_encoderSteps;
    //minutes = time elapsed in ms * 1000ms/s *60 seconds per minute
    float minutes   = posTime/(1000.0*60.0);
    
    //RPM is rotations per minute.
    float RPM = rotations/minutes;
    
    if (stall){
        RPM = 0;
    }
    
    //move back to start point
    _disableAxisForTesting = false;
    _timeLastMoved = millis();
    for (long startTime = millis(); millis() - startTime < 2000; millis()){
        hold();
        delay(50);
        //print to prevent connection timeout
        Serial.println("<Idle,MPos:0,0,0>");
    }
    
    return RPM;
}
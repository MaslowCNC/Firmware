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

#include "Maslow.h"

Motor::Motor(){

  _attachedState = 0;


}

int  Motor::setupMotor(const int& pwmPin, const int& pin1, const int& pin2){

  //store pin numbers as private variables
  _pwmPin = pwmPin;
  _pin1  = pin1;
  _pin2  = pin2;
  _attachedState = 0;

  if (TLE5206 == true) {
  //set pinmodes
    pinMode(_pwmPin,   INPUT);   // TLE5206 'Error Flag' pin
    pinMode(_pin1,     OUTPUT);
    pinMode(_pin2,     OUTPUT);
    
 //stop the motor
    digitalWrite(_pin1,    LOW);
    digitalWrite(_pin2,    LOW) ;
    
  } else if (TLE9201 == true) {
  //set pinmodes
    pinMode(_pwmPin,   OUTPUT);
    pinMode(_pin1,     OUTPUT);   // TLE9201 DIR pin
    pinMode(_pin2,     OUTPUT);   // TLE9201 ENABLE pin
    
  //stop the motor
    digitalWrite(_pin2,    HIGH); // TLE9201 ENABLE pin, LOW = on
    
  } else {
  //set pinmodes
    pinMode(_pwmPin,   OUTPUT);
    pinMode(_pin1,     OUTPUT);
    pinMode(_pin2,     OUTPUT);

    //stop the motor
    digitalWrite(_pin1,    HIGH);
    digitalWrite(_pin2,    LOW) ;
    digitalWrite(_pwmPin,  LOW);
  }
  return 1;
}

void Motor::attach(){
    _attachedState = 1;
}

void Motor::detach(){
    if (_attachedState != 0) {
        if (TLE5206 == true) {
          //stop the motor
          digitalWrite(_pin1,    LOW);
          digitalWrite(_pin2,    LOW) ;
        } else if (TLE9201 == true) {
        //stop the motor
          digitalWrite(_pin2,    HIGH); // TLE9201 ENABLE pin, LOW = on
          analogWrite(_pwmPin,   0);
        } else {
          //stop the motor
          digitalWrite(_pin1,    HIGH);
          digitalWrite(_pin2,    LOW) ;
          digitalWrite(_pwmPin,  LOW);
        }
    }
    _attachedState = 0;
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

void Motor::write(int speed, bool force){
    /*
    Sets motor speed from input. Speed = 0 is stopped, -255 is full reverse, 255 is full ahead. If force is true the motor attached state will be ignored
    */
    if ((_attachedState == 1 or force) and (FAKE_SERVO_STATE != FAKE_SERVO_PERMITTED)){
        speed = constrain(speed, -255, 255);
        _lastSpeed = speed; //saves speed for use in additive write
        bool forward = (speed > 0);
        speed = abs(speed); //remove sign from input because direction is set by control pins on H-bridge

        bool usePin1 = ((_pin1 != 4) && (_pin1 != 13) && (_pin1 != 11) && (_pin1 != 12)); // avoid PWM using timer0 or timer1
        bool usePin2 = ((_pin2 != 4) && (_pin2 != 13) && (_pin2 != 11) && (_pin2 != 12)); // avoid PWM using timer0 or timer1
        bool usepwmPin = ((TLE5206 == false) && (_pwmPin != 4) && (_pwmPin != 13) && (_pwmPin != 11) && (_pwmPin != 12)); // avoid PWM using timer0 or timer1       
        if (!(TLE5206 || TLE9201)) { // L298 boards
            if (forward){
                if (usepwmPin){
                    digitalWrite(_pin1 , HIGH );
                    digitalWrite(_pin2 ,  LOW  );
                    analogWrite(_pwmPin, speed);
                }
                else if (usePin2) {
                    digitalWrite(_pin1 , HIGH );
                    analogWrite(_pin2 , 255 - speed); // invert drive signals - don't alter speed
                    digitalWrite(_pwmPin, HIGH);
                }
                else{
                    analogWrite(_pin1 , speed);
                    digitalWrite(_pin2 , LOW );
                    digitalWrite(_pwmPin, HIGH);
                }
            }
            else { // reverse or zero speed
                if (usepwmPin){
                    digitalWrite(_pin2 , HIGH);
                    digitalWrite(_pin1 , LOW );
                    analogWrite(_pwmPin, speed);
                }
                else if (usePin1) {
                    analogWrite(_pin1 , 255 - speed); // invert drive signals - don't alter speed
                    digitalWrite(_pin2 , HIGH );
                    digitalWrite(_pwmPin, HIGH);
                }
                else {
                    analogWrite(_pin2 , speed);
                    digitalWrite(_pin1 , LOW );
                    digitalWrite(_pwmPin, HIGH);
                }
            }
        } 
        else if (TLE5206)  {
            speed = constrain(speed, 0, 254); // avoid issue when PWM value is 255
            if (forward) {
                if (speed > 0) {
                    if (usePin2) {
                        digitalWrite(_pin1 , HIGH );
                        analogWrite(_pin2 , 255 - speed); // invert drive signals - don't alter speed
                    } 
                    else {
                        analogWrite(_pin1 , speed);
                        digitalWrite(_pin2 , LOW );
                    }
                } 
                else { // speed = 0 so put on the brakes
                    digitalWrite(_pin1 , LOW );
                    digitalWrite(_pin2 , LOW );
                }
            } 
            else { // reverse
                if (usePin1) {
                    analogWrite(_pin1 , 255 - speed); // invert drive signals - don't alter speed
                    digitalWrite(_pin2 , HIGH );
                } else {
                    analogWrite(_pin2 , speed);
                    digitalWrite(_pin1 , LOW );
                }
            }
        }
        else if (TLE9201) {
            int dirPin     = _pin1;
            int enablePin  = _pin2;
            const int TOP  = 1;
            /*
            * The TLE9201 uses dedicated DIR, PWM and DISable pins,
            * so logic from previous chips won't work. In addition, 
            * the left and right motors are affected by chainOverSprocket 
            * but the Z motor is not.
            */

            uint8_t dirCMD;
            uint8_t extend = HIGH;
            uint8_t retract = LOW;
            if (_pwmPin == ENB) { // Z motor unaffected by chainOverSprocket
                if (forward) {
                    dirCMD = retract;
                } 
                else {
                    dirCMD = extend;
                } 
            } else { // L and R motor affected by chainOverSprocket
                if(sysSettings.chainOverSprocket == TOP) {
                    if (forward) {
                        dirCMD = extend;
                    } 
                    else {
                        dirCMD = retract;
                    } 
                } else {
                    if (forward) {
                        dirCMD = retract;
                    } 
                    else {
                        dirCMD = extend;
                    }
                }
            }

            digitalWrite(dirPin, dirCMD); // TLE9201 DIR pin
            analogWrite (_pwmPin, speed); // TLE9201 PWM pin
            digitalWrite(enablePin, LOW); // TLE9201 ENABLE pin, HIGH = disable
        }
    else {} // add new boards here
    }
}

void Motor::directWrite(int voltage){
    /*
    Write directly to the motor, ignoring if the axis is attached or any applied calibration.
    */
    write(voltage, true);
}

int  Motor::attached(){
    
    return _attachedState;
}

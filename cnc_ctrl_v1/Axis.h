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
    
    #ifndef Axis_h
    #define Axis_h

    #include "Arduino.h"
    #include "PID_v1.h"
    #include <EEPROM.h>
    #include "MotorGearboxEncoder.h"
    

    class Axis{
        public:
            Axis(int pwmPin, int directionPin1, int directionPin2, int encoderPin1, int encoderPin2, String axisName, int eepromAdr, float mmPerRotation, float encoderSteps);
            int    write(float targetPosition);
            float  read();
            int    set(float newAxisPosition);
            int    updatePositionFromEncoder();
            void   initializePID();
            int    detach();
            int    attach();
            void   hold();
            void   endMove(float finalTarget);
            float  target();
            float  error();
            float  setpoint();
            void   computePID();
            float  measureMotorSpeed(int speed);
            void   computeMotorResponse();
            void   test();
            void   changePitch(float newPitch);
            void   changeEncoderResolution(int newResolution);
            bool   attached();
            void   wipeEEPROM();
            MotorGearboxEncoder    motorGearboxEncoder;
            
        private:
            int        _PWMread(int pin);
            void       _writeFloat(unsigned int addr, float x);
            float      _readFloat(unsigned int addr);
            int        _sign(float val);
            int        _change(float val);
            void       _writeLinSeg(unsigned int addr, LinSegment linSeg);
            void       _writeAllLinSegs(unsigned int addr);
            LinSegment _readLinSeg(unsigned int addr);
            void       _readAllLinSegs(unsigned int addr);
            int        _direction;
            int        _encoderPin;
            String     _axisName;
            float      _axisTarget;
            int        _currentAngle;
            int        _previousAngle;
            double     _timeLastMoved;
            double     _pidSetpoint, _pidInput, _pidOutput;
            double     _Kp=4700, _KiClose=250, _KiMid = 50, _KiFar = 0, _Kd=170;
            PID        _pidController;
            int        _eepromAdr;
            float      _mmPerRotation;
            float      _encoderSteps;
            float      _oldSetpoint;
            float      _oldVal;
            bool       _disableAxisForTesting = false;
            float      _speedSinceLastCall();
    };

    #endif

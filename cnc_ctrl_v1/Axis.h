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
            void   setPIDAggressiveness(float aggressiveness);
            void   test();
            void   changePitch(float newPitch);
            void   changeEncoderResolution(int newResolution);
            bool   attached();
            void   wipeEEPROM();
            MotorGearboxEncoder    motorGearboxEncoder;
            String     _axisName;
            
        private:
            int        _PWMread(int pin);
            void       _writeFloat(unsigned int addr, float x);
            float      _readFloat(unsigned int addr);
            int        _detectDirectionChange(float _pidSetpoint);
            int        _direction;
            int        _encoderPin;
            float      _axisTarget;
            int        _currentAngle;
            int        _previousAngle;
            double     _timeLastMoved;
            double     _pidSetpoint, _pidInput, _pidOutput;
            double     _Kp=600, _Ki = 10, _Kd=10;
            PID        _pidController;
            int        _eepromAdr;
            float      _mmPerRotation;
            float      _encoderSteps;
            float      _oldSetpoint;
            float      _oldDir;
            bool       _disableAxisForTesting = false;
    };

    #endif

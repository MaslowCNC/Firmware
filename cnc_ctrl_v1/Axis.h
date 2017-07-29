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
            Axis(const int& pwmPin, const int& directionPin1, const int& directionPin2, const int& encoderPin1, const int& encoderPin2, const String& axisName, const int& eepromAdr);
            void   write(const float& targetPosition);
            float  read();
            int    set(const float& newAxisPosition);
            int    updatePositionFromEncoder();
            void   initializePID();
            int    detach();
            int    attach();
            void   hold();
            void   endMove(const float& finalTarget);
            float  target();
            float  error();
            float  setpoint();
            void   computePID();
            void   setPIDAggressiveness(float aggressiveness);
            void   test();
            void   changePitch(const float& newPitch);
            void   changeEncoderResolution(const int& newResolution);
            bool   attached();
            void   wipeEEPROM();
            MotorGearboxEncoder    motorGearboxEncoder;
            void   setPIDValues(float Kp, float Ki, float Kd, float KpV, float KiV, float KdV);
            void   loadPositionFromMemory();
            
        private:
            int        _PWMread(int pin);
            void       _writeFloat(const unsigned int& addr, const float& x);
            float      _readFloat(const unsigned int& addr);
            int        _detectDirectionChange(const float& _pidSetpoint);
            int        _direction;
            int        _encoderPin;
            float      _axisTarget;
            int        _currentAngle;
            int        _previousAngle;
            double     _timeLastMoved;
            double     _pidSetpoint, _pidInput, _pidOutput;
            double     _Kp=0, _Ki = 0, _Kd=0;
            PID        _pidController;
            int        _eepromAdr;
            float      _mmPerRotation = 1;
            float      _encoderSteps  = 100;
            float      _oldSetpoint;
            float      _oldDir;
            bool       _disableAxisForTesting = false;
            String     _axisName;
    };

    #endif

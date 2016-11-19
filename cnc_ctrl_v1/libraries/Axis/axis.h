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
    
    #ifndef Axis_h
    #define Axis_h

    #include "Arduino.h"
    #include "../GearMotor/GearMotor.h"
    #include "../PID/PID_v1.h"
    #include <EEPROM.h>
    #include "../Encoder/Encoder.h"
    

    class Axis{
        public:
            Axis(int pwmPin, int directionPin1, int directionPin2, int encoderDirection, int encoderPin1, int encoderPin2, String axisName, int eepromAdr, float mmPerRotation);
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
            void   computeBoost();
            
        private:
            int        _PWMread(int pin);
            void       _writeFloat(unsigned int addr, float x);
            float      _readFloat(unsigned int addr);
            int        _sign(float val);
            int        _change(float val);
            
            GearMotor  _motor;
            int        _direction;
            int        _encoderPin;
            String     _axisName;
            float      _axisTarget;
            int        _currentAngle;
            int        _previousAngle;
            double     _timeLastMoved;
            double     _pidSetpoint, _pidInput, _pidOutput;
            double     _Kp=1300, _KiClose=250, _KiMid = 50, _KiFar = 0, _Kd=110;
            PID        _pidController;
            int        _eepromAdr;
            float      _mmPerRotation;
            Encoder    _encoder;
            float      _oldSetpoint;
            float      _oldVal;
            bool       _disableAxisForTesting = false;
            int        _posBoost = 4;
            int        _negBoost = 20;
    };

    #endif
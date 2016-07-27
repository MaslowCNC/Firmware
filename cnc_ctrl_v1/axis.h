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
    #include "GearMotor.h"
    #include "PID_v1.h"

    class Axis{
        public:
            Axis(int pwmPin, int directionPin1, int directionPin2, int encoderDirection, int encoderPin, String axisName);
            int    write(float targetPosition);
            float  read();
            int    set(float newAxisPosition);
            int    updatePositionFromEncoder();
            int    returnPidMode();
            void   initializePID();
            int    detach();
            int    attach();
            void   hold();
            void   endMove(float finalTarget);
        private:
            int    _PWMread(int pin);
            
            GearMotor  _motor;
            int        _direction;
            int        _encoderPin;
            String     _axisName;
            float      _axisPosition;
            float      _axisTarget;
            int        _currentAngle;
            int        _previousAngle;
            double     _timeLastMoved;
            double     _pidSetpoint, _pidInput, _pidOutput;
            double     _Kp=300, _Ki=0, _Kd=10;
            PID        _pidController;
    };

    #endif
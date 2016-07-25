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

    class Axis{
        public:
            Axis(int pwmPin, int directionPin1, int directionPin2, int encoderDirection, int encoderPin, String axisName);
            int moveTo(float targetPosition);
        private:
            GearMotor  _motor;
            int        _direction;
            int        _encoderPin;
            String     _axisName;
    };

    #endif
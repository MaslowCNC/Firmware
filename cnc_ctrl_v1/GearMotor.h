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
    
    #ifndef GearMotor_h
    #define GearMotor_h

    #include "Arduino.h"

    class GearMotor{
        public:
            GearMotor();
            void attach(int pin);
            int setupMotor(int pwmPin, int pin1, int pin2);
            void detach();
            void write(int speed);
            int attached();
        private:
            int _pwmPin;
            int _pin1;
            int _pin2;
            bool _attachedState;
            int _deadBand = 28;
    };

    #endif
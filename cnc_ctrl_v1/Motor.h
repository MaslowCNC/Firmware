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
    
    struct LinSegment{
        float slope  = 1;
        float intercept = 0;
        //The bounds are strict, so if the bounds are 0,1 .9 would work
        //but 1 and 0 will not
        int positiveBound = 0;
        int negativeBound = 0;
    };
    
    
    
    class GearMotor{
        public:
            GearMotor();
            void attach();
            int  setupMotor(int pwmPin, int pin1, int pin2);
            void detach();
            void write(int speed);
            int  attached();
            int  _convolve(int input);
            void setSegment(int index, float slope, float intercept, int negativeBound, int positiveBound);
            LinSegment getSegment(int index);
        private:
            int _pwmPin;
            int _pin1;
            int _pin2;
            bool _attachedState;
            LinSegment _linSegments[4];
            
    };

    #endif
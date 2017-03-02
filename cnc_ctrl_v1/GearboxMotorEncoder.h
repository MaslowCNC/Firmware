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
    
    #ifndef GearboxMotorEncoder_h
    #define GearboxMotorEncoder_h

    #include "Arduino.h"
    #include "Motor.h"
    #include "PID_v1.h"
    #include <EEPROM.h>
    #include "Encoder.h"
    
    #define EEPROMVALIDDATA 56
    #define SIZEOFFLOAT      4
    #define SIZEOFLINSEG    17
    
    
    #define NUMBER_OF_ENCODER_STEPS 8148.0 
    

    class GearboxMotorEncoder{
        public:
            GearboxMotorEncoder(int pwmPin, int directionPin1, int directionPin2, int encoderPin1, int encoderPin2, int eepromAdr);
            long  readEncoder();
            void  writeEncoder(long newEncoderValue);
            void  write(int speed);
            void  attach();
            void  detach();
            float measureMotorSpeed(int speed);
            void  computeMotorResponse();
            void  testPID(int speed);
        private:
            Encoder    _encoder;
            Motor      _motor;
            int        _eepromAdr;
            void       _writeLinSeg(unsigned int addr, LinSegment linSeg);
            void       _writeAllLinSegs(unsigned int addr);
            LinSegment _readLinSeg(unsigned int addr);
            void       _readAllLinSegs(unsigned int addr);
            float      _speedSinceLastCall();
            void       _writeFloat(unsigned int addr, float x);
            float      _readFloat(unsigned int addr);
            bool       _disableAxisForTesting = false;
            
    };

    #endif
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
    
    Copyright 2014-2016 Bar Smith*/
    
    #ifndef Kinematics_h
    #define Kinematics_h

    #include "Arduino.h"
    #include "BigNumber.h"
    #include "FormatDouble.h"

    class Kinematics{
        public:
            Kinematics();
            void  forward   (float chainALength, float chainBLength, float* X, float* Y);
            void  inverse   (float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            void  newInverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            float moment(float x, float y, float Theta, float Phi);
            void  test();
            void  speedTest(float input);
        private:
            BigNumber float2BigNum (float value);
    };

    #endif
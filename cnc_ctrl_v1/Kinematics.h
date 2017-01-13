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
    
    #define ORIGINCHAINLEN   1650
    
    #include "Arduino.h"
    #include "BigNumber.h"
    #include "FormatDouble.h"

    class Kinematics{
        public:
            Kinematics();
            void  forward   (float chainALength, float chainBLength, float* X, float* Y);
            void  inverse   (float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            void  oldInverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            void  test();
            void  speedTest(float input);
        private:
            BigNumber _float2BigNum (float value);
            float _moment(float Y1Plus, float Y2Plus, float Phi, float MSinPhi, float MSinPsi1, float MCosPsi1, float MSinPsi2, float MCosPsi2);
            float _YOffsetEqn(float YPlus, float Denominator, float Psi);
            void  _MatSolv();
            void  _MyTrig();
    };

    #endif
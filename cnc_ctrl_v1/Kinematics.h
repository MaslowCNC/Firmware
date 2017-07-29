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
    
    #ifndef Kinematics_h
    #define Kinematics_h
    
    #define ORIGINCHAINLEN   1650
    
    #include "Arduino.h"
    #include "FormatDouble.h"

    class Kinematics{
        public:
            Kinematics();
            void  inverse   (float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            void  quadrilateralInverse   (float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            void  triangularInverse   (float xTarget,float yTarget, float* aChainLength, float* bChainLength);
            void  recomputeGeometry();
            void  forward(const float& chainALength, const float& chainBLength, float* xPos, float* yPos);
            //geometry
            float l             = 310.0;                               //horizontal distance between sled attach points
            float s             = 139.0;                               //vertical distance between sled attach points and bit
            float h             = sqrt((l/2)*(l/2) + s * s);           //distance between sled attach point and bit
            float h3            = 79.0;                                //distance from bit to sled center of mass
            float D             = 2978.4;                              //distance between sprocket centers
            float R             = 10.2;                                //sprocket radius
            float rotationDiskRadius = 0;                              //distance from the bit to the attachment point
            int kinematicsType  = 1;                                   //Which version of the kinematics to use. 1 is quadrilateral, 2 is triangular


            //machine dimensions
            float machineHeight = 1219.2;                              //this is 4 feet in mm
            float machineWidth  = 2438.4;                              //this is 8 feet in mm
            float motorOffsetY  = 463.0;                               //vertical distance from the corner of the work area to the sprocket center
            float halfWidth = machineWidth / 2.0;                      //Half the machine width
            float halfHeight = machineHeight / 2.0;                    //Half the machine height
            
        private:
            float _moment(const float& Y1Plus, const float& Y2Plus, const float& Phi, const float& MSinPhi, const float& MSinPsi1, const float& MCosPsi1, const float& MSinPsi2, const float& MCosPsi2);
            float _YOffsetEqn(const float& YPlus, const float& Denominator, const float& Psi);
            void  _MatSolv();
            void  _MyTrig();
            void _verifyValidTarget(float* xTarget,float* yTarget);
            //target router bit coordinates.
            float x = 0;
            float y = 0;
            float _xCordOfMotor = D/2;
            float _yCordOfMotor = halfHeight + motorOffsetY;




            //utility variables
            float DegPerRad = 360/(4 * atan(1));
            unsigned long Time;
            boolean Mirror;

            //Calculation tolerances
            float MaxError = 0.001;
            byte MaxTries = 10;
            float DeltaPhi = 0.001;
            float DeltaY = 0.01;

            //Criterion Computation Variables
            float Phi = -0.2;
            float TanGamma; 
            float TanLambda;
            float Y1Plus ;
            float Y2Plus;
            float Theta = atan(2*s/l);
            float Psi1 = Theta - Phi;
            float Psi2 = Theta + Phi;
            byte Tries = 0;
            float Jac[9];
            float Solution[3];
            float Crit[3];
            float Offsetx1;
            float Offsetx2;
            float Offsety1;
            float Offsety2;
            float SinPsi1;
            float CosPsi1;
            float SinPsi2;
            float CosPsi2;
            float SinPsi1D;
            float CosPsi1D;
            float SinPsi2D;
            float CosPsi2D;
            float MySinPhi;
            float MySinPhiDelta;

            //intermediate output
            float Lambda;
            float Gamma;

            // output = chain lengths measured from 12 o'clock
            float Chain1; //left chain length 
            float Chain2; //right chain length

            int i;
    };

    #endif

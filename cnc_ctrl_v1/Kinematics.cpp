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

/*
The Kinematics module relates the lengths of the chains to the position of the cutting head
in X-Y space.
*/

#include "Arduino.h"
#include "Kinematics.h"

#define MACHINEHEIGHT    1219.2 //this is the distance from the motors to the center of the work space
#define MACHINEWIDTH     2438.4 //this is the distance between the motors
#define MOTOROFFSETX     270
#define MOTOROFFSETY     463
#define ORIGINCHAINLEN   sqrt(sq(MOTOROFFSETY + MACHINEHEIGHT/2.0)+ sq(MOTOROFFSETX + MACHINEWIDTH/2.0))
#define SLEDWIDTH        310
#define SLEDHEIGHT       139

Kinematics::Kinematics(){
  
  
}

void  Kinematics::forward(float chainALength, float chainBLength, float* X, float* Y){
    float chainLengthAtCenterInMM       = ORIGINCHAINLEN;
    
    //Use the law of cosines to find the angle between the two chains
    float   a   = chainBLength + chainLengthAtCenterInMM;
    float   b   = -1*chainALength + chainLengthAtCenterInMM;
    float   c   = MACHINEWIDTH+2*MOTOROFFSETX;
    float theta = acos( ( sq(b) + sq(c) - sq(a) ) / (2.0*b*c) );
    
    *Y   = MOTOROFFSETY + MACHINEHEIGHT/2 - (b*sin(theta));
    *X   = (b*cos(theta)) - (MACHINEWIDTH/2.0 + MOTOROFFSETX);
}

void  Kinematics::newForward(float chainALength, float chainBLength, float* X, float* Y){
    
    
    
    *Y   = 1;
    *X   = 1;
}

void  Kinematics::newInverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength){
    
    float chainLengthAtCenterInMM       = ORIGINCHAINLEN;
    
    float Bx = MACHINEWIDTH/2.0 + MOTOROFFSETX;
    float By = MACHINEHEIGHT/2.0 + MOTOROFFSETY;
    float Px = xTarget + SLEDWIDTH/2;
    float Py = yTarget + SLEDHEIGHT;
    
    float triangleTipY = ((Py-By)/(Px-Bx))*(xTarget-Bx)+By;
    float triangleTipX = xTarget;
    
    Serial.println("Tip: ");
    Serial.println(triangleTipX);
    Serial.println(triangleTipY);
    
    float X1 = MOTOROFFSETX + MACHINEWIDTH/2.0   + triangleTipX;
    float X2 = MOTOROFFSETX + MACHINEWIDTH/2.0   - triangleTipX;
    float Y  = MOTOROFFSETY + MACHINEHEIGHT/2.0  - triangleTipY;
    
    float La = sqrt( sq(X1) + sq(Y) );
    float Lb = sqrt( sq(X2) + sq(Y) );
    
    *aChainLength = -1*(La - chainLengthAtCenterInMM);
    *bChainLength = Lb - chainLengthAtCenterInMM;
}

void  Kinematics::inverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength){
    
    float chainLengthAtCenterInMM       = ORIGINCHAINLEN;
    
    float X1 = MOTOROFFSETX + MACHINEWIDTH/2.0   + xTarget;
    float X2 = MOTOROFFSETX + MACHINEWIDTH/2.0   - xTarget;
    float Y  = MOTOROFFSETY + MACHINEHEIGHT/2.0  - yTarget;
    
    float La = sqrt( sq(X1) + sq(Y) );
    float Lb = sqrt( sq(X2) + sq(Y) );
    
    *aChainLength = -1*(La - chainLengthAtCenterInMM);
    *bChainLength = Lb - chainLengthAtCenterInMM;
}

void Kinematics::test(){
    /*
    Known good values for testing (from simulation)
    
    Pos: Middle center
    La:1672.4
    Lb:1672.4
    X:1219.2
    Y:605.97
    Offset: -50.6
    
    Pos: Middle Top
    La:1507.45
    Lb:1507.45
    X:1219.2
    Y:1042.7
    Offset:-100
    
    Pos: Middle Bottom
    La:2052.77
    Lb:2052.77
    X:1219.21
    Y:29.1
    Offset: 14.46
    
    
    */
    Serial.println("test kinematics begin-------------------------------------------------------");
    
    float chainA = 100;//ORIGINCHAINLEN - 1672.4;
    float chainB = 100; //ORIGINCHAINLEN - 1672.4;
    float X = 0;
    float Y = 0; 
    
    Serial.print("La: ");
    Serial.println(chainA);
    Serial.print("Lb: ");
    Serial.println(chainB);
    
    newForward(chainA, chainB, &X, &Y);
    
    Serial.print("X: ");
    Serial.println(X);
    Serial.print("Y: ");
    Serial.println(Y);
    
    newInverse(X,Y, &chainA, &chainB);
    
    Serial.print("La: ");
    Serial.println(chainA);
    Serial.print("Lb: ");
    Serial.println(chainB);
}


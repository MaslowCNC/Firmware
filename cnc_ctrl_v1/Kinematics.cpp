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
#define ORIGINCHAINLEN   1650   //sqrt(sq(MOTOROFFSETY + MACHINEHEIGHT/2.0)+ sq(MOTOROFFSETX + MACHINEWIDTH/2.0))
#define SLEDWIDTH        310
#define SLEDHEIGHT       139

#define OLDKINEMATICS

#define AX               -1*MACHINEWIDTH/2 - MOTOROFFSETX
#define AY               MACHINEHEIGHT/2 + MOTOROFFSETY
#define BX               MACHINEWIDTH/2 + MOTOROFFSETX
#define BY               MACHINEHEIGHT/2 + MOTOROFFSETY

Kinematics::Kinematics(){
   
    BigNumber::begin ();
    
}

#ifdef ZAXIS OLDKINEMATICS //This lets you use the old kinematics which make a triangle where the tool is at the tip.

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


#else //Use the regular kinematics

void  Kinematics::forward(float Lac, float Lbd, float* X, float* Y){
    //Compute xy postion from chain lengths
    
    BigNumber::setScale (3);
    BigNumber neg1 = ("-1");
    
    //store variables in BigNumber form
    BigNumber AYb  = float2BigNum(AY);
    BigNumber AXb  = neg1*float2BigNum(AX);
    BigNumber BXb  = float2BigNum(BX);
    

    float chainLengthAtCenterInMM = 1628.4037;
    BigNumber Lacb = float2BigNum(-1*Lac + chainLengthAtCenterInMM);
    BigNumber Lbdb = float2BigNum(Lbd + chainLengthAtCenterInMM);
    
    //Do pre-calculations
    BigNumber alpha        = Lacb.pow(2) - AYb.pow(2);
    BigNumber beta         = Lbdb.pow(2) - AYb.pow(2);
    BigNumber widthb       = float2BigNum(SLEDWIDTH);
    BigNumber gamma        = BXb - AXb - widthb;//widthb - AXb + BXb;
    BigNumber b64          = 64.0;
    BigNumber b16          = 16.0;
    BigNumber b8           = 8.0;
    BigNumber b2           = 2.0; 
    
    //Do calculations
    //Derivation can be found at robotics.stackexchange.com/questions/10607/forward-and-revers-kinematics-for-modified-hanging-plotter
    BigNumber partOne      = b8*gamma.pow(2)*AYb;
    BigNumber partTwo      = b64*gamma.pow(4)*AYb.pow(2);
    BigNumber partThree    = b16*gamma.pow(2);
    BigNumber partFour     = alpha.pow(2) - b2*alpha*beta - b2*alpha*gamma.pow(2) + beta.pow(2) - b2*beta*gamma.pow(2)+gamma.pow(4);
    BigNumber partFive     = b8*gamma.pow(2);
    
    BigNumber insideRoot   = partTwo - (partThree*partFour);
    
    BigNumber Cyb          = (partOne - insideRoot.sqrt())/partFive;
    BigNumber inside       = Lacb.pow(2) - AYb.pow(2) + b2*AYb*Cyb - Cyb.pow(2);
    BigNumber Cxb          = AXb + inside.sqrt();
    
    BigNumber scaleb = ("10000.0");
    float     scalef = 10000.0;
    
    float Cy = Cyb*scaleb;
    Cy       = Cy/scalef;
    
    float Cx = Cxb*scaleb;
    Cx       = Cx/scalef;
    
    float Fx = Cx + SLEDWIDTH/2;
    float Fy = Cy - SLEDHEIGHT;
    
    *X   = Fx;
    *Y   = Fy;
}

void  Kinematics::inverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength){
    //compute chain lengths from an XY position
    
    float Cx = xTarget - SLEDWIDTH/2;
    float Cy = yTarget + SLEDHEIGHT;
    float Dx = xTarget + SLEDWIDTH/2;
    float Dy = Cy;
    
    float Lac = sqrt(sq(AX-Cx) + sq(AY-Cy));
    float Lbd = sqrt(sq(BX-Dx) + sq(BY-Dy));
    
    
    float chainLengthAtCenterInMM = 1628.4037;
    *aChainLength = -1*(Lac - chainLengthAtCenterInMM);
    *bChainLength = Lbd - chainLengthAtCenterInMM;
}

#endif

void Kinematics::test(){
    
    Serial.println("test kinematics begin-------------------------------------------------------");
    
    
    float chainA;
    float chainB;
    float X = -500.0;
    float Y = 500.0; 
    
    Serial.print("X: ");
    Serial.println(X);
    Serial.print("Y: ");
    Serial.println(Y);
    
    inverse(X,Y, &chainA, &chainB);
    
    Serial.println("New: ");
    Serial.print("La: ");
    Serial.println(chainA);
    Serial.print("Lb: ");
    Serial.println(chainB);
    
    forward(chainA, chainB, &X, &Y);
    
    Serial.print("X: ");
    Serial.println(X);
    Serial.print("Y: ");
    Serial.println(Y);
    
}

BigNumber Kinematics::float2BigNum (float value){
    char buf [20];
    fmtDouble (value, 6, buf, sizeof buf);
    BigNumber bigVal = BigNumber (buf);
    
    return bigVal;
}
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


#define MACHINEHEIGHT    1219.2 //this is 4 feet in mm
#define MACHINEWIDTH     2438.4 //this is 8 feet in mm
#define MOTOROFFSETX     270
#define MOTOROFFSETY     463
#define ORIGINCHAINLEN   sqrt(sq(MOTOROFFSETY + MACHINEHEIGHT/2.0)+ sq(MOTOROFFSETX + MACHINEWIDTH/2.0))
#define SLEDWIDTH        310
#define SLEDHEIGHT       139

//Keith's variables
#define L                SLEDWIDTH
#define S                SLEDHEIGHT
#define H                sqrt(sq(L/2) + sq(S))
#define H3               
#define SPROCKETR        15
#define D                 MACHINEWIDTH + 2*MOTOROFFSETX //The width of the wood + 2*lenght of arms


#define AX               -1*MACHINEWIDTH/2 - MOTOROFFSETX
#define AY               MACHINEHEIGHT/2 + MOTOROFFSETY
#define BX               MACHINEWIDTH/2 + MOTOROFFSETX
#define BY               MACHINEHEIGHT/2 + MOTOROFFSETY

Kinematics::Kinematics(){
   
    BigNumber::begin ();
    
}

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

void  Kinematics::newInverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength){
    
    
    //initialization

    float NetMoment = 0.2;                                //primer so stop criteria isn't met before first pass
    float phideg = -0.5;                                  //initial estimate of carver-pen holder tilt in degrees 
    float Gamma = atan(yTarget/xTarget);                  //initial estimate of left chain angle 
    float Lambda = atan(yTarget/(D - xTarget));           //initial estimate of right chain angle
    float Phi = phideg/(360.0/DegPerRad);
    float Theta = atan(2.0*S/L);                          //angle between line connecting pen holder attach points and line from attach point to pen
    int   Tries = 0;

    //Solution

    tic                                                   //start the stop watch
    while abs(NetMoment) > MaxError 
        if Tries > MaxTries, break, end                   //estimate the tilt angle that results in zero net moment about the pen
        Tries = Tries + 1;                                //and refine the estimate until the error is acceptable or time runs out
                                                          //compute the moment given the angle phi
        NetMoment = MomentSproc(h, h3, SprocketR, x, y, D, Theta, Phi);    
        XTry(Tries) = Phi;
        YTry(Tries) = NetMoment;
        LTry(Tries) = tan(Lambda);
        GTry(Tries) = tan(Gamma);
        OldPhi = Phi;

                                                           //compute perturbed moment 
        Phi = Phi + DeltaPhi;
        NewNetPhi = MomentSproc(h, h3, SprocketR, x, y, D, Theta, Phi);
                                                  
        dPhi = (NewNetPhi - NetMoment)/DeltaPhi; 
        
                                                          //compute new estimate of Phi, for zero net moment  
        Phi = OldPhi - NetMoment/dPhi;
    end

    Phideg = Phi * DegPerRad                            

    Psi1 = Theta - Phi;
    Psi2 = Theta + Phi;

    Offsetx1 = h * cos(Psi1);
    Offsetx2 = h * cos(Psi2);
    Offsety1 = h * sin(Psi1);
    Offsety2 = h * sin(Psi2);
}

float Kinematics::momentSproc(float h, float h3, float x, float y, float D, float Theta, float Phi){
    //Serial.println("would run sprocket code here");
    
    float Psi1 = Theta - Phi;
    float Psi2 = Theta + Phi;
    float Offsetx1 = h * cos(Psi1); //these sin and cos operations are duplicated in the MomentSproc calculation
    float Offsetx2 = h * cos(Psi2);
    float Offsety1 = h * sin(Psi1);
    float Offsety2 = h * sin(Psi2);
    float TanGamma = (y - Offsety1)/(x - SPROCKETR - Offsetx1);
    float TanLambda = (y - Offsety2)/(D - SPROCKETR -(x + Offsetx2));

    float MomentSproc = h3*sin(Phi)+ (h/(TanLambda+TanGamma))*(sin(Psi2) - sin(Psi1) + TanGamma*cos(Psi1) - TanLambda * cos(Psi2));

    return MomentSproc;
}

void  Kinematics::speedTest(float input){
    Serial.println("Begin Speed Test");
    
    Serial.println(momentSproc(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
    
    float x = 0;
    float y = .3*1.0;
    long  startTime = micros();
    int iterations = 1000;
    
    for (int i = 0; i < iterations; i++){
        x = momentSproc(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, float(i)/100000.0);
    }
    
    long time = (micros() - startTime)/iterations;
    
    Serial.println(x);
    
    Serial.print("Time per call: ");
    Serial.print(time);
    Serial.println("us");
    
}

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
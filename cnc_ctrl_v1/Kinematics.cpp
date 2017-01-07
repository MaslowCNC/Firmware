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

//#define OLDKINEMATICS 

#define MACHINEHEIGHT    1219.2 //this is 4 feet in mm
#define MACHINEWIDTH     2438.4 //this is 8 feet in mm
#define MOTOROFFSETX     270.0
#define MOTOROFFSETY     463.0
#define SLEDWIDTH        310.0
#define SLEDHEIGHT       139.0

#define AX               -1*MACHINEWIDTH/2 - MOTOROFFSETX
#define AY               MACHINEHEIGHT/2 + MOTOROFFSETY
#define BX               MACHINEWIDTH/2 + MOTOROFFSETX
#define BY               MACHINEHEIGHT/2 + MOTOROFFSETY


//Keith's variables
//Coordinates definition:
//         x -->, y |
//                  v
// (0,0) at center of left sprocket
// upper left corner of plywood (270, 270)

//target router bit coordinates.
float x = 270;
float y = 1489.2;

//utility variables
float DegPerRad = 360/(4 * atan(1));
unsigned long Time;

//geometry
float l = 310.0;
float s = 139.0;
float h = sqrt((l/2)*(l/2) + s * s);
float h3 = 79.0;
float D = 2978.4;
float R = 10.2;

//Calculation tolerances
float MaxError = 0.01;
byte MaxTries = 10;
float DeltaPhi = 0.001;
float DeltaY = 0.01;

//Criterion Computation Variables
float Phi = 0.0;
float TanGamma = y/x;
float TanLambda = y/(D-x);
float Y1Plus = R * sqrt(1 + TanGamma * TanGamma);
float Y2Plus = R * sqrt(1 + TanLambda * TanLambda);
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

//intermediate output
float Lambda;
float Gamma;

// output = chain lengths measured from 12 o'clock

float Chain1; //left chain length 
float Chain2; //right chain length

Kinematics::Kinematics(){
   
    BigNumber::begin ();
    
}

#ifdef OLDKINEMATICS //This lets you use the old kinematics which make a triangle where the tool is at the tip.

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
    
    BigNumber Lacb = float2BigNum(Lac);
    BigNumber Lbdb = float2BigNum(Lbd);
    
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
    
    
    *aChainLength = Lac;
    *bChainLength = Lbd;
}

void  Kinematics::newInverse(float xTarget,float yTarget, float* aChainLength, float* bChainLength){
    
    Time = micros();                            //start the stop watch

    Tries = 0;                                  //initialize                   

    Psi1 = Theta - Phi;
    Psi2 = Theta + Phi;
                                             //These criteria will be zero when the correct values are reached 
                                             //They are negated here as a numerical efficiency expedient
    Crit[0]=  - moment(Y1Plus, Y2Plus, Phi);    
    Crit[1] = - YOffsetEqn(Y1Plus, x - h * cos(Psi1), Psi1);
    Crit[2] = - YOffsetEqn(Y2Plus, D - (x + h * cos(Psi2)), Psi2);

    while (Tries <= MaxTries) {
    if (abs(Crit[0]) < MaxError) {
      if (abs(Crit[1]) < MaxError) {
        if (abs(Crit[2]) < MaxError){
          break;
        }
      }
    }                   
                   //estimate the tilt angle that results in zero net moment about the pen
                   //and refine the estimate until the error is acceptable or time runs out

                          //Estimate the Jacobian components 
                                                       
    Jac[0] = (moment( Y1Plus, Y2Plus,Phi + DeltaPhi) + Crit[0])/DeltaPhi;
    Jac[1] = (moment( Y1Plus + DeltaY, Y2Plus, Phi) + Crit[0])/DeltaY;  
    Jac[2] = (moment(Y1Plus, Y2Plus + DeltaY,  Phi) + Crit[0])/DeltaY;
    Jac[3] = (YOffsetEqn(Y1Plus, x - h * cos(Psi1 - DeltaPhi), Psi1 - DeltaPhi) + Crit[1])/DeltaPhi;
    Jac[4] = (YOffsetEqn(Y1Plus + DeltaY, x - h * cos(Psi1),Psi1) + Crit[1])/DeltaY;
    Jac[5] = 0.0;
    Jac[6] = (YOffsetEqn(Y2Plus, D - (x + h * cos(Psi2+DeltaPhi)), Psi2 + DeltaPhi) + Crit[2])/DeltaPhi;
    Jac[7] = 0.0;
    Jac[8] = (YOffsetEqn(Y2Plus + DeltaY, D - (x + h * cos(Psi2)), Psi2) + Crit[2])/DeltaY;

    //solve for the next guess
    MatSolv();     // solves the matrix equation Jx=-Criterion                                                     
                   
    // update the variables with the new estimate

    Phi = Phi + Solution[0];
    Y1Plus = Y1Plus + Solution[1];                         //don't allow the anchor points to be inside a sprocket
    if (Y1Plus < R){
        Y1Plus = R;                               
    }
    Y2Plus = Y2Plus + Solution[2];                         //don't allow the anchor points to be inside a sprocket
    if (Y2Plus < R){
        Y2Plus = R;
    }

    Psi1 = Theta - Phi;
    Psi2 = Theta + Phi;   
                                                             //evaluate the
                                                             //three criterion equations

    Crit[0] = - moment(Y1Plus, Y2Plus, Phi);
    Crit[1] = - YOffsetEqn(Y1Plus, x - h * cos(Psi1), Psi1);
    Crit[2] = - YOffsetEqn(Y2Plus, D - (x + h * cos(Psi2)), Psi2);
    Tries = Tries + 1;                                       // increment iteration count
    }                                       

    //Variables are within accuracy limits
    //  perform output computation
    Offsetx1 = h * cos(Psi1);
    Offsetx2 = h * cos(Psi2);
    Offsety1 = h * sin(Psi1);
    Offsety2 = h * sin(Psi2);
    TanGamma = (y - Offsety1 + Y1Plus)/(x - Offsetx1);
    TanLambda = (y - Offsety2 + Y2Plus)/(D -(x + Offsetx2));
    Gamma = atan(TanGamma);
    Lambda =atan(TanLambda);
    Gamma = Gamma;
    Lambda = Lambda;

    //compute the chain lengths

    Chain1 = sqrt((x - Offsetx1)*(x - Offsetx1) + (y + Y1Plus - Offsety1)*(y + Y1Plus - Offsety1)) - R * TanGamma + R * Gamma;   //left chain length                       
    Chain2 = sqrt((D - (x + Offsetx2))*(D - (x + Offsetx2))+(y + Y2Plus - Offsety2)*(y + Y2Plus - Offsety2)) - R * TanLambda + R * Lambda;   //right chain length
    
    *aChainLength = Chain1;
    *bChainLength = Chain2;

}

void  Kinematics::MatSolv(){
  float Sum;
  int NN;
  int i;
  int ii;
  int J;
  int JJ;
  int K;
  int KK;
  int L;
  int M;
  int N;

  float fact;

  // gaus elimination, no pivot

  N = 3;
  NN = N-1;
  for (i=1;i<=NN;i++){
    J = (N+1-i);
    JJ = (J-1) * N-1;
    L = J-1;
    KK = -1;
    for (K=0;K<L;K++){
      fact = Jac[KK+J]/Jac[JJ+J];
       for (M=1;M<=J;M++){
        Jac[KK + M]= Jac[KK + M] -fact * Jac[JJ+M];
      }
      KK = KK + N;      
      Crit[K] = Crit[K] - fact * Crit[J-1];
   }
  }

//Lower triangular matrix solver

  Solution[0] =  Crit[0]/Jac[0];
  ii = N-1;
  for (i=2;i<=N;i++){
    M = i -1;
    Sum = Crit[i-1];
    for (J=1;J<=M;J++){
      Sum = Sum-Jac[ii+J]*Solution[J-1]; 
    }
    Solution[i-1] = Sum/Jac[ii+i];
    ii = ii + N;
  }
}

float Kinematics::moment(float Y1Plus,float Y2Plus, float Phi){   //computes net moment about center of mass
    float Temp;
    float Offsetx1;
    float Offsetx2;
    float Offsety1;
    float Offsety2;
    float Psi1;
    float Psi2;
    float TanGamma;
    float TanLambda;

    Psi1 = Theta - Phi;
    Psi2 = Theta + Phi;
    
    Offsetx1 = h * cos(Psi1);
    Offsetx2 = h * cos(Psi2);
    Offsety1 = h * sin(Psi1);
    Offsety2 = h * sin(Psi2);
    TanGamma = (y - Offsety1 + Y1Plus)/(x - Offsetx1);
    TanLambda = (y - Offsety2 + Y2Plus)/(D -(x + Offsetx2));
    
    return h3*sin(Phi) + (h/(TanLambda+TanGamma))*(sin(Psi2) - sin(Psi1) + (TanGamma*cos(Psi1) - TanLambda * cos(Psi2)));   
}

float Kinematics::YOffsetEqn(float YPlus, float Denominator, float Psi){
float Temp;
  Temp = ((sqrt(YPlus * YPlus - R * R)/R) - (y + YPlus - h * sin(Psi))/Denominator);
  return Temp;
}

void  Kinematics::speedTest(float input){
    Serial.println("Begin Speed Test");
    
    float x = 0;
    float y = .3*1.0;
    long  startTime = micros();
    int iterations = 1000;
    float chainA;
    float chainB;
    
    for (int i = 0; i < iterations; i++){
        newInverse(100, float(i)/100000.0, &chainA, &chainB);
    }
    
    long time = (micros() - startTime)/iterations;
    
    
    Serial.print("Time per call: ");
    Serial.print(time);
    Serial.println("us");
    
    
    
    
    startTime = micros();
    
    newInverse(1489.2,1489.2, &chainA, &chainB);
    
    time = (micros() - startTime);
    Serial.print("Time to converge: ");
    Serial.print(time);
    Serial.println("us");
    
    inverse(0, 0, &chainA, &chainB);
    
    Serial.println("Old K Chain Lengths at Center");
    Serial.println(chainA);
    Serial.println(chainB);
    
    inverse(100, 0, &chainA, &chainB);
    
    Serial.println("Old K Chain Lengths at x:+100");
    Serial.println(chainA);
    Serial.println(chainB);
    
    
    newInverse(0, 0, &chainA, &chainB);
    
    Serial.println("New K Chain Lengths at Center");
    Serial.println(chainA);
    Serial.println(chainB);
    
    newInverse(100, 0, &chainA, &chainB);
    
    Serial.println("New K Chain Lengths at x:+100");
    Serial.println(chainA);
    Serial.println(chainB);

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
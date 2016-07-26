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

    Copyright 2014 Bar Smith*/
    
    /*
    This module contains all of the functions necessary to move the machine. Internally the program operates in units of
    rotation. The conversion ratio between rotations and linear movement is stored by the XPITCH... #defines. When a line
    of Gcode arrives the units are converted into rotations, operated on, and the position is then returned by the machine
    using real world units.
    */

#include "MyTypes.h"

#define FORWARD 1
#define BACKWARD -1

//these define the number (or fraction there of) of mm moved with each rotation of each axis
#define XPITCH 1
#define YPITCH 1
#define ZPITCH 1

#define XDIRECTION BACKWARD
#define YDIRECTION FORWARD
#define ZDIRECTION BACKWARD

#define SENSEPIN 53

#define TOLERANCE .3//this sets how close to the target point the tool must be before it moves on.
#define MOVETOLERANCE .2 //this sets how close the machine must be to the target line at any given moment

#include "GearMotor.h"
#include "Axis.h"


int stepsize = 1;
float feedrate = 125;
float unitScalar = 200;
location_st location = {0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 500 , 500 , 500, 0, 0, 0};
Servo x;
Servo y;
Servo z;

Axis xAxis(7,8,9, FORWARD, 10, "X-axis");
Axis yAxis(6,12,13, FORWARD, 10, "Y-axis");

int servoDetachFlag = 1;
int movemode        = 1; //if move mode == 0 in relative mode,   == 1 in absolute mode

float getAngle(float X,float Y,float centerX,float centerY){

/*This function takes in the ABSOLUTE coordinates of the end of the circle and
 the ABSOLUTE coordinates of the center of the circle and returns the angle between 
 the end and the axis in pi-radians*/

    float theta = 0;
    if ( abs(X - centerX) < .1){
        centerX = X;
    }
    if ( abs(Y - centerY) < .1){
        centerY = Y;
    }

    if (X == centerX) { //this resolves div/0 errors
        if (Y >= centerY) {
            //Serial.println("special case one");
            return(0.5);
        }
        if ( Y <= centerY ){
            //Serial.println("special case two");
            return(1.5);
        }
    }
    if (Y == centerY) { //this resolves div/0 errors
        if ( X >= centerX) {
            //Serial.println("special case three");
            return(0);
        }
        if (X <= centerX) {
            //Serial.println("special case four");
            return(1.0);
        }
    }
    if (X > centerX and Y > centerY) { //quadrant one
        //Serial.println("Quadrant 1");
        theta = atan((centerY - Y)/(X - centerX));
        theta = 2 + theta/3.141593;
    }
    if (X < centerX and Y > centerY) { //#quadrant two
        //Serial.println("Quadrant 2");
        theta = atan((Y - centerY)/(X - centerX));
        theta = 1 - theta/3.141593;
    }
    if (X < centerX and Y < centerY) { //#quadrant three
        //Serial.println("Quadrant 3");
        theta = atan((centerY - Y)/(centerX - X));
        //Serial.println(theta);
        //theta = theta/3.141593 + 1;
        theta = 2 - theta/3.141593;
        theta = theta - 1;
        //Serial.println(theta);
    }
    if (X > centerX and Y < centerY) { //#quadrant four
        //Serial.println("Quadrant 4");
        theta = atan((centerY - Y)/(X - centerX));
        //Serial.println(theta);
        theta = theta/3.141593;
        //Serial.println(theta);
    }

    theta = 2 - theta;

    //Serial.println("Theta: ");
    //Serial.println(theta);
    return(theta);
}

void  returnPoz(){
    static unsigned long lastRan = millis();
    
    if (millis() - lastRan > 100){
        
        Serial.print("pz(");
        Serial.print(xAxis.read());
        Serial.print(", ");
        Serial.print(yAxis.read());
        Serial.println(", 0.0)");
        
        lastRan = millis();
    }
    
}

int   Move(float xEnd, float yEnd, float zEnd, float rotationsPerSecond){
    
/*The Move() function moves the tool in a straight line to the position (xEnd, yEnd, zEnd) at 
the speed moveSpeed. Movements are correlated so that regardless of the distances moved in each 
direction, the tool moves to the target in a straight line. This function is used by the G00 
and G01 commands. The units at this point should all be in rotations or rotations per second*/
    
    float  startingLocation           = location.xtarget;
    int    numberOfStepsPerRotation   = 1000;
    float  distanceToMoveInRotations  = xEnd - startingLocation;
    float  millisecondsForMove        = numberOfStepsPerRotation*(distanceToMoveInRotations/rotationsPerSecond);
    int    finalNumberOfSteps         = distanceToMoveInRotations*numberOfStepsPerRotation;
    float  timePerStep                = millisecondsForMove/float(finalNumberOfSteps);
    
    int numberOfStepsTaken   =  0;
    
    xAxis.attach();
    yAxis.attach();
    
    
    while(abs(numberOfStepsTaken) < abs(finalNumberOfSteps)){
        
        float whereItShouldBeAtThisStep = startingLocation + (numberOfStepsTaken/float(numberOfStepsPerRotation));
        
        delay(timePerStep);
        
        xAxis.updatePositionFromEncoder();
        yAxis.updatePositionFromEncoder();
        
        xAxis.write(whereItShouldBeAtThisStep);
        yAxis.write(whereItShouldBeAtThisStep);
        
        numberOfStepsTaken = numberOfStepsTaken + finalNumberOfSteps/abs(finalNumberOfSteps);
        
        returnPoz();
    }
    xAxis.detach();
    yAxis.detach();
    return(1);
    
}

float extractGcodeValue(String readString, char target,float defaultReturn){

/*Reads a string and returns the value of number following the target character.
If no number is found, defaultReturn is returned*/

    int begin;
    int end;
    String numberAsString;
    float numberAsFloat;
    
    begin           =  readString.indexOf(target);
    end             =  readString.indexOf(' ', begin);
    numberAsString  =  readString.substring(begin+1,end);
    numberAsFloat   =  numberAsString.toFloat();
    
    if (begin == -1){ //if the character was not found, return error
        return defaultReturn;
    }
    
    return numberAsFloat;
}

int   G1(String readString){
    
/*G1() is the function which is called to process the string if it begins with 
'G01' or 'G00'*/
    
    float xgoto = location.xtarget;
    float ygoto = location.ytarget;
    float zgoto = location.ztarget;
    float gospeed = 0;
    
    readString.toUpperCase(); //Make the string all uppercase to remove variability
    
    xgoto   = extractGcodeValue(readString, 'X', location.xtarget);
    ygoto   = extractGcodeValue(readString, 'Y', location.ytarget);
    zgoto   = extractGcodeValue(readString, 'Z', location.ztarget);
    gospeed = extractGcodeValue(readString, 'F', feedrate);
    
    
    //convert from mm to rotations
    xgoto = (XDIRECTION*xgoto) / XPITCH;
    ygoto = (XDIRECTION*ygoto) / YPITCH;
    zgoto = zgoto / ZPITCH;
    int secondsPerMinute = 60;
    feedrate = gospeed/(secondsPerMinute*XPITCH); //store the feed rate for later use
    
    int tempo = Move(xgoto, ygoto, zgoto, feedrate); //The move is performed

    if (tempo == 1){ //If the move finishes successfully 
        location.xtarget = xgoto;
        location.ytarget = ygoto;
        location.ztarget = zgoto;
    }
}

int   Circle(float radius, int direction, float xcenter, float ycenter, float startrad, float endrad, float speed){
    
/*Circle two takes in the radius of the circle to be cut and the starting and ending points in radians with 
pi removed so a complete circle is from 0 to 2. If direction is 1 the function cuts a CCW circle, and -1 cuts 
a CW circle. The direction that one moves from zero changes between the two directions, meaning that a quarter 
circle is always given by 0,.5 regardless of the direction. So direction = -1 start = 0 end = .5 makes a 1/4 
circle downward and direction = 1 start = 0 end = .5 makes a 1/4 circle upward starting from the right side of 
the circle*/

    int i = 0;
    int j = 0;
    int comp = 1;
    int endAngle = 0;
    float xdist, ydist;
    float origxloc = -1 * location.xtarget;
    float origyloc = location.ytarget;
    float origxAngleOffset, origyAngleOffset;
    long stime = millis();
    long ntime = millis();
    float deltaX = 5;
    float deltaY = 5;
    float deltaZ = 5;
    float tempXpos = location.xpos;
    float tempYpos = location.ypos;
    float tempZpos = location.zpos;
    float stepMultiplier = 55.0; //prevents the circle from looking like its made up of many small lines. 55 is just a made up number that seems about right.
    float timeStep = -1.45*speed + 200;

    /*Serial.println("Rads: ");
    Serial.println(startrad);
    Serial.println(endrad);*/

    //This addresses a weird issue where sometimes CAD packages use a circle with a HUGE radius to aproximate a straight line. The problem with this is that the arduino has
    //a hard time doing very precise floating point math, so when you use a very large radius it ends up being inaccurate. This should be solved in some better way.
    if(radius > 300 && abs(startrad - endrad) < 0.2){
        return(1);
    }

    endAngle = endrad*stepMultiplier*radius;
    i = startrad*stepMultiplier*radius;

    if(endrad < startrad){ //avoids weird behavior when not valid indices are sent
        endAngle = endAngle + (int)(2*stepMultiplier*radius);
    }

    /*Serial.println("IN CIRCLE: ");
    Serial.println(radius);
    Serial.println(direction);
    Serial.println(startrad,7);
    Serial.println(endrad,7);
    Serial.print("Start i: ");
    Serial.println(i);
    Serial.print("End i: ");
    Serial.println(endAngle);*/

    String stopString = "";
    while(i<endAngle){ //Actual movement takes place by incrementing the target position along the circle.
        if (Serial.available() > 0) {
            char c = Serial.read();  //gets one byte from serial buffer
            stopString += c; //makes the string readString
            //Serial.println(stopString);
            if(stopString == "STOP"){
                Serial.println("STOP");
                Serial.println("Clear Buffer");
                return(0);
            }
            if(stopString [0] != 'S'){
                stopString = "";
            }
        }

        location.xtarget = -1*radius * cos(3.141593*((float)i/(int)(stepMultiplier*radius))) - xcenter; //computes the new target position.
        location.ytarget = direction * radius * sin(3.141593*((float)i/(int)(stepMultiplier*radius))) + ycenter;

        //setpos(&location);
        //SetTarget(location.xtarget, location.ytarget, location.ztarget, &location);
        if( millis() - stime > timeStep ){
            if( abs(location.xpos - location.xtarget) < TOLERANCE && abs(location.ypos - location.ytarget) < TOLERANCE && abs(location.zpos - location.ztarget) < TOLERANCE){ //if the target is reached move to the next position
                i++;
                stime = millis();
            }
        }

        if( millis() - ntime > 300){
            deltaX = abs(tempXpos - location.xpos);
            deltaY = abs(tempYpos - location.ypos);
            deltaZ = abs(tempZpos - location.zpos);

            tempXpos = location.xpos;
            tempYpos = location.ypos;
            tempZpos = location.zpos;

            if(deltaX < .01 && abs(location.xpos - location.xtarget) > .1){
                //Serial.println("x stuck");
                if(location.xpos < location.xtarget){
                    //Unstick(x, -1);
                }
                else{
                    //Unstick(x, 1);
                }
            }
            if(deltaY < .01 && abs(location.ypos - location.ytarget) > .1){
                //Serial.println("y stuck");
                if(location.ypos < location.ytarget){
                    //Unstick(y, -1);
                }
                else{
                    //Unstick(y, 1);
                }
            }
            if(deltaZ < .01 && abs(location.zpos - location.ztarget) > .1){
                //Serial.println("z stuck");
                if(location.zpos < location.ztarget){
                    //Unstick(z, -1);
                }
                else{
                    //Unstick(z, 1);
                }
            }

            ntime = millis();
        }
    }
    //Serial.print("End i: ");
    //Serial.println(i);
    return(1);
}

int   G2(String readString){

    /*G2() is the function which is called when the string sent to the machine is 'G02' or 'G03'. 
    The string is parsed to extract the relevant information which is then used to compute the start and end 
    points of the circle and the the circle() function is called.*/
    
    int rpos;
    int mpos;
    int npos;
    int xpos;
    int ypos;
    int ipos;
    int jpos;
    int fpos;
    int rspace;
    int mspace;
    int nspace;
    int xspace;
    int yspace;
    int ispace;
    int jspace;
    int fspace;

    float radius = 0, mval = 0, nval = 0, xval = 0, yval = 0, ival = 0, jval = 0, fval = 0;
    char rsect[] = "                        ";
    char msect[] = "                        ";
    char nsect[] = "                        ";
    char xsect[] = "                        ";
    char ysect[] = "                        ";
    char isect[] = "                        ";
    char jsect[] = "                        ";
    char fsect[] = "                        ";

    rpos = readString.indexOf('R');
    mpos = readString.indexOf('M');
    npos = readString.indexOf('N');
    xpos = readString.indexOf('X');
    ypos = readString.indexOf('Y');
    ipos = readString.indexOf('I');
    jpos = readString.indexOf('J');
    fpos = readString.indexOf('F');

    rspace = readString.indexOf(' ', rpos);
    mspace = readString.indexOf(' ', mpos);
    nspace = readString.indexOf(' ', npos);
    xspace = readString.indexOf(' ', xpos);
    yspace = readString.indexOf(' ', ypos);
    ispace = readString.indexOf(' ', ipos);
    jspace = readString.indexOf(' ', jpos);
    fspace = readString.indexOf(' ', fpos);

    readString.substring((rpos + 1), rspace).toCharArray(rsect, 23);
    readString.substring((mpos + 1), mspace).toCharArray(msect, 23);
    readString.substring((npos + 1), nspace).toCharArray(nsect, 23);
    readString.substring((xpos + 1), xspace).toCharArray(xsect, 23);
    readString.substring((ypos + 1), yspace).toCharArray(ysect, 23);
    readString.substring((ipos + 1), ispace).toCharArray(isect, 23);
    readString.substring((jpos + 1), jspace).toCharArray(jsect, 23);
    readString.substring((fpos + 1), fspace).toCharArray(fsect, 23);

    xval = atof(xsect)*unitScalar;//The relevant information has been extracted
    yval = atof(ysect)*unitScalar;
    ival = atof(isect)*unitScalar;
    jval = atof(jsect)*unitScalar;
    fval = atof(fsect);

    if (xpos == -1){ //If x is not found in the provided string
        xval = -location.xtarget; //The xval is the current location of the machine
    }

    if (ypos == -1){ //If y is not found in the provided string
        yval = location.ytarget; //The yval is the current location of the machine
    }

    if(unitScalar > 15){ //running in inches
            fval = fval * 25.4; //convert to inches
    }

    if(fval > 4){ //preserves the feedrate for the next call
        feedrate = fval;
    }

    float ScaledXLoc = -1*location.xtarget;
    float ScaledYLoc = location.ytarget;
    float xCenter = ScaledXLoc + ival;
    float yCenter = ScaledYLoc + jval;

    if (xval != 0 || yval != 0 || ival != 0 || jval != 0){ //if some valid data is present
        radius =  sqrt(sq(ival) + sq(jval)); //computes the radius
        mval = getAngle(ScaledXLoc, ScaledYLoc, xCenter, yCenter); //computes the starting point on the circle
        nval = getAngle(xval, yval, xCenter, yCenter); //computes the ending point on the circle
    }


    int CircleReturnVal = 0;
    if(readString[2] == '2' || readString[1] == '2'){
        mval = 2 - mval; //flips the direction
        nval = 2 - nval;
        CircleReturnVal = Circle(radius, -1, xCenter, yCenter, mval, nval, feedrate);
    }
    else{
        CircleReturnVal = Circle(radius, 1, xCenter, yCenter, mval, nval, feedrate);
    }


    if(CircleReturnVal == 1){ //If the circle was cut correctly
        while( abs(location.xpos + xval) > TOLERANCE or abs(location.ypos - yval) > TOLERANCE){ //This ensures that the circle is completed and that if it is a circle with a VERY large radius and a small angle it isn't neglected
            //SetTarget(-1*xval, yval, location.ztarget, &location);
            //setpos(&location);
        }
        location.xtarget = -1*xval;
        location.ytarget = yval;
    }
    else{  //If something went wrong while cutting the circle
        location.xtarget = location.xpos;
        location.ytarget = location.ypos;
    }
}

void  G10(String readString){

/*The G10() function handles the G10 gcode which re-zeroes one or all of the machine's axes.*/


    if(readString.indexOf('X') > 2){
        location.xpos = 0.0;
        location.xtarget = 0.0;
    }
    if(readString.indexOf('Y') > 2){
        location.ypos = 0.0;
        location.ytarget = 0.0;
    }
    if(readString.indexOf('Z') > 2){
        location.zpos = 0.0;
        location.ztarget = 0.0;
    }
}

float readFloat(unsigned int addr){

//readFloat and writeFloat functions courtesy of http://www.alexenglish.info/2014/05/saving-floats-longs-ints-eeprom-arduino-using-unions/


    union{
        byte b[4];
        float f;
    } data;
    for(int i = 0; i < 4; i++)
    {
        data.b[i] = EEPROM.read(addr+i);
    }
    return data.f;
}

void  writeFloat(unsigned int addr, float x){
    union{
        byte b[4];
        float f;
    } data;
    data.f = x;
    for(int i = 0; i < 4; i++){
        EEPROM.write(addr+i, data.b[i]);
    }
}

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
    
    


#include <Servo.h>
#include "MyTypes.h"
#include <SPI.h>
#include <EEPROM.h>
#include "CNC_Functions.h"




int spindle = 17;
String readString;
String prependString;
long time = millis();
const int chipSelect = 40;   
int backLight = 140;
int contrast = 35;
float xgodist;
float ygodist;
float zgodist;
float gospeed;
int i = 0;
int begin;
int end;
char sect[22];

void setup(){
    Serial.begin(19200);
    
    Serial.println("ready");
    Serial.println("gready");
    analogReference(EXTERNAL);
    
    
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 50000;
    TCCR1B |= (1 << CS12);
    TIMSK1 |= (1 << TOIE1);
    interrupts(); 
    /*if (EEPROM.read(4) == 56){ //If the EEPROM has been written to by a previous calibration, this will be 56
        xMagnetScale = float(EEPROM.read(1))/100.0;
        yMagnetScale = float(EEPROM.read(2))/100.0;
        zMagnetScale = float(EEPROM.read(3))/100.0;
    }
    
    if (EEPROM.read(18) == 56){ //If valid data can be loaded
        Serial.println("Position Loaded");
        location.xpos = readFloat(5); //load the position from  the EEPROM
        location.ypos = readFloat(10);
        location.zpos = readFloat(14);
        location.xtarget = location.xpos;
        location.ytarget = location.ypos;
        location.ztarget = location.zpos;
    }*/
    
    
    xAxis.initializePID();
    
    G1("G01 X10 F1");
}

ISR(TIMER1_OVF_vect) //This code does not do anything right now, it is part of an ongoing effort to move the control system to be interupt driven
{
    TCNT1 = 64000;            // preload timer
    //SetPos(&location); 
    //SetTarget(location.xtarget, location.ytarget, location.ztarget, &location, 123);
    
}



void loop(){
    readString = "";
    if (Serial.available()){
        while (Serial.available()) {
            delay(1);  //delay to allow buffer to fill 
            if (Serial.available() > 0) {
                char c = Serial.read();  //gets one byte from serial buffer
                readString += c; //makes the string readString
            } 
        }
    }
    
    
    i = 0;
    while (i < 23){
        sect[i] = ' ';
        i++;
    }
    
    if(readString.substring(0, 3) == "G00" || readString.substring(0, 3) == "G01" || readString.substring(0, 3) == "G02" || readString.substring(0, 3) == "G03" || readString.substring(0, 2) == "G0" || readString.substring(0, 2) == "G1" || readString.substring(0, 2) == "G2" || readString.substring(0, 2) == "G3"){
        prependString = readString.substring(0, 3);
        prependString = prependString + " ";
    }
    
    if(readString[0] == 'X' || readString[0] == 'Y' || readString[0] == 'Z'){
        readString = prependString + readString;
        //Serial.print("prepended: ");
        //Serial.println(prependString);
    }
    
    if(readString.length() > 0){
        Serial.println(readString);
        time = millis();
        
        servoDetachFlag = 0;
    }
    
    if(readString.substring(0, 3) == "G01" || readString.substring(0, 3) == "G00" || readString.substring(0, 3) == "G0 " || readString.substring(0, 3) == "G1 "){
        G1(readString);
        Serial.println("ready");
        Serial.println("gready");
        readString = "";
        time = millis();
    }
    
    if(readString.substring(0, 3) == "G02" || readString.substring(0, 3) == "G2 "){
        //Serial.println("G02 recognized");
        G2(readString);
        Serial.println("ready");
        Serial.println("gready");
        readString = "";
        time = millis();
    }
    
    if(readString.substring(0, 3) == "G03" || readString.substring(0, 3) == "G3 "){
        //Serial.println("G03 recognized");
        G2(readString);
        Serial.println("ready");
        Serial.println("gready");
        readString = "";
        time = millis();
    }
    
    if(readString.substring(0, 3) == "G10"){
        G10(readString);
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G17"){ //XY plane is the default so no action is taken
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G20"){
        //Serial.println("Inches Set");
        unitScalar = 1; //there are 20 rotations per inch
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G21"){
        //Serial.println("mm set");
        unitScalar = 20; //the machine moves 1.27 mm per rotation
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "G90"){ //G90 is the default so no action is taken
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "M06"){ //Tool change are default so no action is taken
        Serial.println("gready");
        readString = "";
    }
    
    if(readString[0] == 'S' || readString[0] == 's'){
        if(readString[1] == '5'){
            digitalWrite(spindle, HIGH);       
        }
        if(readString[1] == '0'){
            digitalWrite(spindle, LOW);
        }
        Serial.println("gready");
        readString = "";
    }
    
    if(readString.substring(0, 3) == "B01"){
        readString = readString + "     ";
        int apos = readString.indexOf('C');
        char rsect[6] = "     ";
        readString.substring(apos +1,apos+4).toCharArray(rsect, 6);
        //int brightness = atof(rsect);
        //analogWrite(blPin,brightness); //0-255
        readString = "";
        Serial.println("gready");
    }

    if(readString.substring(0, 3) == "B02"){
        readString = readString + "     ";
        int apos = readString.indexOf('C');
        char rsect[6] = "     ";
        readString.substring(apos +1,apos+4).toCharArray(rsect, 6);
        int contrast = atof(rsect);
        //setContrast(contrast); 
        readString = "";
        Serial.println("gready");
    }
    
    if(readString.substring(0, 3) == "B03"){ 
        Serial.println("\nFiles found on the card (name, date and size in bytes): ");
        //root.openRoot(volume);
        // list all files in the card with date and size
        //root.ls(LS_R);
        readString = "";
        Serial.println("gready");
    }
    
    if(readString.substring(0, 3) == "B04"){
        //root.openRoot(volume); 
        // list all files in the card with date and size
        //root.ls(LS_R);
        readString = "";
        Serial.println("gready");
    }
    
    if(readString.substring(0, 3) == "B05"){
        Serial.println("Firmware Version .59");
        readString = "";
        Serial.println("gready");
    }
    
    if((readString[0] == 'T' || readString[0] == 't') && readString[1] != 'e'){
        if(readString[1] == '0'){
            digitalWrite(spindle, LOW);
        }
        if(readString[1] != '1'){
            Serial.print("Please insert tool ");
            Serial.println(readString);
            Serial.println("gready");
        }
        readString = "";
    }
    
    if( millis() - time > 500){
        if (servoDetachFlag == 0){
            writeFloat(5,location.xpos);
            writeFloat(10,location.ypos);
            writeFloat(14,location.zpos);
            EEPROM.write(18,56); //This known value is used as a flag for if valid data can be read from EEPROM later
        }
        servoDetachFlag = 1;
        x.detach();
        y.detach();
        z.detach();
    }
    
    if (readString.length() > 0){
        Serial.println(readString);
        readString = "";
        Serial.println("gready");
    }
}

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
#include "CNC_Functions.h"


int spindle = 11;
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


//Sd2Card card;
//SdVolume volume;
//SdFile root;


void setup(){
	Serial.begin(19200);
	x.write(90); y.write(90); z.write(90);
	Serial.println("ready");
	Serial.println("gready");
	pinMode(spindle, OUTPUT);           // set pin to input
	digitalWrite(spindle, LOW);       // turn on pullup resistors
	analogReference(EXTERNAL);
	pinMode(xpot, INPUT);
	pinMode(xpot, INPUT);
	pinMode(zpot, INPUT);
	pinMode(53, OUTPUT);
	//card.init(SPI_HALF_SPEED, chipSelect); //setup SD card
	//volume.init(card);
	//lcdBegin(); //Initialize the LCD
	//setContrast(contrast); 
	//analogWrite(blPin, backLight); //0-255
	//clearDisplay(WHITE);
	//SetScreen(0.0, 0.0, 0.0);
	initialXspot = PWMread(ypot);
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
	
	SetPos(&location); 
	SetTarget(location.xtarget, location.ytarget, location.ztarget, &location, 123);
	
	i = 0;
	while (i < 23){
		sect[i] = ' ';
		i++;
	}
	
	if(readString.substring(0, 3) == "G00" || readString.substring(0, 3) == "G01" || readString.substring(0, 3) == "G02" || readString.substring(0, 3) == "G03"){
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
		if( servoDetachFlag == 1){
			x.attach(XSERVO);
			y.attach(YSERVO);
			z.attach(ZSERVO);
			//analogWrite(blPin, backLight);
			//setContrast(contrast); 
		}
		servoDetachFlag = 0;
	}
	
	if(readString.substring(0, 3) == "G01" || readString.substring(0, 3) == "G00"){
		//Serial.println("G1 recognized");
		G1(readString);
		Serial.println("ready");
		Serial.println("gready");
		readString = "";
		time = millis();
	}
	
	if(readString.substring(0, 3) == "G02"){
		//Serial.println("G02 recognized");
		G2(readString);
		Serial.println("ready");
		Serial.println("gready");
		readString = "";
		time = millis();
	}
	
	if(readString.substring(0, 3) == "G03"){
		//Serial.println("G03 recognized");
		G2(readString);
		Serial.println("ready");
		Serial.println("gready");
		readString = "";
		time = millis();
	}
	
	if(readString.substring(0, 3) == "G10"){
		Serial.println("Rezero ");
		G10(readString);
		Serial.println("gready");
		readString = "";
	}
	
	if(readString.substring(0, 3) == "G20"){
		Serial.println("Inches Set");
		unitScalor = 20; //there are 20 rotations per inch
		Serial.println("gready");
		readString = "";
	}
	
	if(readString.substring(0, 3) == "G21"){
		//Serial.println("mm set");
		unitScalor = 1/1.27; //the machine moves 1.27 mm per rotation
		Serial.println("gready");
		readString = "";
	}
	
	if(readString.substring(0, 3) == "G90"){
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
		Serial.println("gready");
	}
	
	if(readString.substring(0, 3) == "B04"){
		//root.openRoot(volume); 
		// list all files in the card with date and size
		//root.ls(LS_R);
		Serial.println("gready");
	}
	
	if(readString.substring(0, 3) == "B05"){
		Serial.println("Firmware Version .56");
		Serial.println("gready");
	}
	
	if(readString == "Test Encoders"){
		testEncoders();
		readString = "";
		Serial.println("gready");
	}
	
	if(readString == "Test Motors"){
		testMotors();
		readString = "";
		Serial.println("gready");
	}
	
	if(readString == "Test Both"){
		testBoth();
		readString = "";
		Serial.println("gready");
	}
	
	if(readString[0] == 'T' || readString[0] == 't'){
		if(readString[1] == '0'){
			digitalWrite(spindle, LOW);
		}
		if(readString[1] != '1'){
			Serial.print("Please insert tool ");
			Serial.println(readString);
			Serial.println("gready");
		}
	}
	
	if( millis() - time > 500){
		servoDetachFlag = 1;
		x.detach();
		y.detach();
		z.detach();
	}
	
	if( millis() - time > 30000){
		long fadeVal = backLight+(.01*(30000.0-float(millis() - time)));
		if (fadeVal < 0){
			fadeVal = 0;
		}
		//analogWrite(blPin, fadeVal);
	}
	
	if( millis() - time > 45000){
		//setContrast(0); 
	}
	
	if (readString.length() > 0){
		//Serial.print("Didn't know what to do with: ");
		//Serial.println(readString);
		readString = "";
		Serial.println("gready");
	}
}

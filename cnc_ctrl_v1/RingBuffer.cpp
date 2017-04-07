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

/*
The RingBuffer module creates a circular character buffer used for storing incoming
serial data.
*/

#include "Arduino.h"
#include "RingBuffer.h"

#define BUFFERSIZE 128

char buffer[BUFFERSIZE];
int beginningOfString = 0;             //points to the first valid character which can be read
int endOfString       = 0;             //points to the first open space which can be written

RingBuffer::RingBuffer(){
    
}

void RingBuffer::write(char letter){
    /*
    
    Write one character into the ring buffer.
    
    */
    
    buffer[endOfString] = letter;
    _incrementEnd();
}

char RingBuffer::read(){
    /*
    
    Read one character from the ring buffer.
    
    */
    
    char letter;
    if (beginningOfString == endOfString){
        letter = '\0';                          //if the buffer is empty return null
    }
    else{
        letter = buffer[beginningOfString];     //else return first character
        buffer[beginningOfString] = '\0';       //set the read character to null so it cannot be read again
    }
    _incrementBeginning();
    
    return letter;
}

String RingBuffer::readLine(){
    /*
   
    Return one line (terminated with \n) from the buffer
   
    */
    
    String lineToReturn;
    
    bool lineDetected = false;
    
    int  i = 0;
    while (i < BUFFERSIZE){                     //This will always run 128 times even if the buffer isn't full which is a waste
        if(buffer[i] == '\n'){                  //Check to see if the buffer contains a complete line terminated with \n
            lineDetected = true;
        }
        i++;
    }
    
    //Serial.print("Line detected?:");
    //Serial.println(lineDetected);
    
    if(lineDetected){
        char lastReadValue;
        while(lastReadValue != '\n'){                   //read until the end of the line is found, building the string
            lastReadValue = read();
            lineToReturn += lastReadValue;
        }
    }
    
    return lineToReturn;
    
}

void RingBuffer::print(){
    Serial.print("Buffer size: ");
    Serial.println(_bufferSize());
    Serial.print("Begin: ");
    Serial.println(beginningOfString);
    Serial.print("End: ");
    Serial.println(endOfString);
    
    Serial.println("Buffer Contents: ");
    int i = 0;
    while(i < BUFFERSIZE){
        Serial.print(buffer[i]);
        i++;
    }
    
    Serial.println(" ");
    
    Serial.print("Read: ");
    Serial.println(readLine());
    
}

void RingBuffer::_incrementBeginning(){
    /*
    
    Increment the pointer to the beginning of the ring buffer by one.
    
    */
    
    if (beginningOfString == endOfString){
        return;                             //don't allow the beginning to pass the end
    }
    else if (beginningOfString < 127){
        beginningOfString++;                //move the beginning up one
    }
    else{
        beginningOfString = 0;              //wrap back to zero
    }
}

void RingBuffer::_incrementEnd(){
    /*
    
    Increment the pointer to the end of the ring buffer by one.
    
    */
    if (endOfString + 1 == beginningOfString){
        Serial.println("buffer overflow");
        return;
    }
    else if (endOfString < 127){
        endOfString++;
    }
    else{
        endOfString = 0;
    }
}

int  RingBuffer::_bufferSize(){
    /*
    
    Returns the number of charicters held in the buffer
    
    */
    
    if(endOfString > beginningOfString){                //if the buffer is linear
        return endOfString - beginningOfString;
    }
    else if (endOfString == beginningOfString){
        return 0;                                       //if the buffer is empty
    }
    else{                                               //if the buffer has wrapped
        return (BUFFERSIZE - beginningOfString) + endOfString;
    }
}
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

char buffer[128];
int beginningOfString = 0;             //points to the first valid character which can be read
int endOfString = 0;                   //points to the first open space which can be written

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
        letter = buffer[beginningOfString];     //else return first charicter
    }
    _incrementBeginning();
    
    return letter;
}

void RingBuffer::print(){
    Serial.print("Buffer size: ");
    Serial.println(endOfString - beginningOfString);
    Serial.print("Begin: ");
    Serial.println(beginningOfString);
    Serial.print("End: ");
    Serial.println(endOfString);
    Serial.println("Buffer Contents: ");
    
    int i = beginningOfString;
    while (i < endOfString){
        Serial.print(buffer[i]);
        i++;
    }
    
    char temp = read();
    
    Serial.print("Read: ");
    Serial.println(temp);
    
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
    if (endOfString < 127){
        endOfString++;
    }
    else{
        endOfString = 0;
    }
}
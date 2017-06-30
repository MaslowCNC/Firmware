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


RingBuffer::RingBuffer(){
    
}
String temps;
void RingBuffer::write(char letter){
    /*
    
    Write one character into the ring buffer.
    
    */
    if (letter != '?'){                    //ignore question marks because grbl sends them all the time
        _buffer[_endOfString] = letter;
        _incrementEnd();
    }
}

char RingBuffer::read(){
    /*
    
    Read one character from the ring buffer.
    
    */
    
    char letter;
    if (_beginningOfString == _endOfString){
        letter = '\0';                          //if the buffer is empty return null
    }
    else{
        letter = _buffer[_beginningOfString];     //else return first character
        _buffer[_beginningOfString] = '\0';       //set the read character to null so it cannot be read again
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
    
    int  i = _beginningOfString;
    while (i !=  _endOfString && !lineDetected){  // if we haven't gotten to the end of the buffer yet
        if(_buffer[i] == '\n'){                  //Check to see if the buffer contains a complete line terminated with \n
            lineDetected = true;
        }
        _incrementVariable(&i);
    }
    
    if(lineDetected){
        char lastReadValue = '\0';
        while(lastReadValue != '\n'){                   //read until the end of the line is found, building the string
            lastReadValue = read();
            lineToReturn += lastReadValue;
        }
    }
    
    return lineToReturn;
    
}

void RingBuffer::print(){
    Serial.print("Buffer Used: ");
    Serial.println(length());
    Serial.print("Begin: ");
    Serial.println(_beginningOfString);
    Serial.print("End: ");
    Serial.println(_endOfString);
    Serial.print(_buffer[_beginningOfString]);
    Serial.print(_buffer[_beginningOfString+1]);
    Serial.print(_buffer[_beginningOfString+2]);
    Serial.print(_buffer[_beginningOfString+3]);
    Serial.print(_buffer[_beginningOfString+4]);
    Serial.println(_buffer[_beginningOfString+5]);
    Serial.print(_buffer[_endOfString-1]);
    Serial.print(_buffer[_endOfString-2]);
    Serial.print(_buffer[_endOfString-3]);
    Serial.println(_buffer[_endOfString-4]);
    
    if (_beginningOfString < _endOfString ){  // only if buffer is linear
        Serial.println("Buffer Contents: ");
        int i = _beginningOfString;
        while(i < _endOfString){
            Serial.print(_buffer[i]);
            i++;
        }
    }
    
    Serial.println(" ");
    
}

void RingBuffer::_incrementBeginning(){
    /*
    
    Increment the pointer to the beginning of the ring buffer by one.
    
    */
    
    if (_beginningOfString == _endOfString)  
        return;   //don't allow the beginning to pass the end
    else 
        _beginningOfString = (_beginningOfString + 1) % BUFFERSIZE;    //move the beginning up one and wrap to zero based upon BUFFERSIZE
}

void RingBuffer::_incrementEnd(){
    /*
    
    Increment the pointer to the end of the ring buffer by one.
    
    */
    if ( spaceAvailable() == 0 ) {
        Serial.println("buffer overflow");
        Serial.print("Buffer begin: ");
        Serial.println(_beginningOfString);
        Serial.print("Buffer end: ");
        Serial.println(_endOfString);
        Serial.print("BufferLength: ");
        Serial.println(length());
        }
    else
        _endOfString = (_endOfString+1) % BUFFERSIZE;
 }

void RingBuffer::_incrementVariable(int* variable){
    /*
    Increment the target variable. 
    */
    *variable = (*variable + 1 ) % BUFFERSIZE;
   }

int  RingBuffer::spaceAvailable(){
    /*
    Returns the number of characters held in the buffer
    */
  return  BUFFERSIZE - length() - 1;
}

int RingBuffer::length(void)
{
  if ( _endOfString >= _beginningOfString ) // Linear
    return _endOfString - _beginningOfString ;
  else          // must have rolled
    return  ( BUFFERSIZE - _beginningOfString + _endOfString); 
}


void RingBuffer::empty(){
    /*
    Empty the contents of the ring buffer
    */
    
    _beginningOfString = 0;
    _endOfString       = 0;
}

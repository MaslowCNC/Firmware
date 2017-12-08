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

// This file contains precompile configuration settings that apply to the 
// whole system


#define verboseDebug 0     // set to 0 for no debug messages, 1 for single-line messages, 2 to also output ring buffer contents
#define misloopDebug 0     // set to 1 for a warning every time the movement loop fails 
                           // to complete before being interrupted, helpful for loop
                           // LOOPINTERVAL tuning
#define LOOPINTERVAL 10000 // What is the frequency of the PID loop in microseconds
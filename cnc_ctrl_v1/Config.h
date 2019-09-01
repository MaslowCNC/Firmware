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

#ifndef config_h
#define config_h

// Debugging Options
#define verboseDebug 0     // set to 0 for no debug messages, 1 for single-line messages, 2 to also output ring buffer contents
#define misloopDebug 0     // set to 1 for a warning every time the movement loop fails
                           // to complete before being interrupted, helpful for loop
                           // LOOPINTERVAL tuning
#define KINEMATICSDBG 0    // set to 1 for additional kinematics debug messaging

#define FAKE_SERVO_PERMITTED 42 // store this value
#define FAKE_SERVO 4095    // in EEPROM[ 4095 ] to preserve
                           // the state of FAKE_SERVO mode over resets.
                           // Use 'B99 ON' to turn FAKE_SERVO mode on and set EEPROM[ 4095 ] to '1',
                           // 'B99' with no parameter, or any parameter other than 'ON' 
                           // puts a '0' in that location and turns FAKE_SERVO mode off.
                           // Useful for testing on an arduino only (e.g. without motors).

// #define SIMAVR          // Uncomment this if you plan to run the Firmware in the simavr
                           // simulator. Normally, you would not define this directly, but
                           // use PlatformIO to build the simavr environment.

// #define alarmsTLE9201   // Uncomment this to make TLE9201 over-current/over-temp alarms active
                           // Note that in normal operation these alarms are often triggered by 
                           // abrupt direction change. Until acceleration control is available,
                           // the alarms aren't useful.

#define LOOPINTERVAL 10000 // What is the frequency of the PID loop in microseconds

// Define version detect pins
#define VERS1 22
#define VERS2 23
#define VERS3 24
#define VERS4 25
#define VERS5 26
#define VERS6 27

// Serial variables
#define INCBUFFERLENGTH 128 // The number of bytes(characters) allocated to the
                            // incoming buffer.
#define EXPGCODELINE 60     // Maximum expected Gcode line length in characters
                            // including line ending character(s).  Assumes
                            // client will not send more than this.  Ground
                            // Control is currently set to 60.  NIST spec allows
                            // 256. This value must be <= INCBUFFERLENGTH
#define MAXBUFFERLINES 4    // The maximum number of lines allowed in the buffer
#define POSITIONTIMEOUT 200 // The minimum number of milliseconds between
                            // position reports sent to Ground Control.  This
                            // cannot be larger than the connection timout in
                            // Ground Control which is 2000, a smaller number
                            // takes more processing time for sending data
                            // a larger number make position updates in GC less
                            // smooth.  This is only a minimum, and the actual
                            // timeout could be significantly larger.

#define CMD_RESET 0x18      // ctrl-x., if received the program should do a soft reset
                            // if received the program should do a soft reset
#define CMD_RESET2 '`'      // alternate char because GC won't use control characters in a macro

#endif

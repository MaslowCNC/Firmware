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

#ifndef system_h
#define system_h

// Convenience Defines - Maybe move into a nuts and bolts file?
#define FORWARD           1
#define BACKWARD         -1
#define CLOCKWISE        -1
#define COUNTERCLOCKWISE  1
#define MILLIMETERS 1
#define INCHES      25.4

// Storage for global system states
// Some of this could be more appropiately moved to the gcode parser
typedef struct {
  byte stop;                  // Stop flag.
  byte pause;                 // Pause flag.
  byte rcvdMotorSettings;     // Motor Settings Status Flag
  byte rcvdKinematicSettings; // Kinematics Settings Status Flag
  float xPosition;            // Cartessian position of XY axes
  float yPosition;            // Cached because calculating position is intensive
  float steps[3];             // Encoder position of axes
  bool  useRelativeUnits;     // 
  int   lastGCommand;         //Stores the value of the last command run eg: G01 -> 1
  int   lastTool;             //Stores the value of the last tool number eg: T4 -> 4
  int   nextTool;             //Stores the value of the next tool number eg: T4 -> 4
  bool  zAxisAttached;        //Whether the zAxis is controllable
  float inchesToMMConversion; //Used to track whether to convert from inches, can probably be done in a way that doesn't require RAM
  float feedrate;             //The feedrate of the machine in mm/min
  bool  encoderStepsChanged;  //These are used to determine 
  bool  zEncoderStepsChanged; //These are used to determine
} system_t;
extern system_t sys;
extern Axis leftAxis;
extern Axis rightAxis;
extern Axis zAxis;
extern RingBuffer incSerialBuffer;
extern Kinematics kinematics;

bool machineReady();
void finalizeMachineSettings();
void  calibrateChainLengths(String);
void  updateKinematicsSettings(const String&);
void  updateMotorSettings(const String&);
void  setupAxes();
int   getPCBVersion();
void pause();
void maslowDelay(unsigned long);
void  _watchDog();

#endif
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

// This file contains the machine settings that are saved to eeprom

#include "Maslow.h"

void settingsLoadFromEEprom(){
    /* 
    Loads data from EEPROM if EEPROM data is valid, only called on startup
    */
    settingsVersion_t settingsVersionStruct;
    
    settingsReset(); // Load default values first
    EEPROM.get(0, settingsVersionStruct);
    if (settingsVersionStruct.settingsVersion == SETTINGSVERSION &&
        settingsVersionStruct.eepromValidData == EEPROMVALIDDATA){
        // This is a valid data
        EEPROM.get(30, sysSettings);
    }
}


void settingsReset() {
    /* 
    Loads default data into settings, many of these values are approximations
    from the an ideal stock frame.  Other values are just the recommended
    value.  Ideally we want these defaults to match the defaults in GroundControl
    so that if a value is not changed by a user or is not used, it doesn't 
    need to be updated here.
    */
    sysSettings = {
        2438.4, // float machineWidth;
        1219.2, // float machineHeight;
        2978.4, // float distBetweenMotors;
        463.0,  // float motorOffsetY;
        310.0,  // float sledWidth;
        139.0,  // float sledHeight;
        79.0,   // float sledCG;
        1,      // byte kinematicsType;
        100.0,  // float rotationDiskRadius;
        2000,   // int axisHoldTime;
        200,    // int kinematicsMaxGuess;
        1650,   // int originalChainLength;
        8113.7, // float encoderSteps;
        10,     // byte gearTeeth;
        6.35,   // float chainPitch;
        1000,   // int maxFeed;
        false,  // bool zAxisAuto;
        12.60,  // float maxZRPM;
        3.17,   // float zDistPerRot;
        7560.0, // float zEncoderSteps;
        1300.0, // float KpPos;
        0.0,    // float KiPos;
        34.0,   // float KdPos;
        1.0,    // float propWeightPos;
        7.0,    // float KpV;
        0.0,    // float KiV;
        0.28,   // float KdV;
        1.0,    // float propWeightV;
        1300.0, // float zKpPos;
        0.0,    // float zKiPos;
        34.0,   // float zKdPos;
        1.0,    // float zPropWeightPos;
        7.0,    // float zKpV;
        0.0,    // float zKiV;
        0.28,   // float zKdV;
        1.0,    // float zPropWeightV;
        EEPROMVALIDDATA // byte eepromValidData;
    };
}

void settingsSaveToEEprom(){
    /* 
    Saves settings to EEPROM, only called when settings change
    */
    settingsVersion_t settingsVersionStruct = {SETTINGSVERSION, EEPROMVALIDDATA};
    EEPROM.put(0, settingsVersionStruct);
    EEPROM.put(30, sysSettings);
}

void settingsSaveStepstoEEprom(){
    /* 
    Saves position to EEPROM, is called frequently by execSystemRealtime
    */
    EEPROM.put(10, sysSteps);
}

void settingsLoadStepsFromEEprom(){
    /* 
    Saves position to EEPROM, is on startup.  This struct should never change
    so it doesn't check the settings version
    */
    settingsSteps_t tempSteps;
    EEPROM.get(10, tempSteps);
    if (tempSteps.eepromValidData == EEPROMVALIDDATA){
        sysSteps = tempSteps;
    }
    else { //5 left, 105 right, 205 zaxis
        if (EEPROM.read(5) == EEPROMVALIDDATA &&
            EEPROM.read(105) == EEPROMVALIDDATA &&
            EEPROM.read(205) == EEPROMVALIDDATA){
            // Try and load position from pre settings days
            float l, r , z;
            EEPROM.get(9, l);
            EEPROM.get(109, r);
            EEPROM.get(209, z);
            // Old method stored position as a float of rotations
            // TODO figure out when this runs how to convert to steps. 
            // perhaps just set axis to position and let conversion happen
            // naturally there
            // There is a small bug in the old way of doing it as the number
            // of rotations is calculated using either the encoderSteps per 
            // rotation or distance per rotation, changing of either of these
            // values requires a recalibration of the machine
            //position = {l,r,z, EEPROMVALIDDATA}
        }
        else {
            systemRtExecAlarm |= ALARM_POSITION_LOST;  // if this same global is touched by ISR then need to make atomic somehow
                                                       // also need to consider if need difference between flag with bits and
                                                       // error message as a byte.
        }
    }
}
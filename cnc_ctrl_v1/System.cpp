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

// This file contains system level functions and states

#include "Maslow.h"

bool machineReady(){
  bool ret = false;
  if (sys.rcvdMotorSettings && sys.rcvdKinematicSettings){
      ret = true;
  }
  return ret;
}

void  calibrateChainLengths(String gcodeLine){
    /*
    The calibrateChainLengths function lets the machine know that the chains are set to a given length where each chain is ORIGINCHAINLEN
    in length
    */
    
    if (extractGcodeValue(gcodeLine, 'L', 0)){
        //measure out the left chain
        Serial.println(F("Measuring out left chain"));
        singleAxisMove(&leftAxis, ORIGINCHAINLEN, 800);
        
        Serial.print(leftAxis.read());
        Serial.println(F("mm"));
        
        leftAxis.detach();
    }
    else if(extractGcodeValue(gcodeLine, 'R', 0)){
        //measure out the right chain
        Serial.println(F("Measuring out right chain"));
        singleAxisMove(&rightAxis, ORIGINCHAINLEN, 800);
        
        Serial.print(rightAxis.read());
        Serial.println(F("mm"));
        
        rightAxis.detach();
    }
    
}

void  updateKinematicsSettings(const String& readString){
    /*
    Updates the machine dimensions from the Ground Control settings
    */
    
    //Extract the settings values

    float bedWidth           = extractGcodeValue(readString, 'A', -1);
    float bedHeight          = extractGcodeValue(readString, 'C', -1);
    float distBetweenMotors  = extractGcodeValue(readString, 'Q', -1);
    float motorOffsetY       = extractGcodeValue(readString, 'E', -1);
    float sledWidth          = extractGcodeValue(readString, 'F', -1);
    float sledHeight         = extractGcodeValue(readString, 'R', -1);
    float sledCG             = extractGcodeValue(readString, 'H', -1);

    float kinematicsType     = extractGcodeValue(readString, 'Y', -1);
    float rotationDiskRadius = extractGcodeValue(readString, 'Z', -1);
    
    
    
    //Change the machine dimensions in the kinematics if new values have been received
    if (sledWidth != -1){
        kinematics.l            = sledWidth;
    }
    if (sledHeight != -1){
        kinematics.s            = sledHeight;
    }
    if (sledCG != -1){
        kinematics.h3           = sledCG;
    }
    //if (distPerRot != -1){
    //    kinematics.R            = distPerRot / (2.0*3.14159);
    //}
    if (distBetweenMotors != -1){
        kinematics.D            = distBetweenMotors;
    }
    if (motorOffsetY != -1){
        kinematics.motorOffsetY = motorOffsetY;
    }
    if (bedWidth != -1){
        kinematics.machineWidth = bedWidth;
    }
    if (bedHeight != -1){
        kinematics.machineHeight= bedHeight;
    }
    if (kinematicsType != -1){
        kinematics.kinematicsType = kinematicsType;
    }
    if (rotationDiskRadius != -1){
        kinematics.rotationDiskRadius = rotationDiskRadius;
    }
    
    //propagate the new values
    sys.rcvdKinematicSettings = 1;
    kinematics.recomputeGeometry();
    finalizeMachineSettings();
    
    Serial.println(F("Kinematics Settings Loaded"));
}

void updateMotorSettings(const String& readString){
    /*
    
    Update settings related to the motor configurations
    
    */
    
    if (extractGcodeValue(readString, 'I', -1) != -1){
        sys.zAxisAttached            = extractGcodeValue(readString, 'I', -1);
    }
    float encoderSteps       = extractGcodeValue(readString, 'J', -1);
    float gearTeeth          = extractGcodeValue(readString, 'K', -1);
    float chainPitch         = extractGcodeValue(readString, 'M', -1);
    
    float zDistPerRot        = extractGcodeValue(readString, 'N', -1);
    float zEncoderSteps      = extractGcodeValue(readString, 'P', -1);
    
    float propWeight         = extractGcodeValue(readString, 'R', -1);
    float KpPos              = extractGcodeValue(readString, 'S', -1);
    float KiPos              = extractGcodeValue(readString, 'T', -1);
    float KdPos              = extractGcodeValue(readString, 'U', -1);
    float KpV                = extractGcodeValue(readString, 'V', -1);
    float KiV                = extractGcodeValue(readString, 'W', -1);
    float KdV                = extractGcodeValue(readString, 'X', -1);
    if (extractGcodeValue(readString, 'Y', -1) != -1) {
	zAxisAuto            = extractGcodeValue(readString, 'Y', -1);
    }
      
    //Write the PID values to the axis if new ones have been received
    if (KpPos != -1){
        leftAxis.setPIDValues(KpPos, KiPos, KdPos, propWeight, KpV, KiV, KdV);
        rightAxis.setPIDValues(KpPos, KiPos, KdPos, propWeight, KpV, KiV, KdV);
        zAxis.setPIDValues(KpPos, KiPos, KdPos, propWeight, KpV, KiV, KdV);
    }
    
    //Change the motor properties in cnc_funtions if new values have been sent
    if (gearTeeth != -1 and chainPitch != -1){
        float distPerRot = gearTeeth*chainPitch; 
        leftAxis.changePitch(distPerRot);
        rightAxis.changePitch(distPerRot);
        zAxis.changePitch(zDistPerRot);
        
        kinematics.R = (gearTeeth * chainPitch)/(2.0 * 3.14159);
    }
    
    //update the number of encoder steps if new values have been received
    if (encoderSteps != -1){
        leftAxis.changeEncoderResolution(encoderSteps);
        rightAxis.changeEncoderResolution(encoderSteps);
        sys.encoderStepsChanged = true;
    }
    if (zEncoderSteps != -1){
        zAxis.changeEncoderResolution(zEncoderSteps);
        sys.zEncoderStepsChanged = true;
    }
    
    sys.rcvdMotorSettings = 1;
    finalizeMachineSettings(); 
    Serial.println(F("Motor Settings Loaded"));
}

void finalizeMachineSettings(){
    if(machineReady()){
        if (sys.encoderStepsChanged){
            leftAxis.loadPositionFromMemory();
            rightAxis.loadPositionFromMemory();
            
            
            sys.encoderStepsChanged = false;
        }
        if (sys.zEncoderStepsChanged){
            zAxis.loadPositionFromMemory();
            sys.zEncoderStepsChanged = false;
        }
        kinematics.forward(leftAxis.read(), rightAxis.read(), &sys.xPosition, &sys.yPosition);
    }
}

void   setupAxes(){
    /*
    
    Detect the version of the Arduino shield connected, and use the appropriate pins
    
    This function runs before the serial port is open so the version is not printed here
    
    */
    
    // These shouldn't be CAPS, they are not precompile defines
    int ENCODER1A;
    int ENCODER1B;
    int ENCODER2A;
    int ENCODER2B;
    int ENCODER3A;
    int ENCODER3B;
    
    int IN1;
    int IN2;
    int IN3;
    int IN4;
    int IN5;
    int IN6;
    
    int ENA;
    int ENB;
    int ENC;
    
    //read the pins which indicate the PCB version
    int pcbVersion = getPCBVersion();
    
    if(pcbVersion == 0){
        //Beta PCB v1.0 Detected
        //MP1 - Right Motor
        ENCODER1A = 18; // INPUT
        ENCODER1B = 19; // INPUT
        IN1 = 9;        // OUTPUT
        IN2 = 8;        // OUTPUT
        ENA = 6;        // PWM
        
        //MP2 - Z-axis
        ENCODER2A = 2;  // INPUT
        ENCODER2B = 3;  // INPUT
        IN3 = 11;       // OUTPUT
        IN4 = 10;       // OUTPUT
        ENB = 7;        // PWM
        
        //MP3 - Left Motor
        ENCODER3A = 21; // INPUT
        ENCODER3B = 20; // INPUT
        IN5 = 12;       // OUTPUT
        IN6 = 13;       // OUTPUT
        ENC = 5;        // PWM
    }
    else if(pcbVersion == 1){
        //PCB v1.1 Detected
        //MP1 - Right Motor
        ENCODER1A = 20; // INPUT
        ENCODER1B = 21; // INPUT
        IN1 = 6;        // OUTPUT
        IN2 = 4;        // OUTPUT
        ENA = 5;        // PWM
        
        //MP2 - Z-axis
        ENCODER2A = 19; // INPUT
        ENCODER2B = 18; // INPUT
        IN3 = 9;        // OUTPUT
        IN4 = 7;        // OUTPUT
        ENB = 8;        // PWM
        
        //MP3 - Left Motor
        ENCODER3A = 2;   // INPUT
        ENCODER3B = 3;   // INPUT
        IN5 = 10;        // OUTPUT
        IN6 = 11;        // OUTPUT
        ENC = 12;        // PWM
    }
    else if(pcbVersion == 2){
        //PCB v1.2 Detected
        
        //MP1 - Right Motor
        ENCODER1A = 20;  // INPUT
        ENCODER1B = 21;  // INPUT
        IN1 = 4;         // OUTPUT
        IN2 = 6;         // OUTPUT
        ENA = 5;         // PWM
        
        //MP2 - Z-axis
        ENCODER2A = 19;  // INPUT
        ENCODER2B = 18;  // INPUT
        IN3 = 7;         // OUTPUT
        IN4 = 9;         // OUTPUT
        ENB = 8;         // PWM
        
        //MP3 - Left Motor
        ENCODER3A = 2;   // INPUT
        ENCODER3B = 3;   // INPUT
        IN5 = 11;        // OUTPUT
        IN6 = 12;        // OUTPUT
        ENC = 10;        // PWM


    }
    leftAxis.setup (ENC, IN6, IN5, ENCODER3B, ENCODER3A, 'L',  LEFT_EEPROM_ADR, LOOPINTERVAL);
    rightAxis.setup(ENA, IN1, IN2, ENCODER1A, ENCODER1B, 'R', RIGHT_EEPROM_ADR, LOOPINTERVAL);
    zAxis.setup    (ENB, IN3, IN4, ENCODER2B, ENCODER2A, 'Z',     Z_EEPROM_ADR, LOOPINTERVAL);
}

int getPCBVersion(){
    return (8*digitalRead(53) + 4*digitalRead(52) + 2*digitalRead(23) + 1*digitalRead(22)) - 1;
}


// This should likely go away and be handled by setting the pause flag and then
// pausing in the execSystemRealtime function
// Need to check if all returns from this subsequently look to sys.stop
void pause(){
    /*
    
    The pause command pauses the machine in place without flushing the lines stored in the machine's
    buffer.
    
    When paused the machine enters a while() loop and doesn't exit until the '~' cycle resume command 
    is issued from Ground Control.
    
    */
    
    sys.pause = true;
    Serial.println(F("Maslow Paused"));
    
    while(1){
        
        // Run realtime commands
        execSystemRealtime();
        if (sys.stop){return;}
        
        if (!sys.pause){
            return;
        }
    }    
}


// This is an important concept.  I think maybe this should be expanded and Used
// whenever we have a delay.  This should be all of the 'realtime' operations
// and should probably include check for stop command.  Although, the holdPosition
// would have to be moved out of here, but I think that is probably correct

// need to check if all returns from here check for sys.stop
void maslowDelay(unsigned long waitTimeMs) {
  /*
   * Provides a time delay while holding the machine position, reading serial commands,
   * and periodically sending the machine position to Ground Control.  This prevents
   * Ground Control from thinking that the connection is lost.
   * 
   * This is similar to the pause() command above, but provides a time delay rather than
   * waiting for the user (through Ground Control) to tell the machine to continue.
   */
   
    unsigned long startTime  = millis();
    
    while ((millis() - startTime) < waitTimeMs){
        execSystemRealtime();
        if (sys.stop){return;}
    }
}

// This executes all of the actions that we want to happen in 'realtime'.  This
// should be called whenever there is a delay in the code or when it may have
// been a long time since this command was called.  Everything that is executed
// by this command should be relatively fast.  Should always check for sys.stop
// after returning from this function
void execSystemRealtime(){
    readSerialCommands();
    returnPoz();
    // check systemRtExecAlarm flag and do stuff
}

// This should be the ultimate fallback, it would be best if we didn't even need 
// something like this at all
void  _watchDog(){
    /*
    Watchdog tells ground control that the machine is ready every second. _watchDog() should only be called when 
    the machine is actually ready.
    
    This fixes the issue where the machine is ready, but Ground Control doesn't know the machine is ready and the system locks up.
    */
    static unsigned long lastRan = millis();
    unsigned long        timeout = 5000;
    
    if ((millis() - lastRan) > timeout){
        
        if (!leftAxis.attached() and !rightAxis.attached() and !zAxis.attached()){
            
            if (incSerialBuffer.length() == 0) {       // if the buffer is empty
                #if defined (verboseDebug) && verboseDebug > 0              
                Serial.println(F("_watchDog requesting new code"));
                #endif
                reportStatusMessage(STATUS_OK);
            }
        }
        
        lastRan = millis();
    }
}

byte systemExecuteCmdstring(String& cmdString){
    /*
    This function processes the $ system commands

    This is taken heavily from grbl.  https://github.com/grbl/grbl
    */
    byte char_counter = 1;
    byte helper_var = 0; // Helper variable
    float parameter, value;
    if (cmdString.length() == 1){
        reportMaslowHelp();
    }
    else {
        switch( cmdString[char_counter] ) {
          case '$': case 'G': case 'C': case 'X':
            if ( cmdString.length() > 2 ) { return(STATUS_INVALID_STATEMENT); }
            switch( cmdString[char_counter] ) {
              case '$' : // Prints Maslow settings
                // if ( sys.state & (STATE_CYCLE | STATE_HOLD) ) { return(STATUS_IDLE_ERROR); } // Block during cycle. Takes too long to print.
                // else {
                  reportMaslowSettings();
                // }
                break;
              // case 'G' : // Prints gcode parser state
              //   report_gcode_modes();
              //   break;
              // case 'C' : // Set check g-code mode [IDLE/CHECK]
              //   // Perform reset when toggling off. Check g-code mode should only work if Grbl
              //   // is idle and ready, regardless of alarm locks. This is mainly to keep things
              //   // simple and consistent.
              //   if ( sys.state == STATE_CHECK_MODE ) {
              //     mc_reset();
              //     report_feedback_message(MESSAGE_DISABLED);
              //   } else {
              //     if (sys.state) { return(STATUS_IDLE_ERROR); } // Requires no alarm mode.
              //     sys.state = STATE_CHECK_MODE;
              //     report_feedback_message(MESSAGE_ENABLED);
              //   }
              //   break;
              // case 'X' : // Disable alarm lock [ALARM]
              //   if (sys.state == STATE_ALARM) {
              //     report_feedback_message(MESSAGE_ALARM_UNLOCK);
              //     sys.state = STATE_IDLE;
              //     // Don't run startup script. Prevents stored moves in startup from causing accidents.
              //     if (system_check_safety_door_ajar()) { // Check safety door switch before returning.
              //       bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
              //       protocol_execute_realtime(); // Enter safety door mode.
              //     }
              //   } // Otherwise, no effect.
              //   break;
            }
          //case 'J' : break;  // Jogging methods
              // TODO: Here jogging can be placed for execution as a seperate subprogram. It does not need to be
              // susceptible to other realtime commands except for e-stop. The jogging function is intended to
              // be a basic toggle on/off with controlled acceleration and deceleration to prevent skipped
              // steps. The user would supply the desired feedrate, axis to move, and direction. Toggle on would
              // start motion and toggle off would initiate a deceleration to stop. One could 'feather' the
              // motion by repeatedly toggling to slow the motion to the desired location. Location data would
              // need to be updated real-time and supplied to the user through status queries.
              //   More controlled exact motions can be taken care of by inputting G0 or G1 commands, which are
              // handled by the planner. It would be possible for the jog subprogram to insert blocks into the
              // block buffer without having the planner plan them. It would need to manage de/ac-celerations
              // on its own carefully. This approach could be effective and possibly size/memory efficient.
              // break;
              // }
              //break;
          // default :
          //   // Block any system command that requires the state as IDLE/ALARM. (i.e. EEPROM, homing)
          //   if ( !(sys.state == STATE_IDLE || sys.state == STATE_ALARM) ) { return(STATUS_IDLE_ERROR); }
          //   switch( line[char_counter] ) {
          //     case '#' : // Print Grbl NGC parameters
          //       if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
          //       else { report_ngc_parameters(); }
          //       break;
          //     case 'H' : // Perform homing cycle [IDLE/ALARM]
          //       if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) {
          //         sys.state = STATE_HOMING; // Set system state variable
          //         // Only perform homing if Grbl is idle or lost.
          // 
          //         // TODO: Likely not required.
          //         if (system_check_safety_door_ajar()) { // Check safety door switch before homing.
          //           bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
          //           protocol_execute_realtime(); // Enter safety door mode.
          //         }
          // 
          // 
          //         mc_homing_cycle();
          //         if (!sys.abort) {  // Execute startup scripts after successful homing.
          //           sys.state = STATE_IDLE; // Set to IDLE when complete.
          //           st_go_idle(); // Set steppers to the settings idle state before returning.
          //           system_execute_startup(line);
          //         }
          //       } else { return(STATUS_SETTING_DISABLED); }
          //       break;
          //     case 'I' : // Print or store build info. [IDLE/ALARM]
          //       if ( line[++char_counter] == 0 ) {
          //         settings_read_build_info(line);
          //         report_build_info(line);
          //       } else { // Store startup line [IDLE/ALARM]
          //         if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
          //         helper_var = char_counter; // Set helper variable as counter to start of user info line.
          //         do {
          //           line[char_counter-helper_var] = line[char_counter];
          //         } while (line[char_counter++] != 0);
          //         settings_store_build_info(line);
          //       }
          //       break;
          //     case 'R' : // Restore defaults [IDLE/ALARM]
          //       if (line[++char_counter] != 'S') { return(STATUS_INVALID_STATEMENT); }
          //       if (line[++char_counter] != 'T') { return(STATUS_INVALID_STATEMENT); }
          //       if (line[++char_counter] != '=') { return(STATUS_INVALID_STATEMENT); }
          //       if (line[char_counter+2] != 0) { return(STATUS_INVALID_STATEMENT); }
          //       switch (line[++char_counter]) {
          //         case '$': settings_restore(SETTINGS_RESTORE_DEFAULTS); break;
          //         case '#': settings_restore(SETTINGS_RESTORE_PARAMETERS); break;
          //         case '*': settings_restore(SETTINGS_RESTORE_ALL); break;
          //         default: return(STATUS_INVALID_STATEMENT);
          //       }
          //       report_feedback_message(MESSAGE_RESTORE_DEFAULTS);
          //       mc_reset(); // Force reset to ensure settings are initialized correctly.
          //       break;
          //     case 'N' : // Startup lines. [IDLE/ALARM]
          //       if ( line[++char_counter] == 0 ) { // Print startup lines
          //         for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++) {
          //           if (!(settings_read_startup_line(helper_var, line))) {
          //             report_status_message(STATUS_SETTING_READ_FAIL);
          //           } else {
          //             report_startup_line(helper_var,line);
          //           }
          //         }
          //         break;
          //       } else { // Store startup line [IDLE Only] Prevents motion during ALARM.
          //         if (sys.state != STATE_IDLE) { return(STATUS_IDLE_ERROR); } // Store only when idle.
          //         helper_var = true;  // Set helper_var to flag storing method.
          //         // No break. Continues into default: to read remaining command characters.
          //       }
          //     default :  // Storing setting methods [IDLE/ALARM]
          //       if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
          //       if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
          //       if (helper_var) { // Store startup line
          //         // Prepare sending gcode block to gcode parser by shifting all characters
          //         helper_var = char_counter; // Set helper variable as counter to start of gcode block
          //         do {
          //           line[char_counter-helper_var] = line[char_counter];
          //         } while (line[char_counter++] != 0);
          //         // Execute gcode block to ensure block is valid.
          //         helper_var = gc_execute_line(line); // Set helper_var to returned status code.
          //         if (helper_var) { return(helper_var); }
          //         else {
          //           helper_var = trunc(parameter); // Set helper_var to int value of parameter
          //           settings_store_startup_line(helper_var,line);
          //         }
          //       } else { // Store global setting.
          //         if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
          //         if((line[char_counter] != 0) || (parameter > 255)) { return(STATUS_INVALID_STATEMENT); }
          //         return(settings_store_global_setting((uint8_t)parameter, value));
          //       }
          //   }
        }
    }
}
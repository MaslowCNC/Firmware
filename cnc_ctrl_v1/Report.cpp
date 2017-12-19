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

// This file contains the functions for outgoing Serial responses

#include "Maslow.h"

void  reportStatusMessage(byte status_code){
    /*
    
    Sends confirmation protocol response for commands. For every incoming line,
    this method responds with an 'ok' for a successful command or an 'error:'
    to indicate some error event with the line or some critical system error during
    operation.
    
    Taken from Grbl http://github.com/grbl/grbl
    */
    if (status_code == 0) { // STATUS_OK
      Serial.println(F("ok"));
    } else {
      Serial.print(F("error: "));
      #ifdef REPORT_GUI_MODE
        Serial.println(status_code);
      #else
        switch(status_code) {
          // case STATUS_EXPECTED_COMMAND_LETTER:
          // Serial.println(F("Expected command letter")); break;
          // case STATUS_BAD_NUMBER_FORMAT:
          // Serial.println(F("Bad number format")); break;
          case STATUS_INVALID_STATEMENT:
          Serial.println(F("Invalid statement")); break;
          // case STATUS_NEGATIVE_VALUE:
          // Serial.println(F("Value < 0")); break;
          // case STATUS_SETTING_DISABLED:
          // Serial.println(F("Setting disabled")); break;
          // case STATUS_SETTING_STEP_PULSE_MIN:
          // Serial.println(F("Value < 3 usec")); break;
          // case STATUS_SETTING_READ_FAIL:
          // Serial.println(F("EEPROM read fail. Using defaults")); break;
          // case STATUS_IDLE_ERROR:
          // Serial.println(F("Not idle")); break;
          // case STATUS_ALARM_LOCK:
          // Serial.println(F("Alarm lock")); break;
          // case STATUS_SOFT_LIMIT_ERROR:
          // Serial.println(F("Homing not enabled")); break;
          // case STATUS_OVERFLOW:
          // Serial.println(F("Line overflow")); break;
          // #ifdef MAX_STEP_RATE_HZ
          //   case STATUS_MAX_STEP_RATE_EXCEEDED:
          //   Serial.println(F("Step rate > 30kHz")); break;
          // #endif
          // Common g-code parser errors.
          // case STATUS_GCODE_MODAL_GROUP_VIOLATION:
          // Serial.println(F("Modal group violation")); break;
          // case STATUS_GCODE_UNSUPPORTED_COMMAND:
          // Serial.println(F("Unsupported command")); break;
          // case STATUS_GCODE_UNDEFINED_FEED_RATE:
          // Serial.println(F("Undefined feed rate")); break;
          default:
            // Remaining g-code parser errors with error codes
            Serial.print(F("Invalid gcode ID:"));
            Serial.println(status_code); // Print error code for user reference
        }
      #endif
    }
}

// Prints alarm messages.
void  reportAlarmMessage(byte alarm_code)
{
  Serial.print(F("ALARM: "));
  #ifdef REPORT_GUI_MODE
    Serial.println(alarm_code);
  #else
    switch (alarm_code) {
      case ALARM_POSITION_LOST:
      Serial.println(F("Position Lost")); break;
    }
  #endif
}

// Maslow global settings print out.
// NOTE: The numbering scheme here must correlate to storing in settings.c
void reportMaslowSettings() {
  // Print Maslow settings.
  // Taken from Grbl. http://github.com/grbl/grbl
  #ifdef REPORT_GUI_MODE
    Serial.print(F("$1=")); Serial.println(sysSettings.machineWidth);
  #else
    Serial.print(PSTR("$0=")); Serial.print(sysSettings.machineWidth);
    Serial.print(PSTR(" (units for prior line)\r\n$1=")); Serial.print(sysSettings.machineWidth);
  #endif

  // Print axis settings
  // uint8_t idx, set_idx;
  // uint8_t val = AXIS_SETTINGS_START_VAL;
  // for (set_idx=0; set_idx<AXIS_N_SETTINGS; set_idx++) {
  //   for (idx=0; idx<N_AXIS; idx++) {
  //     printPgmString(PSTR("$"));
  //     print_uint8_base10(val+idx);
  //     printPgmString(PSTR("="));
  //     switch (set_idx) {
  //       case 0: printFloat_SettingValue(settings.steps_per_mm[idx]); break;
  //       case 1: printFloat_SettingValue(settings.max_rate[idx]); break;
  //       case 2: printFloat_SettingValue(settings.acceleration[idx]/(60*60)); break;
  //       case 3: printFloat_SettingValue(-settings.max_travel[idx]); break;
  //     }
  //     #ifdef REPORT_GUI_MODE
  //       printPgmString(PSTR("\r\n"));
  //     #else
  //       printPgmString(PSTR(" ("));
  //       switch (idx) {
  //         case X_AXIS: printPgmString(PSTR("x")); break;
  //         case Y_AXIS: printPgmString(PSTR("y")); break;
  //         case Z_AXIS: printPgmString(PSTR("z")); break;
  //       }
  //       switch (set_idx) {
  //         case 0: printPgmString(PSTR(", step/mm")); break;
  //         case 1: printPgmString(PSTR(" max rate, mm/min")); break;
  //         case 2: printPgmString(PSTR(" accel, mm/sec^2")); break;
  //         case 3: printPgmString(PSTR(" max travel, mm")); break;
  //       }
  //       printPgmString(PSTR(")\r\n"));
  //     #endif
  //   }
  //   val += AXIS_SETTINGS_INCREMENT;
  // }
}

void  returnError(){
    /*
    Prints the machine's positional error and the amount of space available in the 
    gcode buffer
    */
        Serial.print(F("[PE:"));
        Serial.print(leftAxis.error());
        Serial.print(',');
        Serial.print(rightAxis.error());
        Serial.print(',');
        Serial.print(incSerialBuffer.spaceAvailable());
        Serial.println(F("]"));
}

void  returnPoz(){
    /*
    Causes the machine's position (x,y) to be sent over the serial connection updated on the UI
    in Ground Control. Also causes the error report to be sent. Only executes 
    if hasn't been called in at least POSITIONTIMEOUT ms.
    */
    
    static unsigned long lastRan = millis();
    
    if (millis() - lastRan > POSITIONTIMEOUT){
        
        
        Serial.print(F("<Idle,MPos:"));
        Serial.print(sys.xPosition/sys.inchesToMMConversion);
        Serial.print(F(","));
        Serial.print(sys.yPosition/sys.inchesToMMConversion);
        Serial.print(F(","));
        Serial.print(zAxis.read()/sys.inchesToMMConversion);
        Serial.println(F(",WPos:0.000,0.000,0.000>"));
        
        returnError();
        
        lastRan = millis();
    }
    
}

void  reportMaslowHelp(){
    /*
    This function outputs a brief summary of the $ system commands available.
    The list is somewhat aspirational based on what Grbl offers. Maslow
    does not currently support all of these features.

    This is taken heavily from grbl.  https://github.com/grbl/grbl
    */
    #ifndef REPORT_GUI_MODE
        Serial.println(F("$$ (view Maslow settings)"));
        Serial.println(F("$# (view # parameters)"));
        Serial.println(F("$G (view parser state)"));
        Serial.println(F("$I (view build info)"));
        Serial.println(F("$N (view startup blocks)"));
        Serial.println(F("$x=value (save Maslow setting)"));
        Serial.println(F("$Nx=line (save startup block)"));
        Serial.println(F("$C (check gcode mode)"));
        Serial.println(F("$X (kill alarm lock)"));
        Serial.println(F("$H (run homing cycle)"));
        Serial.println(F("~ (cycle start)"));
        Serial.println(F("! (feed hold)"));
        Serial.println(F("? (current status)"));
        Serial.println(F("ctrl-x (reset Maslow)"));
    #endif
}
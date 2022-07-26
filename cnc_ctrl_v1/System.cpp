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

bool TLE5206;
bool TLE9201;
bool TB6643;

// extern values using AUX pins defined in  setupAxes()
int SpindlePowerControlPin;  // output for controlling spindle power
int ProbePin;                // use this input for zeroing zAxis with G38.2 gcode
int LaserPowerPin;           // Use this output to turn on and off a laser diode

// external values used to identify TLE9201 Current/Temperature alarms
int SO1;
int SO2;
int SO3;
int ENA;
int ENB;
int ENC;

void  calibrateChainLengths(String gcodeLine){
    /*
    The calibrateChainLengths function lets the machine know that the chains are set to a given length where each chain is ORIGINCHAINLEN
    in length
    */

    if (extractGcodeValue(gcodeLine, 'L', 0)){
        //measure out the left chain
        Serial.println(F("Measuring out left chain"));
        singleAxisMove(&leftAxis, sysSettings.originalChainLength, (sysSettings.maxFeed * .9));

        Serial.print(leftAxis.read());
        Serial.println(F("mm"));

        leftAxis.detach();
    }
    else if(extractGcodeValue(gcodeLine, 'R', 0)){
        //measure out the right chain
        Serial.println(F("Measuring out right chain"));
        singleAxisMove(&rightAxis, sysSettings.originalChainLength, (sysSettings.maxFeed * .9));

        Serial.print(rightAxis.read());
        Serial.println(F("mm"));

        rightAxis.detach();
    }

}

void   setupAxes(){
    /*

    Detect the version of the Arduino shield connected, and use the appropriate pins

    This function runs before the serial port is open so the version is not printed here

    */


    int encoder1A;
    int encoder1B;
    int encoder2A;
    int encoder2B;
    int encoder3A;
    int encoder3B;

    int in1;
    int in2;
    int in3;
    int in4;
    int in5;
    int in6;

    int enA;
    int enB;
    int enC;

    int aux1;
    int aux2;
    int aux3;
    int aux4;
    int aux5;
    int aux6;
    int aux7;
    int aux8;
    int aux9;

    //read the pins which indicate the PCB version
    int pcbVersion = getPCBVersion();

    if(pcbVersion == 0){
        //Beta PCB v1.0 Detected
        //MP1 - Right Motor
        encoder1A = 18; // INPUT
        encoder1B = 19; // INPUT
        in1 = 9;        // OUTPUT
        in2 = 8;        // OUTPUT
        enA = 6;        // PWM

        //MP2 - Z-axis
        encoder2A = 2;  // INPUT
        encoder2B = 3;  // INPUT
        in3 = 11;       // OUTPUT
        in4 = 10;       // OUTPUT
        enB = 7;        // PWM

        //MP3 - Left Motor
        encoder3A = 21; // INPUT
        encoder3B = 20; // INPUT
        in5 = 12;       // OUTPUT
        in6 = 13;       // OUTPUT
        enC = 5;        // PWM

        //AUX pins
        aux1 = 17;
        aux2 = 16;
        aux3 = 15;
        aux4 = 14;
        aux5 = 0;        // warning! this is the serial TX line on the Mega2560
        aux6 = 1;        // warning! this is the serial RX line on the Mega2560

        aux7 = -1;       // unconnected
        aux8 = -1;       // unconnected
        aux9 = -1;       // unconnected
    }
    else if(pcbVersion == 1){
        //PCB v1.1 Detected
        //MP1 - Right Motor
        encoder1A = 20; // INPUT
        encoder1B = 21; // INPUT
        in1 = 6;        // OUTPUT
        in2 = 4;        // OUTPUT
        enA = 5;        // PWM

        //MP2 - Z-axis
        encoder2A = 19; // INPUT
        encoder2B = 18; // INPUT
        in3 = 9;        // OUTPUT
        in4 = 7;        // OUTPUT
        enB = 8;        // PWM

        //MP3 - Left Motor
        encoder3A = 2;   // INPUT
        encoder3B = 3;   // INPUT
        in5 = 10;        // OUTPUT
        in6 = 11;        // OUTPUT
        enC = 12;        // PWM

        //AUX pins
        aux1 = 17;
        aux2 = 16;
        aux3 = 15;
        aux4 = 14;
        aux5 = A7;
        aux6 = A6;

        aux7 = -1;       // unconnected
        aux8 = -1;       // unconnected
        aux9 = -1;       // unconnected
    }
    else if(pcbVersion == 2){
        //PCB v1.2 Detected

        //MP1 - Right Motor
        encoder1A = 20;  // INPUT
        encoder1B = 21;  // INPUT
        in1 = 4;         // OUTPUT
        in2 = 6;         // OUTPUT
        enA = 5;         // PWM

        //MP2 - Z-axis
        encoder2A = 19;  // INPUT
        encoder2B = 18;  // INPUT
        in3 = 7;         // OUTPUT
        in4 = 9;         // OUTPUT
        enB = 8;         // PWM

        //MP3 - Left Motor
        encoder3A = 2;   // INPUT
        encoder3B = 3;   // INPUT
        in5 = 11;        // OUTPUT
        in6 = 12;        // OUTPUT
        enC = 10;        // PWM

        //AUX pins
        aux1 = 17;
        aux2 = 16;
        aux3 = 15;
        aux4 = 14;
        aux5 = A7;
        aux6 = A6;

        aux7 = -1;       // unconnected
        aux8 = -1;       // unconnected
        aux9 = -1;       // unconnected
    }
    else if((pcbVersion == 3) || (pcbVersion == 4)) { // TLE5206
        //TLE5206 PCB v1.3 Detected
        //MP1 - Right Motor
        encoder1A = 20; // INPUT
        encoder1B = 21; // INPUT
        in1 = 6;        // OUTPUT
        in2 = 4;        // OUTPUT
        enA = 5;        // errorFlag

        //MP2 - Z-axis
        encoder2A = 19; // INPUT
        encoder2B = 18; // INPUT
        in3 = 7;        // OUTPUT
        in4 = 9;        // OUTPUT
        enB = 8;        // errorFlag

        //MP3 - Left Motor
        encoder3A = 2;   // INPUT
        encoder3B = 3;   // INPUT
        in5 = 10;        // OUTPUT
        in6 = 11;        // OUTPUT
        enC = 12;        // errorFlag

        //AUX pins
        aux1 = 40;
        aux2 = 41;
        aux3 = 42;
        aux4 = 43;
        aux5 = 68;
        aux6 = 69;
        aux7 = 45;
        aux8 = 46;
        aux9 = 47;
    }
    else if(pcbVersion == 5){ // EBS PCB v1.5b Detected
        
        //MP1 - Right Motor
        encoder1A = 2;  // INPUT
        encoder1B = 3;  // INPUT
        in1 = 9;        // OUTPUT
        in2 = 10;       // OUTPUT
      

        //MP2 - Z-axis
        encoder2A = 18; // INPUT
        encoder2B = 19; // INPUT
        in3 = 7;        // OUTPUT
        in4 = 8;        // OUTPUT
       

        //MP3 - Left Motor
        encoder3A = 21; // INPUT
        encoder3B = 20; // INPUT
        in5 = 6;        // OUTPUT
        in6 = 5;        // OUTPUT
       
        //AUX pins
        aux1 = 48;      // Router ON pin->HIGH
        aux2 = 49;      // Probe INPUT
        aux3 = 51;      // NC
        aux4 = 50;      // Laser ON pin->HIGH
        aux5 = 43;      // NC
        aux6 = 44;      // NC
    }
    else if (pcbVersion == 6){ // TLE9201
        //TLE9201 PCB v1.6 Detected
        //MP1 - Right Motor
        encoder1A = 20;  // INPUT
        encoder1B = 21;  // INPUT
        in1 = 4;         // OUTPUT - TLE9201 DIR
        in2 = 5;         // OUTPUT - TLE9201 ENABLE
        enA = 6;         // OUTPUT - TLE9201 PWM
        ENA = enA;

        //MP2 - Z-axis
        encoder2A = 19;  // INPUT
        encoder2B = 18;  // INPUT
        in3 = 7;         // OUTPUT - TLE9201 DIR
        in4 = 8;         // OUTPUT - TLE9201 ENABLE
        enB = 9;         // OUTPUT - TLE9201 PWM
        ENB = enB;

        //MP3 - Left Motor
        encoder3A = 2;   // INPUT
        encoder3B = 3;   // INPUT
        in5 = 11;        // OUTPUT - TLE9201 DIR
        in6 = 12;        // OUTPUT - TLE9201 ENABLE
        enC = 10;        // OUTPUT - TLE9201 PWM
        ENC = enC;

        //AUX pins
        aux1 = 40;
        aux2 = 41;
        aux3 = 42;
        aux4 = 43;
        aux5 = 68;
        aux6 = 69;
        aux7 = 45;
        aux8 = 46;
        aux9 = 47;
    }
	else if (pcbVersion == 8) { // MASLOW TB6643 PCB v1.8
 
       //MP1 - Right Motor
        encoder1A = 20; // INPUT
        encoder1B = 21; // INPUT
        in1 = 6;        // OUTPUT
        in2 = 5;        // OUTPUT

        //MP2 - Z-axis
        encoder2A = 19; // INPUT
        encoder2B = 18; // INPUT
        in3 = 8;        // OUTPUT
        in4 = 7;        // OUTPUT

        //MP3 - Left Motor
        encoder3A = 2;  // INPUT
        encoder3B = 3;  // INPUT
        in5 = 10;       // OUTPUT
        in6 = 9;        // OUTPUT

        //AUX pins
        aux1 = 40;      // SPINDLE PWR OUTPUT (5v = HIGH = ON)
        aux2 = 41;
        aux3 = 42;
        aux4 = 43;      // Z ZERO PROBE INPUT  (5v = HIGH = TOUCH)
        aux5 = 68;
        aux6 = 69;
        aux7 = 45;
        aux8 = 46;
        aux9 = 47;
    }
    else { // board not recognized
        reportAlarmMessage(ALARM_BOARD_VERSION_INVALID);
        // Do we need to assure that no gpio pins are activated?
        encoder1A = 0;
        encoder1B = 0;
        in1 = 0;
        in2 = 0;
        enA = 0;

        //MP2 - Z-axis
        encoder2A = 0;
        encoder2B = 0;
        in3 = 0;
        in4 = 0;
        enB = 0;

        //MP3 - Left Motor
        encoder3A = 0;
        encoder3B = 0;
        in5 = 0;
        in6 = 0;
        enC = 0;

        //AUX pins
        aux1 = 0;
        aux2 = 0;
        aux3 = 0;
        aux4 = 0;
        aux5 = 0;
        aux6 = 0;
        aux7 = 0;
        aux8 = 0;
        aux9 = 0;
    }

    /*
    * Set motor directions - Left and Right must turn in opposite directions, and 
    * the setting chainOverSprocket reverses their directions. The Z motor is not
    * affected by chainOverSprocket.
    * The TLE9201 uses dedicated DIR, PWM and DISable pins,
    * so logic from previous chips won't work. 
    * The direction logic for the TLE9201 is handled in Motor::write()
    */
    if(sysSettings.chainOverSprocket == 1){
        if (!TLE9201) {
            leftAxis.setup (enC, in6, in5, encoder3B, encoder3A, 'L', LOOPINTERVAL);
            rightAxis.setup(enA, in1, in2, encoder1A, encoder1B, 'R', LOOPINTERVAL);
        } 
        else { // TLE9201 
        // TLE9201 values: (pwm, enable, direction, encoderB, encoderA, name, LOOPINTERVAL)
            leftAxis.setup (enC, in5, in6, encoder3B, encoder3A, 'L', LOOPINTERVAL);
            rightAxis.setup(enA, in1, in2, encoder1A, encoder1B, 'R', LOOPINTERVAL);
        }
    }
    else{ // chain Under Sprocket...
        if (!TLE9201) {
            leftAxis.setup (enC, in5, in6, encoder3A, encoder3B, 'L', LOOPINTERVAL);
            rightAxis.setup(enA, in2, in1, encoder1B, encoder1A, 'R', LOOPINTERVAL);
        }
        else {
            leftAxis.setup (enC, in5, in6, encoder3A, encoder3B, 'L', LOOPINTERVAL);
            rightAxis.setup(enA, in1, in2, encoder1B, encoder1A, 'R', LOOPINTERVAL);
        }
    }

    zAxis.setup    (enB, in3, in4, encoder2B, encoder2A, 'Z', LOOPINTERVAL);

    leftAxis.setPIDValues(&sysSettings.KpPos, &sysSettings.KiPos, &sysSettings.KdPos, &sysSettings.propWeightPos, &sysSettings.KpV, &sysSettings.KiV, &sysSettings.KdV, &sysSettings.propWeightV);
    rightAxis.setPIDValues(&sysSettings.KpPos, &sysSettings.KiPos, &sysSettings.KdPos, &sysSettings.propWeightPos, &sysSettings.KpV, &sysSettings.KiV, &sysSettings.KdV, &sysSettings.propWeightV);
    zAxis.setPIDValues(&sysSettings.zKpPos, &sysSettings.zKiPos, &sysSettings.zKdPos, &sysSettings.zPropWeightPos, &sysSettings.zKpV, &sysSettings.zKiV, &sysSettings.zKdV, &sysSettings.zPropWeightV);
    // Assign AUX pins to extern variables used by functions like Spindle and Probe
    if (pcbVersion == 5){
        SpindlePowerControlPin = aux1;   // output for controlling spindle power
        LaserPowerPin = aux4;            // output for controlling a laser diode
        ProbePin = aux2;                 // use this input for zeroing zAxis with G38.2 gcode
    } 
    else {
        SpindlePowerControlPin = aux1;   // output for controlling spindle power
        LaserPowerPin = aux2;            // output for controlling a laser diode
        ProbePin = aux4;                 // use this input for zeroing zAxis with G38.2 gcode
    }
    
   
    pinMode(LaserPowerPin, OUTPUT);
    digitalWrite(LaserPowerPin, LOW);

    // implement the AUXx values that are 'used'. This accomplishes setting their values at runtime.
    // Using these variables in a test permits to avoid warnings like
    //  "warning: variable ‘xxxxx’ set but not used [-Wunused-but-set-variable]"
    //  for AUX pins defined but not connected

    // defined auxX are inputs by default
    if (aux3 > 0) pinMode(aux3,INPUT);
    if (aux5 > 0) pinMode(aux5,INPUT);
    if (aux6 > 0) pinMode(aux6,INPUT);
    if (aux7 > 0) pinMode(aux7,INPUT);
    if (aux8 > 0) pinMode(aux8,INPUT);
    if (aux9 > 0) pinMode(aux9,INPUT);
}

int getPCBVersion(){
/*
*       On the original Maslow L298P boards, 
*     D22-D23 and D52-D53 on XIO were used to 
*     indicate the board revision number in binary. 
*     The software uses INPUT_PULLUP to read these pins 
*     and detect the shield version. TLE5206 boards
*     expanded the XIO pins to allow more board numbers.
*       The value read from the gpio pins is/was 1-based,
*     but the board version number silkscreened on those boards
*     and reported by the firmware was zero-based - a source of
*     confusion when creating new boards. 
*       Beginning with board v1.6, the value from the gpio pins for new boards
*     will be zero-based to match the number reported by the firmware
*     and the silkscreen.
*     
*     
*     "x" = not used
*     #53-#52    #27-#26    #25-#24   #23-#22
*     -------    -------    -------   -------
*     GND GND     PU PU     PU  PU    GND PU  -> rev.0001  PCB v1.0 aka beta release board
*     GND GND     PU PU     PU  PU    PU  GND -> rev.0002  PCB v1.1
*     GND GND     PU PU     PU  PU    PU  PU  -> rev.0003  PCB v1.2
*      x   x      x   x     GND PU    GND GND -> PCB v1.3 and 1.4 TLE5206
*      x   x      GND GND   GND PU    GND PU  -> reserved for v1.4 TLE5206, v1.5 is unused
*      x   x      GND GND   GND PU    PU  GND -> PCB v1.6 TLE9201
*/
    pinMode(VERS1,INPUT_PULLUP);
    pinMode(VERS2,INPUT_PULLUP);
    pinMode(VERS3,INPUT_PULLUP);
    pinMode(VERS4,INPUT_PULLUP);
    pinMode(VERS5,INPUT_PULLUP);
    pinMode(VERS6,INPUT_PULLUP);
    int pinCheck = (32*digitalRead(VERS6) + 16*digitalRead(VERS5) + 8*digitalRead(VERS4) + 4*digitalRead(VERS3) + 2*digitalRead(VERS2) + 1*digitalRead(VERS1));
    switch (pinCheck) {
        // boards v1.1, v1.2, v1.3 don't strap VERS3-6 low
        case B111101: case B111110: case B111111: // v1.1, v1.2, v1.3
            pinCheck &= B000011; // strip off the unstrapped bits
            TLE5206 = false;
            TLE9201 = false;
            TB6643 = false;
            break;
        case B110100: case B000100: // some versions of board v1.4 don't strap VERS5-6 low
            pinCheck &= B000111;    // strip off the unstrapped bits           
            TLE5206 = true;
            TLE9201 = false;
            TB6643 = false; 
            break;
        case B000110: //  v 5
            pinCheck &= B000110; // 110 for 6 
            TLE5206 = false;
            TLE9201 = false;
            TB6643 = true;
            break;
        case B000111:  //v 1.6 AND V 1.7 (1.7 is a place holder and will not exist because 1.6 uses the same pins)
            pinCheck &= B000111;
            TLE5206 = false;
            TLE9201 = true;
            TB6643 = false;
            break;
    	case B001000:  // v 1.8
            pinCheck &= B001000;
            TLE5206 = false;
            TLE9201 = false;
            TB6643 = true;
            break;
	}
    return pinCheck<8 ? pinCheck-1 : pinCheck;
}


//
// PWM frequency change
//  constrain choices to valid values
//  tailor the PWM frequency to less than the driver chip upper limit
//
void setPWMPrescalers(int prescalerChoice) {
    // limit prescalerChoice to valid values 1..3
    prescalerChoice = constrain(abs(prescalerChoice),1,3);
    #if defined (verboseDebug) && verboseDebug > 0
        Serial.print(F("fPWM set to "));
        switch (prescalerChoice) {
            case 1:
                if (TLE5206) {
                    Serial.println(F("490Hz - TLE5206 upper limit"));
                } else if (TLE9201) {
                    Serial.println(F("4,100Hz - TLE9201 upper limit"));
                } else { // L298 works at 31,000Hz
                    Serial.println(F("31,000Hz"));
                }
            break;
            case 2:
                if (TLE5206) {
                    Serial.println(F("490Hz - TLE5206 upper limit"));
                } else {
                    Serial.println(F("4,100Hz"));
                }
            break;
            case 3:
                Serial.println(F("490Hz"));
            }
    #endif
    // tailor the PWM frequency to the chip
    if (TLE5206) {
    // The upper limit to PWM frequency for TLE5206 is 1,000Hz
    //  so only '3' is valid
        prescalerChoice = 3;
    } else if (TLE9201) { 
    // The upper limit to PWM frequency for TLE9201 is 20,000Hz
        prescalerChoice = constrain(prescalerChoice,2,3);
    } else if (TB6643) {
        prescalerChoice = constrain(prescalerChoice,1,3); // upper limit PWM frequency for TB6643 is 100kHz
    }
// first must erase the bits in each TTCRxB register that control the timers prescaler
    int prescalerEraser = 7;      // this is 111 in binary and is used as an eraser
    TCCR2B &= ~prescalerEraser;   // this operation sets the three bits in TCCR2B to 0
    TCCR3B &= ~prescalerEraser;   // this operation sets the three bits in TCCR3B to 0
    TCCR4B &= ~prescalerEraser;   // this operation sets the three bits in TCCR4B to 0
    // now set those same three bits
// ————————————————————————————–
// TIMER 2       (Pin 9, 10)
// Value  Divisor  Frequency
// 0x01   1        31.374 KHz
// 0x02   8        3.921 KHz
// 0x03   32       980.3 Hz        // don;t use this...
// 0x04   64       490.1 Hz        // default
// 0x05   128      245 hz
// 0x06   256      122.5 hz
// 0x07   1024     30.63 hz
// Code:  TCCR2B = (TCCR2B & 0xF8) | value ;
// —————————————————————————————-
// Timers 3, 4 ( Pin 2, 3, 5), (Pin 6, 7, 8)
//
// Value  Divisor  Frequency
// 0x01   1        31.374 KHz
// 0x02   8        3.921 Khz
// 0x03   64       490.1 Hz        // default
// 0x04   256      122.5 Hz
// 0x05   1024     30.63 Hz
// Code:  TCCR3B = (TCCR3B & 0xF8) | value ;
// —————————————————————————————-
    // and apply it
    if (prescalerChoice >= 3) {
      TCCR2B |= (prescalerChoice + 1); // pins 9, 10 - change to match timers3&4
    } else {
      TCCR2B |= prescalerChoice; // pins 9, 10
      }
    TCCR3B |= prescalerChoice;   // pins 2, 3, 5
    TCCR4B |= prescalerChoice;   // pins 6, 7, 8
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

    bit_true(sys.pause, PAUSE_FLAG_USER_PAUSE);
    Serial.println(F("Maslow Paused"));

    while(bit_istrue(sys.pause, PAUSE_FLAG_USER_PAUSE)) {

        // Run realtime commands
        execSystemRealtime();
        if (sys.stop){return;}
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
    systemSaveAxesPosition();
    motionDetachIfIdle();
    // check systemRtExecAlarm flag and do stuff
}

void systemSaveAxesPosition(){
    /*
    Save steps of axes to EEPROM if they are all detached
    */
    if (sys.writeStepsToEEPROM && !leftAxis.attached() && !rightAxis.attached() && !zAxis.attached()){
        settingsSaveStepstoEEprom();
    }
}

void systemReset(){
    /*
    Stops everything and resets the arduino
    */
    leftAxis.detach();
    rightAxis.detach();
    zAxis.detach();
    setSpindlePower(false);
    laserOff();
    // Reruns the initial setup function and calls stop to re-init state
    sys.stop = true;
    setup();
}

byte systemExecuteCmdstring(String& cmdString){
    /*
    This function processes the $ system commands

    This is taken heavily from grbl.  https://github.com/grbl/grbl
    */
    byte char_counter = 1;
//    byte helper_var = 0; // Helper variable
    float parameter, value;
    if (cmdString.length() == 1){
        reportMaslowHelp();
    }
    else {
        switch( cmdString[char_counter] ) {
          case '$': // case 'G': case 'C': case 'X':
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
            break;
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
          default :
            // Block any system command that requires the state as IDLE/ALARM. (i.e. EEPROM, homing)
            // if ( !(sys.state == STATE_IDLE || sys.state == STATE_ALARM) ) { return(STATUS_IDLE_ERROR); }
            switch( cmdString[char_counter] ) {
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
              case 'R' : // Restore defaults [IDLE/ALARM]
                if (cmdString[++char_counter] != 'S') { return(STATUS_INVALID_STATEMENT); }
                if (cmdString[++char_counter] != 'T') { return(STATUS_INVALID_STATEMENT); }
                if (cmdString[++char_counter] != '=') { return(STATUS_INVALID_STATEMENT); }
                if (cmdString.length() != 6) { return(STATUS_INVALID_STATEMENT); }
                switch (cmdString[++char_counter]) {
                  case '$': settingsWipe(SETTINGS_RESTORE_SETTINGS); break;
                  case '#': settingsWipe(SETTINGS_RESTORE_MASLOW); break;
                  case '*': settingsWipe(SETTINGS_RESTORE_ALL); break;
                  default: return(STATUS_INVALID_STATEMENT);
                }
                reportFeedbackMessage(MESSAGE_RESTORE_DEFAULTS);
                systemReset(); // Force reset to ensure settings are initialized correctly.
                break;
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
              default :  // Storing setting methods [IDLE/ALARM]
                if(!readFloat(cmdString, char_counter, parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
                if(cmdString[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
                // if (helper_var) { // Store startup line
                //   // Prepare sending gcode block to gcode parser by shifting all characters
                //   helper_var = char_counter; // Set helper variable as counter to start of gcode block
                //   do {
                //     line[char_counter-helper_var] = line[char_counter];
                //   } while (line[char_counter++] != 0);
                //   // Execute gcode block to ensure block is valid.
                //   helper_var = gc_execute_line(line); // Set helper_var to returned status code.
                //   if (helper_var) { return(helper_var); }
                //   else {
                //     helper_var = trunc(parameter); // Set helper_var to int value of parameter
                //     settings_store_startup_line(helper_var,line);
                //   }
                // } else { // Store global setting.
                  if(!readFloat(cmdString, char_counter, value)) { return(STATUS_BAD_NUMBER_FORMAT); }
                  if((cmdString[char_counter] != 0) || (parameter > 255)) { return(STATUS_INVALID_STATEMENT); }
                  return(settingsStoreGlobalSetting((byte)parameter, value));
                // }
            }
        }
    }
    return(STATUS_OK);
}

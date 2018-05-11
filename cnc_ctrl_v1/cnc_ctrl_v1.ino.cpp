# 1 "/var/folders/1y/91bw_rsx4fdbc4yzb1wpkhvw0000gp/T/tmphLYhgy"
#include <Arduino.h>
# 1 "/Users/scottsmi/Documents/GitHub/blurfl/Firmware/cnc_ctrl_v1/cnc_ctrl_v1.ino"
# 36 "/Users/scottsmi/Documents/GitHub/blurfl/Firmware/cnc_ctrl_v1/cnc_ctrl_v1.ino"
#include "Maslow.h"


system_t sys;


settings_t sysSettings;


byte systemRtExecAlarm;


Axis leftAxis;
Axis rightAxis;
Axis zAxis;



Kinematics kinematics;
void setup();
void runsOnATimer();
void loop();
#line 56 "/Users/scottsmi/Documents/GitHub/blurfl/Firmware/cnc_ctrl_v1/cnc_ctrl_v1.ino"
void setup(){
    Serial.begin(57600);
    Serial.print(F("PCB v1."));
    Serial.print(getPCBVersion());
    if (TLE5206 == true) { Serial.print(F(" TLE5206 ")); }
    Serial.println(F(" Detected"));
  Serial.println(F("TLE5206-release firmware"));
    sys.inchesToMMConversion = 1;
    settingsLoadFromEEprom();
    setupAxes();
    settingsLoadStepsFromEEprom();

    leftAxis.write(leftAxis.read());
    rightAxis.write(rightAxis.read());
    zAxis.write(zAxis.read());
    readyCommandString.reserve(INCBUFFERLENGTH);
    gcodeLine.reserve(INCBUFFERLENGTH);

    #ifndef SIMAVR

    Timer1.initialize(LOOPINTERVAL);
    Timer1.attachInterrupt(runsOnATimer);
    #endif

    Serial.println(F("Grbl v1.00"));
    Serial.println(F("ready"));
    reportStatusMessage(STATUS_OK);

}

void runsOnATimer(){
    #if misloopDebug > 0
    if (inMovementLoop && !movementUpdated){
        movementFail = true;
    }
    #endif
    movementUpdated = false;
    leftAxis.computePID();
    rightAxis.computePID();
    zAxis.computePID();
}

void loop(){

    initGCode();
    if (sys.stop){
        initMotion();
        setSpindlePower(false);
    }

    kinematics.init();


    sys.stop = false;


    while (!sys.stop){
        gcodeExecuteLoop();
        #ifdef SIMAVR
        runsOnATimer();
        #endif
        execSystemRealtime();
    }
}
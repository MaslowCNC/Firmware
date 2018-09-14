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

// EEPROM addresses 300 and up can be used by Maslow.  Under 300 was used
// previously by pre v1.00 Firmware.

#include "Maslow.h"
#include <EEPROM.h>

void initializeCalibration(){
    /*
    initializes the data to zero
    */
    Serial.print(F(" Zeroing Calibration\r\n"));
    for (int x=0; x<31; x++){
      for (int y=0; y<15; y++){
        calibration.xError[x][y]=0;
        calibration.yError[x][y]=0;
      }
    }
}

byte calibrationUpdateMatrix(int _x, int _y, int xValue, int yValue) {
  if ( (_x==31) && (_y==15) )
  {
    settingsSaveToEEprom();
    return(STATUS_OK);
  }
  else
  {
    if ( (_x>=0) && (_x<31) ) {
      if ( (_y>=0) && (_y<15) ){
          calibration.xError[_x][_y]=xValue;
          calibration.yError[_x][_y]=yValue;
          if ((_x==15) && (_y==7)){
            kinematics.init();
          }
          return(STATUS_OK);
      }
    }
    return(STATUS_INVALID_STATEMENT);
  }
}

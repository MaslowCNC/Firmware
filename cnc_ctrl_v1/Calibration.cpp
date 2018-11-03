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

#include "Maslow.h"
#include <EEPROM.h>

void initializeCalibration() {
  Serial.print(F("Zeroing Calibration\r\n"));
  for (int x=0; x<31; x++) {
    for (int y=0; y<15; y++) {
      calibration.xError[x][y] = 0;
      calibration.yError[x][y] = 0;
    }
  }
}

byte calibrationUpdateMatrix(int x, int y, int xValue, int yValue) {
  if ((x==31) && (y==15)) {
    settingsSaveToEEprom();
    return(STATUS_OK);
  } else {
    if (((x>=0) && (x<31)) && ((y>=0) && (y<15))) {
      calibration.xError[x][y] = xValue;
      calibration.yError[x][y] = yValue;
      if ((x==15) && (y==7)) {
        kinematics.init();
      }
      return(STATUS_OK);
    }
    return(STATUS_INVALID_STATEMENT);
  }
}

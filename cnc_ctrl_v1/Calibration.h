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

#ifndef calibration_h
#define calibration_h

typedef struct {
  int xError[31][15]; // these are ints to save memory.  each represent 0.001 mm so maximum error is +/~32 mm which is crazy high still
  int yError[31][15]; // these are ints to save memory.  each represent 0.001 mm so maximum error is +/~32 mm which is crazy high still
  //float leftLength[31][15]; //Need more memory
  //float rightLength[31][15]; //Need more memory
} calibration_t;
extern calibration_t calibration;


void initializeCalibration();
byte calibrationUpdateMatrix(const int, const int, const int, const int);

#endif

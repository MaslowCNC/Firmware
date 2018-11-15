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

#ifndef calibration_h
#define calibration_h

typedef struct {
  // These are int to save memory. Each represents 0.001 mm so maximum error is +/~32 mm
  int xError[31][15];
  int yError[31][15];
} calibration_t;
extern calibration_t calibration;

/*
initializes the data to zero
*/
void initializeCalibration();

byte calibrationUpdateMatrix(const int x, const int y, const int xValue, const int yValue);

#endif

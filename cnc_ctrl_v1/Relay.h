    /*This file is part of the Makesmith Control Software.

    The Makesmith Control Software is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Makesmith Control Software is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the Makesmith Control Software.  If not, see <http://www.gnu.org/licenses/>.

        Copyright 2014 Andrew Albinger*/

   /* Relay.h provides a class for handling a relay attached to an arduino pin.
      Provided functions are:
	void Relay(int) - constructor 
	void Relay.enable() - energize the relay
	void Relay.disable() - de-energize the relay
	int Relay.state() - return the current state of the relay.
   */
#ifndef Relay_h
#define Relay_h

#include "Arduino.h"

class Relay
{
  public:
    Relay(int pin);
    void enable();
    void disable();
    int state();
  private:
    int _pin;
};

#endif





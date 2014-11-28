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

#include "Relay.h"

Relay::Relay(int pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);  // Start out with relays disabled
  _pin = pin;
}

void Relay::enable()
{
  digitalWrite(_pin, HIGH);
}

void Relay::disable()
{
  digitalWrite(_pin, LOW);
}

int Relay::state()
{
  int _state = digitalRead(_pin);
  return(_state); 
}

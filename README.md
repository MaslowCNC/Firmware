MakesmithFirmware
=================

Makesmith Firmware

This is the firmware for the Makesmith CNC Router

It is intended to run on an Arduino Mega

It consists of fifteen functions outside the Main() loop. They are:

SetScreen() updates the position displayed on the LCD display (yes, we're working on adding an LCD display upgrade.

The SetPos() function updates the machine's position by essentially integrating the input from the encoder

BoostLimit sets the upper and lower bounds of the signals which go to the servos to prevent weird behavior. Valid input to set the servo speed ranges from 0-180, and the Arduino servo library gives strange results if you go outside those limits.

SetSpeed() takes a position and a target and sets the speed of the servo to hit that target. Right now it implements a proportional controller, where the gain is set by the 'gain' input. A PID controller would be better.

The SetTarget() function moves the machine to the position stored in the location structure.

The Unstick() function is called to attempt to unstick the machine when it becomes stuck. 

The Move() function moves the tool in a straight line to the position (xEnd, yEnd, zEnd) at the speed moveSpeed. Movements are correlated so that regardless of the distances moved in each direction, the tool moves to the target in a straight line. This function is used by the G00 and G01 commands.

g1go() is the function which is called to process the string if it begins with 'G01' or 'G00'

Circle two takes in the radius of the circle to be cut and the starting and ending points in radians with pi removed so a complete circle is from 0 to 2. If direction is 1 the function cuts a CCW circle, and -1 cuts a CW circle. The direction that one moves from zero changes between the two directions, meaning that a quarter circle is always given by 0,.5 regardless of the direction. So direction = 1 start = 0 end = .5 makes a 1/4 circle downward and direction = 1 start = 0 end = .5 makes a 1/4 circle upward starting from the right side of the circle

g2go() is the function which is called when the string sent to the machine is 'G02' or 'G03'. The string is parsed to extract the relevant information which is then used to compute the start and end points of the circle and the the circle() function is called.


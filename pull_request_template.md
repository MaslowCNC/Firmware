Thanks for contributing to The Maslow Firmware! You rock.

Please let the community know some basic information about this pull request.

## What does this pull request do?
Does it add a new feature or fix a bug?

This is a new feature. I have made some small tweaks to the code to be able to set upper and lower limits for the z axis in order to prevent motors from forcing the Z Axis beyond its limits.

I have added two variables to settings
float zAxisUpperLimit
float zAxisLowerLimit

I have created 4 "B" commands
B17 Sets the z axis upper limit to the current z axis position
B18 Sets the z axis lower limit to the current z axis position
B19 Clears z axis limits
B20 Echoes back z axis limits

## Does this firmware change affect kinematics or any part of the calibration process?

It does not affect Kinematics. And I have kept it apart from calibration as some users might want to adjust this on the fly as part of the regular use.

### a) If so, does this change require recalibration?

It does not require recalibration. However if the user wishes to establish some limits, he/she must set the limits using gcode commands B17 (Upper) and B18 (Lower) 

### b) If so, is there an option for user to opt-out of the change until ready for recalibration? If not, explain why this is not possible.

By default this limits are set to NAN and will be ignored until specified.

### c) Has the calibration model in gc/hc/wc been updated to agree with firmware change?

No recalibration needed. I intend to update webcontrol to be able to set the limits within the user interface. Meanwhile to make use of it you can send gcode directly.

### d) Has this PR been tested on actual machine and/or in fake servo mode (indicate which or both)?

It has been tested on an actual machine only

## How can this pull request be tested?
Please provide detailed steps about how to test this pull request.

1 - Move the Z axis to the highest point you with to set as a limit.
2 - Send a B17 gcode command.
3 - Move the Z axis to the lowest point you with to set as a limit.
4 - Send a B18 gcode command.
5 - Now attempt to move the Z axis beyond those limits. It should prevent you from exceeding the limits. 
6 - Send the B20 gcode command and observe the limits echoed back
7 - Now attempt to reset the 0 location for the z axis
8 - Send the B20 gcode command and observe the limits echoed back, they should have been updated to account for the new 0 location
9 - Send the B19 gcode command to erase the limits 
10 - Send the B20 gcode command and observe the limits echoes back, the should have been cleared.
11 - Now attempt to move the Z axis beyond the limits set in steps 1 and 2. It should allow you to move freely. (Careful not to exceed the physical limits of your machine)

Thanks for contributing!






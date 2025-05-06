# Introduction to TMux (Terminal Multiplexer)

We are using a TMux terminal to automate the plethora of terminals we have to start. Feel free to share this with other teams.

## Steps to start
1. Start computer and control-pc machines. Arm should have flashing yellow lights.
2. Navigate to `~/ws_bumpkin/bumpkin`
3. Run `./tmux_bumpkin`
   - This should start a new terminal with a green bar at the bottom along with a new google-chrome window
   - The green bar at the bottom will have `n: window-name`

To navigate to any of the terminals open in the green bar, press `Ctrl + b` then press the number `n`. This will open that one.

When working in code and if you want to split the terminal vertically or horizontally,
1. Press `Ctrl+b` to enter TMux "command mode"
   1. Press `shift + \` to split vertically
   2. Press `-` to split horizontally

## Robot died, I need to restart the control PC instance, what now?
1. Press `Ctrl+b` then `2` or whichever window control-pc is on
2. Close the 3 terminals the script opens
3. Ensure other conditions are met
   - The EStop is not pressed (light is blue)
   - Arm is not locked (guide mode can move the robot)
   - Workspace is not blocked and arm can move freely
4. Press up arrow and restart. 

## Arm is locked (guide mode can NOT move the robot)
- Press Estop so lights turn white
- Press the two buttons on the "wrist?" and move the robot to a not-locked position
- Follow `Robot died, I need to restart the control PC instance, what now?`

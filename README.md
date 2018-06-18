# EyeOfTheCrane
Hack a hydraulic crane to automagically align to walls for Techfest Munich 2018

Winner of [Hawe](https://www.hawe.com) Track, 2nd place overall at [Techfest Munich](https://techfestmunich.com/) 2018

## Idea
**Problem**:  
Window cleaners and other users of hydraulic cranes need to be skilled operators and think and control in  spherical coordinates.

**Solution**:
 1. Interactively choose a window to move the crane's basket to in a 3D scene
 2. Have the basket stay aligned to the wall at all times, even if the wall curves away
 3. Replace the machine-centered spherical control system with an intuitive wall-aligned left/right/up/down control system
 
**Implementation**:
 1. Scan the crane's environment with a pan-tilt mounted lidar
 2. Calculate wall position
 3. Calculate and send hydraulics commands via Wifi to computer connected via serial to Arduino board
 4. Fake crane remote commands to make hydraulics move

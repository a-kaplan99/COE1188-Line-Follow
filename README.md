# COE1188-Line-Follow
A group project for Cyberphysical Design. We programmed a robot equipped with an MSP432 Microcontroller to follow a line using reflective LED's.

Below is a list and brief description of relevant files.

JackiFSMmain.c - the main control loop that runs until the robot is turned off

Bump.c - detects phyisical activation of the robot's bump sensors using edge-triggered interrupts

Motor.c - controls for the motors using pulse width modulation input

Reflectance.c - monitors the robot's LED sensors using System Timer interrupts

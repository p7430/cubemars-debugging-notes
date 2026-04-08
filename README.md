# Cubemars Debugging Notes
Getting these CubeMars AK60-6 and AK70-10 actuators running was pretty challenging, so I put together a brief write up showing some of my code, tips, and hardware to help others. I did this a few months ago and posted about it on Reddit and since then many people have been asking me about for a guide.

## My Hardware
To get the motors running with this setup, you're going to need the following:
1. Any microcontroller with a CAN bus. I used the NVIDIA Jetson Orin Nano (Super Developer Kit, from Amazon). It's $250 as of 2026, built for AI inference, and has served me very well, but it's not necessary.
2. CAN transceiver module. I used the SN65HVD230 CAN transceiver board (3.3V). 
3. The RUBIK LINK V2.0. There are multiple version of the Rubik Link, so just make sure the one you're buying is compatible with the motors you have. 
4. A 120 Ω resistor.

## Overview
First, I'll go over how to set everything up, then how to use the CubeMars Software (called "Upper Computer"), and finally how to actually control the motors.
### 1. Part One: Physical setup
<img width="551" height="323" alt="Screenshot 2026-04-07 at 10 22 55 PM" src="https://github.com/user-attachments/assets/a11f4e40-a527-427a-8fae-942e91df7fb2" />

### 2. Part Two: Using Upper Computer to Set Motor Mode to MIT
### 3. Part Three: Motor Control and Software

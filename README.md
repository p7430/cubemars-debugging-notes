## cubemars-debugging-notes
Getting these CubeMars AK60-6 and AK70-10 actuators running was pretty challenging, so I put together some of my code, tips, and hardware to help others. I did this a few months ago and posted about it on Reddit and since then many people have been asking me about for a guide.

## My Hardware
To get the motors running with this setup, you're going to need the following:
1. Any microcontroller with a CAN bus. I used the NVIDIA Jetson Orin Nano (Super Developer Kit, from Amazon). It's $250 as of 2026, built for AI inference, and has served me very well, but it's not necessary.
2. CAN transceiver module. I used the SN65HVD230 CAN transceiver board (3.3V). 
3. The RUBIK LINK V2.0. There are multiple version of the Rubik Link, so just make sure the one you're buying is compatible with the motors you have. 
4. A 120 Ω resistor.

## Overview
This guide will be structured as such:
# 1. Part One: Physical setup
# 2. Part Two: Setting Motor Mode to MIT
# 3. Part Three: Motor Control and Software

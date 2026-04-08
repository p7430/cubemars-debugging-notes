# Cubemars Debugging Notes
Getting my AK60-6 and AK70-10 actuators running was challenging (extremely), so I put together a brief write up showing some of my code, tips, and hardware to help others. I did this a few months ago and posted about it on Reddit and since then many people have been asking me for a guide.

## My Hardware
To get the motors running with this setup, I used the following:
1. A microcontroller with a CAN bus. I used the NVIDIA Jetson Orin Nano (Super Developer Kit, from Amazon). It's $250 as of 2026, built for AI inference, and has served me very well, but it's not necessary.
2. CAN transceiver module. I used the SN65HVD230 CAN transceiver board (3.3V). 
3. The RUBIK LINK V2.0. There are multiple version of the Rubik Link, so just make sure the one you're buying is compatible with the motors you have. 
4. A 120 Ω resistor.
5. A power supply. I used the MEAN WELL RSP-500-24 AC-DC Switching Enclosed Power Supply.

## Prelude: Motivations
### 1. Why MIT Mode?
You may be wondering: why shift to the seemingly more complex MIT Mode when you can just run it in Servo mode? A few reasons:
- I found Servo mode to be too "dumb" for my application (a robot arm). In real robotics environments, you will actually want more granular control when  rotating your actuators. To learn more, watch: https://www.youtube.com/watch?v=0ZqeBEa_MWo&t=13s. BTW, this textbook is great for robotics and I believe it's online for free. If you're intimidated, don't worry, I'll provide the code for this fifth-derivative trajectory generation. 
- Servo mode seemed to introduce some level of latency as I controlled the actuator which made the robot arm I'm building have a jittery motion. MIT Mode, on the other hand, seems to have far less latency.
### 2. Can't you just buy the motors with MIT mode pre-installed?
Yes, actually, you can. And if I had known about that I would have done this. If you haven't bought them yet and plan on using MIT Mode, I believe you can email CubeMars to do this for you when you buy them. However, even if you buy the motors with MIT mode pre-installed, you'll still need the upper computer to define the "names" of your motors so you can control them with CAN Bus.

## Overview
First, I'll go over how to set everything up, then how to use the CubeMars Software (called "Upper Computer"), and finally how to actually control the motors.
### 1. Part One: Physical setup for using the Upper Computer
<img width="551" height="323" alt="Screenshot 2026-04-07 at 10 22 55 PM" src="https://github.com/user-attachments/assets/a11f4e40-a527-427a-8fae-942e91df7fb2" />

a. Power your motors and check that they turn on. The motor should light up where the wires go into the motor.
b. Connect your R-Link to your computer. It will be USB cable --> R-link --> motor. All necessary cables should come with the R-Link. 

### 2. Part Two: Using Upper Computer to Set Motor Mode to MIT
The Upper Computer software only works on Windows, so if you plan on running the Upper Computer on a Mac, you're going to want to get Parallels Desktop or a similar Windows emulator.

You can find the download link for the software at https://www.cubemars.com/article.php?id=261.

### 3. Part Three: Motor Control and Software


## Some word of caution & a disclaimer:
1. The AK series motors are pretty powerful, so when you start testing them, I HIGHLY recommend not having anything attached to them as they may move unpredictably. When I first started experimenting with these I naively did have some stuff attached to one (a metal rod, essentially). Now I have a dent in my wall to show for it.
2. This is meant for educational purposes, don't sue me, this is not meant to be advice. 



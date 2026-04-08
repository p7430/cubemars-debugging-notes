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
- You get more feedback/data using a CAN Bus. 
### 2. Can't you just buy the motors with MIT mode pre-installed?
Yes, actually, you can. Just email CubeMars to send you one with MIT mode preinstalled when you buy them. However, even if you buy the motors with MIT mode pre-installed, you'll still need the upper computer to define the "names" (CAN_ID) of your motors so you can control them over CAN Bus.

## Overview
First, I'll go over how to set everything up, then how to use the CubeMars Software (called "Upper Computer"), and finally how to actually control the motors.
### 1. Part One: Physical setup for using the Upper Computer
<img width="551" height="323" alt="Screenshot 2026-04-07 at 10 22 55 PM" src="https://github.com/user-attachments/assets/a11f4e40-a527-427a-8fae-942e91df7fb2" />

This video shows how to connect everything: https://www.youtube.com/watch?v=JjgukaBcMKM

- Power your motors and check that they turn on. The motor should light up where the wires go into the motor when powered.
- Connect your R-Link to your computer. It will be USB cable --> R-link --> motor. All necessary cables should come with the R-Link. 

### 2. Part Two: Using Upper Computer to Set Motor Mode to MIT
The Upper Computer software only works on Windows, so if you plan on running the Upper Computer on a Mac, you're going to want to get Parallels Desktop or a similar Windows emulator.

I won't spend too much time here because I don't have a Parallels Desktop subscription anymore so I can't actually use the Upper Computer. However, there's already a good amount of online content to point you in the right direction, which I'll link below. 

Generally speaking, you will want to switch to MIT mode if you're in Servo mode (there's a tab for this called "Mode Switch" and you just click the MIT button), calibrate the motor, then "name" your motor by giving it a CAN Bus ID. Side note, if I'm remembering correctly, to calibrate and name your motor, you will need to click "Debug" in MIT mode, which will open a terminal where you type commands like "calibrate".

#### Here are some videos that give an overview:
This is the most detailed explanation I've found so far. It walks you through how to switch to MIT Mode (however I believe he skips the calibration stage): https://www.youtube.com/watch?v=5KYqYAnAdCM

Official CubeMars Youtube Page Upper Computer Introduction (more general): https://www.youtube.com/watch?v=6X0y0op3MJo

You can find the download link for the software at https://www.cubemars.com/article.php?id=261.

### 3. Part Three: Motor Control and Software
Now that the actuator(s) are in MIT mode and have CAN IDs, you can actually control them. 
1. Start by disconnecting them from the R-Link
2. Conenct them to your controller. I'm using a Jetson Orin Nano, so I attached my CAN Bus Transceiver to the Jetson Orin Nano, and then added a terminating resistor connecting CAN High and CAN Low. If you're unfamiliar with how CAN Bus actually works, I highly recommend watching some videos to understand the setup:
- https://www.youtube.com/watch?v=JZSCzRT9TTo
- Here's a diagram of what your setup should look like with one or multiple motors (the diagram says two but you would keep chaining motors as pictured):
<img width="679" height="389" alt="Screenshot 2026-04-08 at 12 23 25 AM" src="https://github.com/user-attachments/assets/0be1bd37-7f9e-4791-8a67-a6518f3aeb4d" />
<img width="522" height="265" alt="Screenshot 2026-04-08 at 12 24 12 AM" src="https://github.com/user-attachments/assets/51cbcf90-4410-4e8e-a0b2-34f4bcfc0d40" />

I have attached code to the repo (ROS2) that I use for my robot arm that does the following:
#### 1. mit_motor_driver.py
- Motor startup code. Essentially, it 'deactivates' the motors so they can't move (and therefore can't do any abrupt movements before being calibrated), calibrates them by setting the origin to the current position, and then 'reactivates' them. This prevents them from jerking to a random position, which could be dangerous on startup.
- A  lower level controller. This takes the trajectory generated by the planner and executes it.
#### 2. trajectory_controller.py
- A higher level controller. It's a point to point fifth-derivative trajectory planner. This is important because if you tell the motor to move to, say 10°, it will move there at full speed, which is... dangerous. On the other hand, the planner smoothes the motion and makes it safe.
- I didn't include inverse kinematics code to calculate the desired joint state because each robot is different. 

### Some words of caution & a disclaimer:
1. The AK series motors are pretty powerful, so when you start testing them, I HIGHLY recommend not having anything attached to them as they may move unpredictably. When I first started experimenting with these I naively did have some stuff attached to one (a metal rod, essentially). Now I have a dent in my wall to show for it.
2. This is meant for educational purposes only, it is not meant to be advice. 



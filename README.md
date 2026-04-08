# cubemars-debugging-notes
Getting these CubeMars AK60-6 and AK70-10 actuators running was pretty challenging, so I put together some of my code, tips, and hardware to help others. 

# My Hardware
To get the motors running with this setup, you're going to need the following:
1. NVIDIA Jetson Orin Nano Super Developer Kit (or really any microcontroller with access to a CAN bus). The Jetson Orin Nano was about $250, built for AI inference, and has served me very well, but it's not necessary.
2. CAN transceiver module. I used the SN65HVD230 CAN transceiver board (3.3V). The links are below.
3. The RUBIK LINK V2.0. There are multiple version of the Rubik Link, so just make sure the one you're buying is compatible with the motors you have. 
4. A 120 Ω resistor.

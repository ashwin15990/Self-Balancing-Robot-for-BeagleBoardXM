*Instructions

To compile the attached self balancing robot source code use the following command:

$gcc -o <executable> selfBalance_main.c -lm -lpthread

This is needed to create an executable to run the code on Beagleboard XM 

The project files are
1) i2c_dev.h
2) selfBalance_main.c

All of the drivers are witten in selfBalance_main.c. This file contains drivers for
1) Accelerometer
2) Gyroscope 
3) DC Motor controller

All three are controlled using the i2c interface. 

The main function of selfBalance_main.c spawns 3 threads for each of the three services and the sceduler for the same is written in the main function of the file


i2c_dev.h is just an include file which will be present in your bin folder if i2c_tools is installed in your system.

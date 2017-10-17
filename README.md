# **PID Control Project**

**Udacity Self Driving Car Nanodegree - Project #9**

2017/10/16

## Overview

This project implements a **PID control** algorithm in C++ to control the steering while driving a simulated vehicle around a track using feedback on the measured lateral **Cross-Track Error (CTE)**.

Besides implementing the basic **Proportional/Integral/Derivative error terms**, the PID controller includes some **additional features** such as an integral windup min/max guard, D term latching, smoothing, and min/max guard to prevent spikes from CTE discontinuities, and rate limiting on final output to smooth out the steering movement.

The PID control gains were **initially tuned manually** to explore their effects.  After base gains were chosen at the reference speed (30% throttle), further tuning adjustment was done automatically using a **"twiddle" (coordinate ascent) algorithm** to optimize an error function based on combining accumulated CTE and steering work.

## Project Reflection

For more details about the results of this tuning activity, see the [project reflection document](Reflection.md).

## Key Files

| File              | Description                                                                                                    |
|:-----------------:|:--------------------------------------------------------------------------------------------------------------:|
| /src/main.cpp     | Source code for **main loop** that handles **uWebSockets communication to simulator**                          |
| /src/PID.cpp, .h  | Source code for **PID control algorithm** that controls the steering value based on feedback from measured CTE |
| /build/pid        | Output **executable program binary**                                                                           |
| [Reflection.md](Reflection.md)     | **Reflection document** describing the PID gain tuning activity                                                |
| install-mac.sh    | Script for Mac to install uWebSocketIO required to interface with simulator                                    |
| install-ubuntu.sh | Script for Linux to install uWebSocketIO required to interface with simulator                                  |

The original Udacity project repository is [here](https://github.com/udacity/CarND-PID-Control-Project).

## How to Build and Run Code

This project involves the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two scripts (**install-mac.sh** and **install-ubuntu.sh**) that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./pid

If using Xcode to build, run the following commands:

1. mkdir xbuild
2. cd xbuild
3. cmake -G "Xcode" ..
4. Open "PID.xcodeproj" in Xcode and build
5. cd Debug
6. ./pid

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Communication protocol between uWebSocketIO and Simulator

**INPUT to main.cpp**: values provided by the simulator to the C++ program

* ["cte"] => Measured lateral Cross-Track Error
* ["speed"] => Vehicle's last speed (mph)
* ["steering_angle"] => Vehicle's last steering angle (-1 to +1)

**OUTPUT from main.cpp**: values provided by the C++ program to the simulator

* ["steering_angle"] <= Control value for steering angle (-1 to +1)
* ["throttle"] <= Control value for throttle (0 to 1)

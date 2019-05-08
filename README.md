# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

### Important:
```
In addition to udacity provided CMake build targets, a GoogleTest and GoogleMocks 
based unit test configuration is also added. For this CMakeLists.txt is modified 
to download GoogleTest as part of the build's configure step. So please make sure 
that you have internet connectivity during cmake step above.
```

Tips for setting up your environment can be found in the classroom lesson for the EKF project.

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

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
* python2.7 and matplotlib
  * Linux/Windows: Ideally you should have these installed if you had already installed [CarND Term1 Starter Kit](https://github.com/udacity/CarND-Term1-Starter-Kit)
* cucumber and cucumber-cpp (Optional)
  * Linux/Windows: Please refer to [this document](install_cucumber.md) for installation instructions.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF`

## Running Unit Tests

A unit test suite is also compiled as part of above build step. You can simply run it with following command from build directory

`./UnscentedKFTests`

## Running Behavior-driven Scenario Specifications (Linux and Windows Only)

A cucumber-cpp based behaviour-driven feature test suite is also included with this. These are however not built by default and must be manually enabled from cmake with following commands from build directory. (You'll need to install additional dependancies with `install-cucumber-ubuntu.sh`. This is currently only supported on Linux and Windows using Ubuntu on WSL)

1. Run CMAKE with additional argument: `cmake -DUKF_ENABLE_CUKE=on ..`
2. Compile: `make`

Then you can simply run bdd feature-tests with following command from build directory

`./UnscentedKFSteps >/dev/null & cucumber ..`

## Supported command line arguments

Following command line arguments are supported by UnscentedKF.

```
usage: ./UnscentedKF [--twiddle|--lidar-only|--radar-only]
  options:
    --twiddle     Twiddle parameter tuning
    --lidar-only  Use Lidar only for measurement updates
    --radar-only  Use Radar only for measurement updates
```

Please note that you can specify only one of these arguments at once.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see the project page in the classroom
for instructions and the project rubric.

## Python Version of ExtendedKF

* Python version of this project is available under /python directory. Please refer to [/python/README.md](python/README.md) for more details on using python version of the project.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).


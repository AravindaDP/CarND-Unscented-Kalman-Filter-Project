# Unscented Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

### Dependencies
This package requires:

* [CarND Term1 Starter Kit](https://github.com/udacity/CarND-Term1-Starter-Kit)

The lab enviroment can be created with CarND Term1 Starter Kit. Click [here](https://github.com/udacity/CarND-Term1-Starter-Kit/blob/master/README.md) for the details.

Once above dependencies are installed, main program can be built and run by doing the following from the directory containing this README.

```
python unscented_kf.py
```

Note that the programs that need to be written to accomplish the project are ukf/ukf.py, and ukf/tools.py

The program unscented_kf.py has already been filled out, but feel free to modify it.

Here is the main protocol that unscented_kf.py uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the python program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the python program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* unittest
  * In addition to python libraries included in Term1 Starter Kit, unittest based test package is included in this. To successfully run these tests python unittest package is also required.

* behave
  * In addition to test driven development unit tests, behavior driven scenario specifications are also defined. To run this test specifications python behave package is also required.

## Basic Build Instructions

1. Clone this repo.
2. Move to python directory: `cd python`
3. Run it: `python unscented_kf.py`

## Running Unit Tests

A unit test suite is also included with this. You can simply run it with following command from python directory

`python -m unittest -v`

## Running Behavior-driven Scenario Specifications

A behaviour-driven feature test suite is also included with this. You can simply run it with following command from repository root (One level up from python directory)

`behave`

## Supported command line arguments

Following command line arguments are supported by unscented_kf.py.

```
usage: unscented_kf.py [--twiddle|--lidar-only|--radar-only]
  options:
    --twiddle     Twiddle parameter tuning
    --lidar-only  Use Lidar only for measurement updates
    --radar-only  Use Radar only for measurement updates
```

Please note that you can specify only one of these arguments at once.

## Code Style

Please (do your best to) stick to [Google's Python style guide](http://google.github.io/styleguide/pyguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see the project page in the classroom
for instructions and the project rubric.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).


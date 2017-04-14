# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

---

## Overview
In this project, a sensor fusion algorithm is implemented for tracking pedestrians using data from Lidar and Radar sensors.  

Depending on the sensor type, a linear or non-linear measurement model is utilised to project the measurement into cartesian space 
for commonality, to update our belief of the tracked pedestrian's pose and heading.

To process the Radar measurements, given in polar coordinates, an extended kalman filter model is used to support such measurements.
This is done by using projection and multivariate Taylor expansion, to express our non-linear measurement model into linear cartesian 
coordinates.  Without applying these techniques, our Gaussian probability function P, would become unusable and corrupt the belief state.

## Structure
The Kalman Filter algorithm has been decoupled in such a way that will allow it to seamlessly handle linear and non-linear measurements.  
This is performed using invariant projection and dampening methods in the corresponding sensor class. The projections are defined for 
converting between cartesian space, required by the kalman filter, and the sensor measurement space.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.


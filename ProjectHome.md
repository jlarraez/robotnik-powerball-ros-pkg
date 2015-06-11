# ROBOTNIK POWERBALL ROS PACKAGE #

## 1. Package Summary ##

Driver for ROS to control the [Schunk Powerball](http://www.schunk.com/schunk/schunk_websites/products/latest_products_detail.html?article_id=20026&country=INT&lngCode=EN&lngCode2=EN), currently only for simulation.

  * Autor: [Robotnik Automation](http://www.robotnik.eu/)
  * License: BSD
  * Source: svn http://robotnik-powerball-ros-pkg.googlecode.com/svn/trunk/

## 2. Supported Hardware ##

At the moment this package only works in simulation so any hardware is required. We are working in a version for the real one.


## 3. Packages ##

### 3.1 robotnik\_powerball\_sim ###

[This package](http://code.google.com/p/robotnik-powerball-ros-pkg/wiki/robotnik_powerball_sim) implements the cartesian control via PS3 pad over the arm.

### 3.2 robotnik\_powerball\_description ###

[This package](http://code.google.com/p/robotnik-powerball-ros-pkg/wiki/robotnik_powerball_description) implements the Gazebo arm simulation. It contains the meshes and URDF files for the visualization in the simulator.

### 3.3 robotnik\_powerball\_pad ###

[This package](http://code.google.com/p/robotnik-powerball-ros-pkg/wiki/robotnik_powerball_pad) makes possible to use a PS3 pad with the Schunk Powerball and [WSG 50 gripper](http://code.google.com/p/wsg50-ros-pkg/) (driver also available simulation), adapting the different buttons pulses and movements in orders.


## 4. Examples of use ##

![http://i49.tinypic.com/2l253m.jpg](http://i49.tinypic.com/2l253m.jpg)
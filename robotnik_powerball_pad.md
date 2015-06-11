## 1. Package Summary ##

Allows to use a pad with the robotnik\_powerball\_sim, sending the messages received through the joystick input, correctly adapted, to the "arm\_controller/command". If you want to test the arm with the WSG 50 gripper it is necessary to have correctly installed the [WSG 50 package](http://code.google.com/p/wsg50-ros-pkg) to satisfy some dependencies and uncomment a line in **robotnik\_powerball\_pad.cpp**.

  * Autor: [Robotnik Automation](http://www.robotnik.eu/)
  * License: BSD
  * Source: svn http://robotnik-powerball-ros-pkg.googlecode.com/svn/trunk/robotnik-powerball-ros-pkg/robotnik_powerball_pad

## 2. Topics ##

### 2.1 Subscribed topics ###

  * **/joy**: Listen this topic to know the button pressed and the axis value.

### 2.2 Published topics ###

  * **/arm\_controller/command**: Adapt the input coming from /joy to the command that expects **arm\_controller**.

## 3. Parameters ##

You can easily configure the pad map changing the .yaml file that load at the launch file (_default: ps3.yaml_).
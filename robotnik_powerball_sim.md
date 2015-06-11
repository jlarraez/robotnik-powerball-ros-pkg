## 1. Package Summary ##

This package implements the Cartesian/Euler teleoperation basing on KDL computation methods.


  * Autor: [Robotnik Automation](http://www.robotnik.eu/)
  * License: BSD
  * Source: svn http://robotnik-powerball-pkg.googlecode.com/svn/trunk/robotnik-powerball-ros-pkg/robotnik_powerball_sim

## 2. Topics ##

### 2.1. Subscribed topics ###

  * **/robotnik\_powerball\_pad/arm\_reference**: Listen the arm reference from the robotnik\_powerball\_pad (axis values, joint/modes changes...).

### 2.2. Published topics ###

  * **/arm\_controller/command**: Used to publish the computed reference to the simulated motors.

## 3. Start up ##

  * **roslaunch robotnik\_powerball\_sim powerball.launch**: Launch a standard Powerball.
  * **roslaunch robotnik\_powerball\_sim powerball\_robotnik.launch**: Launch a Powerball modified by Robotnik (longer).
  * **roslaunch robotnik\_powerball\_sim powerball\_gripper.launch**: Launch a standard Powerball with a WSG 50 gripper attached.
  * **roslaunch robotnik\_powerball\_sim powerball\_gripper\_robotnik.launch**: Launch a Powerball modified by Robotnik (longer) with a WSG 50 gripper attached.

We can control the arm in two modes, cartesian/euler pad teleoperation and sending directly joint position commands (trajectory\_msgs/JointTrajectory) to the controller topic (/arm\_controller/command).

### 3.1 Sending commands ###

Simply launch _trajectory\_msgs/JointTrajectory_ commands to the controller command topic. Example:

```
rostopic pub /arm_controller/command trajectory_msgs/JointTrajectory {'header: {stamp: {secs: 0,nsecs: 0}} ,joint_names: ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'], points: [{positions:  [0.707, -0.707, 0, 1, -1, 0], velocities: [], accelerations: [], time_from_start: {secs: 120, nsecs: 0}}]}'
```

### 3.2 Pad teleoperation ###

  * **roslaunch robotnik\_powerball\_pad powerball\_pad.launch**

When we are controlling with Cartesian/Euler via pad, we actuate directly over the XYZ position of the final effector, but the type of the command sent to the controller are the same as above. Each joint position necessary to reach the desired position are calculated easily thanks to the KDL library.
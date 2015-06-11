## 1. Package Summary ##

The URDF, meshes, and other elements needed in the simulation are contained here.

  * Autor: [Robotnik Automation](http://www.robotnik.eu/)
  * License: BSD
  * Source: svn http://robotnik-powerball-ros-pkg.googlecode.com/svn/trunk/robotnik-powerball-ros-pkg/robotnik_powerball_description

## 2. Folders ##

### 2.1. calibration ###

Contains the calibration file used by the URDFs.

### 2.2 meshes ###

This folder has the meshes of the two Powerball available, the normal one and the Robotnik long modificated (only vary the 2 and 4 stl link).

### 2.3 urdf ###

This folder has four URDF of the two Powerball available, the normal one and the Robotnik long modificated. Also includes these same Powerball with the gripper attached. **Note that to launch correctly the two last URDF is necessary to have the [WSG 50 ROS package](http://code.google.com/p/wsg50-ros-pkg/).**
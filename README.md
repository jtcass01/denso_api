# Denso RC8 Python API
Author: Jacob Taylor Cassady (jtcass01@louisville.edu) - Undergraduate Research Assistant

University of Louisville J.B. Speed School of Engineering

Next Generation Systems Robotics Lab

## 1 : Description
Python API for Denso RC8 with a RC8-native server written in PacScript.  NGS is interested in removing Labview from the control loop and interfacing with the RC8 directly to ensure well defined instruction sets.

## 2 : 3rd Part Dependencies
### 2.1 Operating System
* [Ubuntu 18 Linux Operating System](https://www.ubuntu.com/)
### 2.2 Frameworks
* [Robotic Operating System (melodic distribution)](http://wiki.ros.org/melodic)
### 2.3 Python Modules
* [Python 3.4 enum module backported for use in Python 2.7](https://pypi.org/project/enum34/)

## 3 : Usage
### 3.1 : Starting the PacScript RC8 Server
* Turn on the RC8 Controller.

* On the RC8 controller, under settings, set the robot's executable token to TP and the robot's control mode to 'Auto'.

* Start the program named 'common.pcs'.

* Turn the motor of the robot on.

### 3.2 : Starting the ROS Server Components
#### 3.2.1 : Initializing the environment
* Open a new commandline

* Navigate to directory: denso_rc8_api/src/ngs_ros/

* Delete the directories 'build' and 'devel'. (Only required the first time)

* Run the following commands:
```bash
	source /opt/ros/[YOUR_ROS_DISTRO]/setup.bash
	catkin_make
	source ./devel/setup.bash
	roscore
```

#### 3.2.2 : Starting the NGS RC8 API Server Node
This server parses and forwards commands to the RC8 PacScript Server.
* Open a new commandline

* Navigate to directory: denso_rc8_api/src/ngs_ros/

* Run the following commands:
```bash
	source ./devel/setup.bash
	rosrun ngs_denso_rc8_api rc8_api_server.py
```

#### 3.2.3 : (Optional) Starting a communication channel with the NGS RC8 API Node
* Open a new commandline

* Navigate to directory: denso_rc8_api/src/ngs_ros/

* Run the following commands:
```bash
	source ./devel/setup.bash
	rosrun ngs_denso_rc8_api api_communication_channel.py
```

#### 3.2.4 : (Optional) Starting a robot state machine to interfaces with the NGS RC8 API Server Node
* Open a new commandline

* Navigate to directory: denso_rc8_api/src/ngs_ros/

* Run the following commands:
```bash
	source ./devel/setup.bash
	rosrun ngs_denso_rc8_api robot.py
```

#### 3.2.5 : (Optional) Starting a controller to send commands to a robot state machine.
* Open a new commandline

* Navigate to directory: denso_rc8_api/src/ngs_ros/

* Run the following commands:
```bash
	source ./devel/setup.bash
	rosrun ngs_denso_rc8_api robot.py
```

### 3.3 : API Functionality
#### 3.3.1 : Implemented Commands
##### 3.3.1.1 : [Move Command](https://densorobotics.com/content/user_manuals/19/000207.html)
To move the robot to the designated coordinates.
###### 3.3.1.1.1 : Supported DataTypes
* [Position](https://densorobotics.com/content/user_manuals/19/000456.html)
* [Joint](https://densorobotics.com/content/user_manuals/19/000457.html)
###### 3.3.1.1.2 : Supported Interpolation Methods
* [Point to Point](https://densorobotics.com/content/user_manuals/19/001615.html)
* [Linear](https://densorobotics.com/content/user_manuals/19/001616.html)
###### 3.3.1.1.3 : Command Examples
**Syntax:** Move motion interpolation,target position

* Position Example
```PacScript
Move L, P( 600, 50, 400, 180, 0, 180, -1 )
```

* Joint Example
```PacScript
Move P, J(10, 20, 30, 40, 50, 60, 70, 80)
```

##### 3.3.1.2 : [Draw Command](https://densorobotics.com/content/user_manuals/19/000193.html)
To move from the current position to a relative position.
###### 3.3.1.2.1 : Supported DataTypes
* [Vector](https://densorobotics.com/content/user_manuals/19/000455.html)
###### 3.3.1.2.2 : Supported Interpolation Methods
* [Point to Point](https://densorobotics.com/content/user_manuals/19/001615.html)
* [Linear](https://densorobotics.com/content/user_manuals/19/001616.html)
###### 3.3.1.2.3 : Command Examples 
**Syntax:** Move motion interpolation,target position
* Example
```
Draw L, V( 50, 10, 50 )
```

## 4 : References
* [Denso Robotics RC8 Programmer's Manual](https://densorobotics.com/content/user_manuals/19/001301.html)

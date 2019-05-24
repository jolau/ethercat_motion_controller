# Package Name

## Overview

Motion controller for Maxon EPOS4 over EtherCAT.

**Keywords:** EtherCAT, EPOS4, motor controller, maxon

### License

The source code is released under a [BSD 3-Clause license](ros_package_template/LICENSE).

**Author: Jonas Lauener, mail@jolau.ch**

The varileg\_lowlevel\_controller package has been tested under [ROS] Kinetic and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

<img src="https://esd.eu/sites/default/files/ethercat_logo_778x240.png" alt="EtherCAT" width="300"/>

<img src="https://www.maxonmotor.com/medias/sys_master/root/8830565941278/EPOS4-Compact-50-8-EtherCAT.jpg" alt="EPOS4 EtherCAT" width="300"/>

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [openethercat_soem](https://gitlab.ethz.ch/varileg/varileg_soem_interface)
- [soem_interface](https://gitlab.ethz.ch/varileg/varileg_soem_interface)
- [varileg_msgs](https://gitlab.ethz.ch/varileg/varileg_msgs)


#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd ~/git
	git clone https://github.com/ethz-asl/ros_package_template.git
	cd ~/catkin_workspace/src
	ln -s ~/git/varileg_lowlevel_controller
	cd ../
	catkin build varileg_lowlevel_controller


## Usage

Describe the quickest way to run this software, for example:

Run the main node with

	roslaunch varileg_lowlevel_controller varileg_lowlevel_controller_setcap.launch

## Config files

Config file folder config:

* **config.yaml** Set the names of the Ethernet ports. Can be found by using command ifconfig. Also defines topic names.
* **joint_config.yaml** Configure mapping of joints to EPOS's NodeID. Set parameters of each joint.


## Launch files

* **varileg\_lowlevel\_controller varileg\_lowlevel\_controller\_setcap.launch:** Start the main node with root rights by starting varileg\_lowlevel\_controller\_setcap.sh and then finally varileg\_lowlevel\_controller varileg\_lowlevel\_controller\_setcap.launch

     Arguments:

     - **`pasword`** Root password Default: `varileg`.
     - **`motor_current`** Max. motor current in mA Default: `2000`.
     - **`frequency`** Frequency [Hz] of the update loop. Trajectory point have to come with same frequency. Default: `100`.


* **varileg\_lowlevel\_controller epos_example_setcap.launch**: Starts a simple testing node, great for developing. Usage similiar to above.

## Nodes

#### Subscribed Topics

* **`/joint_trajectory`** ([varileg_msgs/ExtendedJointTrajectories.msg])

	Trajectory points for all joints.


#### Published Topics

* **`joint_states`** ([varileg_msgs/ExtendedJointStates.msg])

	Current position, velocity and torque of all joints.
	
* **`device_states`** ([varileg_msgs/ExtendedDeviceStates.msg])

	Current device states of all joints.

#### Services

* **`set_device_state`** ([varileg_msgs/SetDeviceState.srv])

	Set the targeted device state of a joint. Will be tried to reach with the next update cycle.
	
* **`set_device_state`** ([varileg_msgs/SetOperatingMode.srv])

	Change the operating mode of a joint. Will be tried to be changed with the next update cycle. Joint needs to be in SWITCH ON DISABLED state.


#### Actions

* **`homing/[joint_name]`** ([varileg_msgs/Homing.action])

	Start homing of a joint. Needs to be in homing mode and enabled.


[ROS]: http://www.ros.org
[varileg_msgs/ExtendedJointTrajectories.msg]: https://gitlab.ethz.ch/varileg/varileg_msgs/blob/master/varileg_msgs/msg/ExtendedJointTrajectories.msg
[varileg_msgs/ExtendedJointStates.msg]: https://gitlab.ethz.ch/varileg/varileg_msgs/blob/master/varileg_msgs/msg/ExtendedJointStates.msg
[varileg_msgs/ExtendedDeviceStates.msg]: https://gitlab.ethz.ch/varileg/varileg_msgs/blob/master/varileg_msgs/msg/ExtendedDeviceStates.msg
[varileg_msgs/SetDeviceState.srv]: https://gitlab.ethz.ch/varileg/varileg_msgs/blob/master/varileg_msgs/srv/SetDeviceState.srv
[varileg_msgs/SetOperatingMode.srv]: https://gitlab.ethz.ch/varileg/varileg_msgs/blob/master/varileg_msgs/srv/SetOperatingMode.srv
[varileg_msgs/Homing.action]: https://gitlab.ethz.ch/varileg/varileg_msgs/blob/master/varileg_msgs/action/Homing.action
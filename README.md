# FrankaPanda driver for SAI
Driver for the Franka Panda arm for use with SAI and communications via redis.

This program provides a wrapper around the libfranka API to control a Franka robot in torque mode. It reads torque commands from redis and publishes to redis the robot joint angles, joint velocities, sensed torques, mass matrix, gravity torques, coriolis torques.

It also implements torque saturation, joint position limit and joint velocity limit avoidance.

## Install

This code has been developped and tested on a Ubuntu machine using the rt_preempt kernel

1. Set up your Panda robot and controller following the instructions on [this page](https://frankaemika.github.io/docs/getting_started.html). Connect your computer to the robot controller (not the robot directly)

2. Download libfranka library and compile it. Use the version tag required by your hardware. This code has been tested on tag 0.13.0

		sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
		git clone --recursive https://github.com/frankaemika/libfranka
		cd libfranka
		git checkout 0.13.0
		git submodule update
create a build folder inside libfranka and compile the library

		mkdir build
		cd build
		cmake -DCMAKE_BUILD_TYPE=Release ..
		cmake --build .

3. Build the driver

		sudo apt-get install libjsoncpp-dev cpufrequtils
		cd ../../redis_driver
		mkdir build
		cd build
		cmake ..
		make

4. Don't forget to give realtime permissions to your computer account

		sudo usermod -a -G realtime $(whoami)

## Usage

1. Open the robot interface by connecting to the ip address of the controller in your web browser (e.g. 172.16.0.10)

2. Open the User stop and the brakes of the robot through the interface

3. Open a terminal and go to the FrankaPanda/redis_driver folder

		cd <path_to_FrankaPanda_directory>/redis_driver

3. Launch the redis server from the given script

		sh launch_redis_server.sh

4. Launch the driver from the provided script. Make sure the config file in the script is the correct one (if none is specified, it will use the default_config)

		sh launch_driver.sh

## Gripper usage

Once the robot interface is connected through the web browser and the gripper is initialized you can launch the gripper driver
		
		./build/sai_franka_gripper_redis_driver [(optional) config_file_name]

If no config file is provided, it will use the `default_config.xml`
You can set the desired width, desired speed, desired force and gripper mode (m for motion g for grasp) through redis

## Config file and redis keys

The config file needs to be in the folder `config_folder`. It is an xml file with one `saiFrankaDriverConfig` tag containing the following attributes:

<saiFrankaDriverConfig robotName="FrankaRobot" robotIP="172.16.0.10" redisPrefix="sai" robotType="fr3"/>

- redisPrefix: the prefix for the redis keys
- robotName: the name of the robot (for constructing the redis keys)
- robotIP: the ip address of the robot control box
- robotType: can be either "fr3" or "panda"

#### The robot driver will listen to the following redis key:

`<prefix>::commands::<robot_name>::control_torques`: this is the key sending the command torques to the robot. The robot performs its own gravity compensation so the sent command torques should not contain the gravity compensation torques.

#### The robot driver will publish the following keys:

`<prefix>::sensors::<robot_name>::joint_positions`: the position of the robot joints

`<prefix>::sensors::<robot_name>::joint_velocities`: the velocities of the robot joints

`<prefix>::sensors::<robot_name>::joint_torques`: the sensed joint torques (from robot torque sensors)

`<prefix>::sensors::<robot_name>::model::mass_matrix`: the robot joint space mass matrix

`<prefix>::sensors::<robot_name>::model::robot_gravity`: the joint gravity torques

`<prefix>::sensors::<robot_name>::model::coriolis`: the joint coriolis and centrifugal forces vector

`<prefix>::sensors::<robot_name>::safety_controller::safety_torques`: The torques computes by the integrated safety controller

`<prefix>::sensors::<robot_name>::safety_controller::constraint_nullspace`: The nullspace of the safety controller constraint task

`<prefix>::sensors::<robot_name>::safety_controller::sent_torques`: The torques actually sent to the robot. This will be equal to the command torques when no joint position, velocity or torque limit is triggered.

#### The gripper driver reads the following keys:

`<prefix>::<robot_name>::gripper::mode`: The mode for the gripper. Can be "m" for motion or "g" for grasp

`<prefix>::<robot_name>::gripper::desired_width`: The desired width of the gripper

`<prefix>::<robot_name>::gripper::desired_speed`: The desired speed of the gripper when moving

`<prefix>::<robot_name>::gripper::desired_force`: The desired grasping force of the gripper in grasp mode

#### The gripper driver publishes the following keys:
`<prefix>::<robot_name>::gripper::max_width`: The maximum width of the gripper

`<prefix>::<robot_name>::gripper::current_width`: The current width of the gripper

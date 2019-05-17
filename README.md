FrankaPanda
===========
model description and driver for the Franka Panda arm


Install
-------

This code has been developped and tested on a Ubuntu 16.04 machine using rt_preempt patch (4.16.18-rt12).

1. Set up your Panda robot and controller following the instructions on [this page](https://frankaemika.github.io/docs/getting_started.html). Connect your computer to the robot controller (not the robot directly)

2. Download libfranka library and compile it. use tag 0.5.0.

		sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
		git clone --recursive https://github.com/frankaemika/libfranka
		cd libfranka
		git checkout 0.5.0
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

Usage
-----

1. Open the robot interface by connecting to the ip address of the controller in your web browser (e.g. 172.16.0.10)

2. Open the User stop and the brakes of the robot through the interface

3. Open a terminal and go to the FrankaPanda/redis_driver folder

		cd <path_to_FrankaPanda_directory>/redis_driver

3. Launch the redis server from the given script

		sh launch_redis_server.sh

4. Launch the driver from the provided script. Make sure the ip address in the script is the one of your robot controller

		sh launch_driver.sh

Gripper usage
-------------

Once the robot interface is connected through the web browser and the gripper is initialized you can launch the gripper driver
		
		./build/gripper_driver <ip-address-of-robot>

You can set the desired width, desired speed, desired force and gripper mode (m for motion g for grasp) through redis

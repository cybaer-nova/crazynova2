# Instructions to run code with Crazyflies

First open VS Code and the folder `~/arena/ros2_ws/src/crazynova2`.

You should edit only the file:
	~/arena/ros2_ws/src/crazynova2/crazyflie_examples/cybaer_controller.py

This file contains the linear motion controller function `doControl`, and the reference generation function `getReference` that you need, at the top of the function. You should not change the other functions.

## change the reference

You should change the final maximum reference time `T_final` (in seconds), and your group name `group_str` for the csv logs to be clearer.

Then you should start with very simple references to see if you controller works.

If you want to setup a path planning algorithm for the trials, you can compute the path using an additional function. Let me know if this is the case.

## setup the crazyflie you are using

Edit the config file:

	~/arena/ros2_ws/src/crazynova2/crazyflie/crazyflies.yaml

Set `robots:cf{i}:enabled` to `false` for the ones you are not using in your trial.
Set `robots:cf{i}:enabled` to `true` for the one you are using in your trial.
Do not change anything else.

Connect all lighthouse beacons.

Ensure the drone you are using has a full battery.

Power on the drone in its spot and and do not touch it any longer.

## run trial

Open to terminal 1 and run the commands:

	cd arena/ros2_ws
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
	source install/local_setup.bash
	ros2 launch crazyflie launch.py backend:=cpp

Check if all is ok in the link with the drone.

Open terminal 2 and run:

	cd arena/ros2_ws
	source install/local_setup.bash
	
**Before starting the trial**: ensure the safety conditions are all met, in particular, at least tow authorized persons are present, everyone outside of the flight arena, and all hardware and software has been checked.

**Start the trial** running:

	ros2 run crazyflie_examples cybaer_controller 

CSV logs are saved in the folder where the controller is called, at `~/arena/ros2_ws`. You can import these in matlab and show the results in your report.

	
	


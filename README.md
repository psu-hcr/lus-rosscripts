# lus-rosscripts
This is a temporary file for lus iiwa_ros scripts. These need to be organized into a single package and removed from the organization repo.

##These scripts must be documented in the readme or removed. 

## Requirements

This project is set up as a submodule of the psu-hcr fork of epfl-lasa/iiwa_ros repo. You should clone this repo and initialize submodules

	$ git clone https://github.com/psu-hcr/iiwa_ros.git
	$ git submodule init
	$ git submodule update
	
The command above will place this submodule repo in a detached head state. If you want to make modifications and push to the lus-rosscripts repo, contact Lu.

## To source the project
You should source setup.bash of your workspace whenever you open a new Terminal. Run command below to source your project: 

	$ source devel/setup.bash
	
## Run a simple example moving Joint 4 of the kuka from 0 to 90deg
### In Simulation
Step 1: Run command below to launch Gazebo with IIWA in position-control mode. 

	$ roslaunch iiwa_gazebo iiwa_gazebo.launch controller:=PositionController

Step 2: Open a new Terminal. Source the project. If BendJ4.py is not executable, run command:

	$ chmod +x BendJ4.py

Step 3: Run command below to run code.

	$ rosrun iiwa_ros BendJ4.py
	
### On the LBR
Step 1 & 2: Follow step 1 & 2 in epfl-lasa/iiwa_ros repo section "Control IIWA with FRI". Choose "position" and stiffness "150" on smartpad

Step 3: Within 5 seconds before the timeout, launch: 

	$ roslaunch iiwa_driver iiwa_bringup.launch controller:=PositionController
	
Step 4: Open a new Terminal. Source the project. Run command below to run code.
	
	$ rosrun iiwa_ros BendJ4.py

## Run a simple example Joint 2 and Joint 6 each from 0deg to 90deg
### In Simulation
Step 1: Run command below to launch Gazebo with IIWA in position-control mode.

	$ roslaunch iiwa_gazebo iiwa_gazebo.launch controller:=PositionController

Step 2: Open a new Terminal. Source the project. If BendJ2J6.py is not executable, run command:

	$ chmod +x BendJ2J6.py

Step 3: Rosrun code

	$ rosrun iiwa_ros BendJ2J6.py
	
### On the LBR
Step 1 & 2: Follow step 1 & 2 in epfl-lasa/iiwa_ros repo section "Control IIWA with FRI". Choose "position" and stiffness "150" on smartpad

Step 3: Within 5 seconds before the timeout, launch: 

	$ roslaunch iiwa_driver iiwa_bringup.launch controller:=PositionController
	
Step 4: Open a new Terminal. Source the project. Run command below to run code.
	
	$ rosrun iiwa_ros BendJ2J6.py
	
## Run a simple example using a low torque to move Joint 4 from 90def to 0deg (slowly increase the constant torque value until error results)
### In Simulation
Step 1: Run command below to launch Gazebo with IIWA in position-control mode.

	$ roslaunch iiwa_gazebo iiwa_gazebo.launch 

Step 2: Open a new Terminal. Source the project. If BendJ4_T.py is not executable, run command:

	$ chmod +x BendJ4_T.py

Step 3: Run command below to run code.

	$ rosrun iiwa_ros BendJ4_T.py
	
### On the LBR
Step 1 & 2: Follow step 1 & 2 in epfl-lasa/iiwa_ros repo section "Control IIWA with FRI" Choose "Torque" and stiffness "0" on smartpad


Step 3: Within 5 seconds before the timeout, launch: 

	$ roslaunch iiwa_driver iiwa_bringup.launch 
	
Step 4: Open a new Terminal. Source the project. Run command below to run code.
	
	$ rosrun iiwa_ros BendJ4_T.py
	
## How to fix some common issues
### Smartpad shows "hardware limit exceeded drive (X)"
Step 1: look at "Axis Position" in "robot" page. Find the exceed limit axis.

Step 2: Go to "mastering" in "robot" page. Unmaster the problem axis

Step 3: Go to "T1" mode. Jog the robot back to working space

Step 4: Master again and error should be solved

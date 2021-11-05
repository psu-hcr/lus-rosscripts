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
You should source the project whenever you open a new Terminal. Run command below to source your project: 

	$ source /opt/ros/noetic/setup.bash 
	
## Run a simple example moving Joint 4 of the kuka from 0 to 90deg
### In Simulation
Step 1: Run command below to launch Gazebo with IIWA in position-control mode. 

	$ roslaunch iiwa_gazebo iiwa_gazebo.launch controller:=PositionController

Step 2: Open a new Terminal. Source the project. Run command below to run code.

	$ rosrun iiwa_ros BendJ4.py
	
### On the LBR
Step 1 & 2: Follow step 1 & 2 in epfl-lasa/iiwa_ros repo section "Control IIWA with FRI"

Step 3: Within 5 seconds before the timeout, launch: 

	$ roslaunch iiwa_driver iiwa_bringup.launch controller:=PositionController
	
Step 4: Open a new Terminal. Source the project. Run command below to run code.
	
	$ rosrun iiwa_ros BendJ4.py

## Run a simple example Joint 2 and Joint 6 each from 0deg to 90deg
### In Simulation
Step 1: Run command below to launch Gazebo with IIWA in position-control mode. 

	$ roslaunch iiwa_gazebo iiwa_gazebo.launch controller:=PositionController

Step 2: Open a new Terminal. Source the project. Run command below to run code.

	$ rosrun iiwa_ros BendJ2J6.py
	
### On the LBR
Step 1 & 2: Follow step 1 & 2 in epfl-lasa/iiwa_ros repo section "Control IIWA with FRI"

Step 3: Within 5 seconds before the timeout, launch: 

	$ roslaunch iiwa_driver iiwa_bringup.launch controller:=PositionController
	
Step 4: Open a new Terminal. Source the project. Run command below to run code.
	
	$ rosrun iiwa_ros BendJ2J6.py
	
## Run a simple example using a low torque to move Joint 4 from 90def to 0deg (slowly increase the constant torque value until error results
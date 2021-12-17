# lus-rosscripts
This is a temporary file for lus iiwa_ros scripts. These need to be organized into a single package and removed from the organization repo.
This project is set up as a submodule of the psu-hcr fork of epfl-lasa/iiwa_ros repo. If you want to make modifications and push to the lus-rosscripts repo, contact Lu.

## Requirements
### Setup catkin workspace
Step 1: source your environment

	$ source /opt/ros/noetic/setup.bash

Step 2: create and build a catkin workspace. 

	$ mkdir -p ~/catkin_ws/src
	$ cd ~/catkin_ws/
	$ catkin_make
	
### Clone this repo and initialize submodules

	$ cd ~/catkin_ws/src
	$ git clone https://github.com/psu-hcr/iiwa_ros.git
	$ cd iiwa_ros/
	$ git submodule init
	$ git submodule update

### Install Dependcies from epfl-lasa/iiwa_ros repo
Follow the **Dependcies** and **Compilation** sections in [epfl-lasa/iiwa_ros repo](https://github.com/epfl-lasa/iiwa_ros).
Go through **Basic Usage** section to make sure repo is set up correctly.

## To source the project
You should source setup.bash of your workspace whenever you open a new Terminal. Run command below to source your project: 

	$ source ~/catkin_ws/devel/setup.bash
	
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
	
## How to use iiwa_moveit
Step 1: lunch iiwa_moveit

	$ roslaunch iiwa_moveit demo.launch driver:=true

step 2: Within 10 seconds before the timeout, run "FRIOverlay" applications on KUKA smart pad
	
## How to fix some common issues
### Smartpad shows "hardware limit exceeded drive (X)"
Step 1: look at "Axis Position" in "robot" page. Find the exceed limit axis.

Step 2: Go to "mastering" in "robot" page. Unmaster the problem axis

Step 3: Go to "T1" mode. Jog the robot back to working space

Step 4: Master again and error should be solved

## Adding/changing robot end effectors

Step 1: Create a 3d model of the end effector.

Step 2: The default unit for the URDF format is the meter, which means if you made the part in millimeter you need to scale your model down 1000x.
In addition, it is critical that you know where the origin is relative to the model. You may have to move the model to get the origin to align.

Step 3: Export the end effector as an STL file.

Step 4: Go to the catkin_ws/src/iiwa_ros/iiwa_description/meshes/iiwa14 and put the stl in both the visual and collision folders

Step 5: Go back to the iiwa_description folder and create a copy of the iiwa14.xacro and iiwa14.urdf.xacro files. Rename the files by changing the number.
	
Step 6: In the .xacro folder find the joint between link 6 and end effector (~line 390). Change the link visual and collision geometry to the file path to the STL file of the end effector (lines 408 and 417 respectively). Make sure to uncomment the line.

Step 7: In the .urdf.xacro file change the robot name (line 27) and the iiwa macro filename (line 31)  to the number assigned to the new file (ex. from iiwa14 to iiwa15).

Step 8: go to the directory catkin_ws/src/iiwa_ros/iiwa_description/launch and create a copy of the iiwa14_upload.launch file. Rename the file with the same model number you used for the .urdf.xacro and .xacro files. On line 34, change the name of the .urdf.xacro file to have the correct model number.

Step 9: To launch the robot with the end effector use the following command: roslaunch iiwa_gazebo iiwa_gazebo.launch model:="(model number assigned to new model)"

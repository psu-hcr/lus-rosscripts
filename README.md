# lus-rosscripts
This is a temporary file for lus iiwa_ros scripts. These need to be organized into a single package and removed from the organization repo.

##These scripts must be documented in the readme or removed. 

## Requirements

This project is set up as a submodule of the psu-hcr fork of epfl-lasa/iiwa_ros repo. You should clone this repo and initialize submodules

	$ git clone https://github.com/psu-hcr/iiwa_ros.git
	$ git submodule init
	$ git submodule update
	
The command above will place this submodule repo in a detached head state. If you want to make modifications and push to the lus-rosscripts repo, contact Lu.

## To build the project

	$ do you have to run something?
	
## Run a simple example moving Joint 4 of the kuka from 0 to 90deg
### In Simulation

	$ roslaunch bendJ4
	
### On the LBR

	$ roslaunch bendJ4

## Run a simple example Joint 2 and Joint 6 each from 0deg to 90deg

    $ roslaunch bendJ2J6
	
## Run a simple example compensating for gravity with a torque on Joint 4 when it is held at 90def
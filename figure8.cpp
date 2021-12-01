#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include"src/error_cost.hpp"
#include"src/SAC.hpp"
#include"src/rk4_int.hpp"
#include"src/koopsys.hpp"
#include"src/iiwabasis1.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
#include<math.h>
#include<armadillo>

//desire traj
arma::vec xdk(double t){
	double x = 600;
	double y = 250*cos((1/5)*t);
	double z = 100*sin((2/5)*t)+700;
	double wx = 1;
	double wy = 0;
	double wz = 0;
	double w = 0;
	return arma::vec{x, y, z, wx, wy, wz, w};
}

class figure8 {
	private:
	ros::Subscriber sub;
	
	public:
	figure8(ros::NodeHandle *nh, KoopSys<BasisName> systK){
		sub = nh->subscribe("/iiwa/joint_states", 1000, &figure8::chatterCallback, this);
	}
	
	void chatterCallback(sensor_msgs::JointState jointState){
		std::cout << jointState;
	}
		
};




int main(int argc, char **argv){
	ros::init(argc, argv,"figure8");
	ros::NodeHandle nh;
	BasisName iiwaBasis;
	KoopSys<BasisName> systK (0.01, &iiwaBasis);
	
	arma::mat R = 0.001*arma::eye(7,7);
 	arma::mat Qk = 200*arma::eye(14,14);
    
	
	figure8 nc = figure8(&nh, systK);
	
	ros::spin();
	
	return 0;
}
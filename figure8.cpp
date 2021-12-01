#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include </home/zxl5344/IROS2022/src/iiwa_ros/koopman-op/src/SAC.hpp>
#include </home/zxl5344/IROS2022/src/iiwa_ros/koopman-op/src/rk4_int.hpp>

#include <sstream>
#include <iostream>
#include <fstream>
#include<math.h>
#include<armadillo>
using namespace std;

class figure8 {
	private:
	ros::Subscriber sub;
	
	public:
	figure8(ros::NodeHandle *nh){
		sub = nh->subscribe("/iiwa/joint_states", 1000, &figure8::chatterCallback, this);
	}
	
	void chatterCallback(sensor_msgs::JointState jointState){
		cout << jointState;
	}
		
};



int main(int argc, char **argv){
	ros::init(argc, argv,"figure8");
	ros::NodeHandle nh;
	
	figure8 nc = figure8(&nh);
	
	ros::spin();
	
	return 0;
}
#include <ros/ros.h>
#include <autonomous_robot/SingleAgentExploration.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "sa_exploration_node");
	ros::NodeHandle nh;
	autonomous_robot::saExploration exploration(nh);
	ros::spin();
	return 0;
}

#ifndef AUTONOMOUS_ROBOT_SPOT_PLATFORM_H
#define AUTONOMOUS_ROBOT_SPOT_PLATFORM_H

#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <communication_manager/robotConfigMsg.h>

namespace autonomous_robot {
	// Spot interface: odometry in, waypoint poses out (the Spot controller tracks the published pose),
	// plus the robot configuration shared with the mapping and planning modules.
	class SpotPlatform {
	private:
		ros::NodeHandle nh_;
		ros::Subscriber odomSub_;
		ros::Publisher wptPub_;
		std::string odomTopic_;
		std::string cmdTopic_;
		nav_msgs::Odometry odom_;
		bool odomReceived_ = false;
		bool trajectoryReady_ = false;
		nav_msgs::Path trajectory_;
		geometry_msgs::PoseStamped poseTgt_;

		// parameters
		double desiredVel_ = 0.5;
		double desiredAngularVel_ = 0.5;
		double reachGoalDistance_ = 0.5;
		int robotId_ = 0;
		bool aggressiveMode_ = false;
		Eigen::Vector3d collisionBoxMin_, collisionBoxMax_;  // body frame
		Eigen::Vector3d safeDistVec_;
		std::vector<Eigen::Vector3d> circleDecompCenters_;
		double circleDecompRadius_ = 0.0, circleDecompZMin_ = 0.0, circleDecompZMax_ = 0.0;
		Eigen::Vector2d sampleHeightRange_;
		std::string rangeSensorType_;
		Eigen::Matrix<double, 4, 4, Eigen::RowMajor> body2RangeSensor_;
		Eigen::Vector4d lidarConfig_;                 // dmin, dmax, fovUpper, fovLower
		Eigen::Matrix<double, 11, 1> depthCamConfig_;  // fx, fy, cx, cy, dmin, dmax, scale, filterMargin, skipPixel, cols, rows

		void loadParameters();
		void odomCB(const nav_msgs::OdometryConstPtr& odom);

	public:
		// Subscribes to odometry and blocks until the first message arrives.
		explicit SpotPlatform(const ros::NodeHandle& nh);
		// Commands a 0.35 m forward step and waits until the body is standing (z >= 0.2 m).
		bool initialize();
		// Publishes the current pose as the target.
		void stop();
		// 2-D arrival test; with useYaw only the heading is checked (within 0.1 rad).
		bool isReach(const geometry_msgs::PoseStamped& poseTgt, double dist, bool useYaw) const;
		// Positional arrival test that also marks the current trajectory as consumed.
		bool reachedGoal(const geometry_msgs::PoseStamped& poseTgt, double dist);
		void publishTgtCmd(const geometry_msgs::PoseStamped& poseTgt);
		// Publishes the last pose of the current trajectory (or the previous target when it is empty).
		void executeTraj();
		// Fills the robot config message shared with the mapping and planning modules.
		void setRobotConfigMsg(communication_manager::robotConfigMsg& msg) const;

		nav_msgs::Odometry getOdom() const { return this->odom_; }
		geometry_msgs::PoseStamped getCurrentPose() const;
		bool isOdomReceived() const { return this->odomReceived_; }
		double getDesiredVelocity() const { return this->desiredVel_; }
		double getDesiredAngularVelocity() const { return this->desiredAngularVel_; }
		double getReachGoalDistance() const { return this->reachGoalDistance_; }
		bool isTrajectoryReady() const { return this->trajectoryReady_; }
		void setTrajectoryReady(bool ready) { this->trajectoryReady_ = ready; }
		void setTrajectory(const nav_msgs::Path& path) { this->trajectory_ = path; }
		nav_msgs::Path getTrajectory() const { return this->trajectory_; }
		void clearTrajectory() { this->trajectory_ = nav_msgs::Path(); }
	};
}

#endif

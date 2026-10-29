#include <autonomous_robot/spotPlatform.h>
#include <autonomous_robot/utils.h>
#include <algorithm>
#include <cmath>

namespace autonomous_robot {
	SpotPlatform::SpotPlatform(const ros::NodeHandle& nh) : nh_(nh) {
		this->nh_.getParam("odom_topic", this->odomTopic_);
		this->nh_.param<std::string>("cmd_topic", this->cmdTopic_, "/cmd_vel");
		this->odomSub_ = this->nh_.subscribe(this->odomTopic_, 30, &SpotPlatform::odomCB, this);
		this->wptPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>(this->cmdTopic_, 10);
		ros::Rate r(10);
		while (ros::ok() and not this->odomReceived_) {
			ROS_INFO_THROTTLE(2.0, "[SpotPlatform] waiting for odometry on %s", this->odomTopic_.c_str());
			ros::spinOnce();
			r.sleep();
		}
		this->loadParameters();
	}

	void SpotPlatform::odomCB(const nav_msgs::OdometryConstPtr& odom) {
		this->odom_ = *odom;
		this->odomReceived_ = true;
	}

	bool SpotPlatform::initialize() {
		if (not this->odomReceived_) {
			ROS_ERROR("[SpotPlatform] no odometry received");
			return false;
		}
		geometry_msgs::PoseStamped initGoal = this->getCurrentPose();
		initGoal.pose.position.x += 0.35;
		this->publishTgtCmd(initGoal);
		ros::Rate rate(10);
		while (ros::ok() && this->odom_.pose.pose.position.z < 0.2) {
			ROS_INFO_THROTTLE(2.0, "[SpotPlatform] waiting for the robot to stand up (z = %.3f)", this->odom_.pose.pose.position.z);
			ros::spinOnce();
			rate.sleep();
		}
		return true;
	}

	void SpotPlatform::stop() {
		geometry_msgs::PoseStamped pose;
		pose.pose = this->odom_.pose.pose;
		this->publishTgtCmd(pose);
	}

	geometry_msgs::PoseStamped SpotPlatform::getCurrentPose() const {
		geometry_msgs::PoseStamped pose;
		pose.header = this->odom_.header;
		pose.pose = this->odom_.pose.pose;
		return pose;
	}

	bool SpotPlatform::isReach(const geometry_msgs::PoseStamped& poseTgt, double dist, bool useYaw) const {
		if (useYaw) {
			double targetYaw = rpy_from_quaternion(poseTgt.pose.orientation);
			double currYaw = rpy_from_quaternion(this->odom_.pose.pose.orientation);
			return std::abs(targetYaw - currYaw) < 0.1;
		}
		return std::abs(poseTgt.pose.position.x - this->odom_.pose.pose.position.x) < dist &&
		       std::abs(poseTgt.pose.position.y - this->odom_.pose.pose.position.y) < dist;
	}

	bool SpotPlatform::reachedGoal(const geometry_msgs::PoseStamped& poseTgt, double dist) {
		bool reached = this->isReach(poseTgt, dist, false);
		if (reached) this->trajectoryReady_ = false;
		return reached;
	}

	void SpotPlatform::publishTgtCmd(const geometry_msgs::PoseStamped& poseTgt) {
		this->wptPub_.publish(poseTgt);
	}

	void SpotPlatform::executeTraj() {
		if (not this->trajectory_.poses.empty()) {
			this->poseTgt_ = this->trajectory_.poses.back();
		}
		this->publishTgtCmd(this->poseTgt_);
	}

	void SpotPlatform::loadParameters() {
		this->nh_.param("desired_velocity", this->desiredVel_, 0.5);
		this->nh_.param("desired_angular_velocity", this->desiredAngularVel_, 0.5);
		this->nh_.param("reach_goal_distance", this->reachGoalDistance_, 0.5);
		this->nh_.param("robot_id", this->robotId_, 0);
		this->nh_.param("aggressive_mode", this->aggressiveMode_, false);

		std::vector<double> collisionBox;
		if (this->nh_.getParam("collision_box", collisionBox) && collisionBox.size() >= 6) {
			this->collisionBoxMin_ = Eigen::Vector3d(collisionBox[0], collisionBox[1], collisionBox[2]);
			this->collisionBoxMax_ = Eigen::Vector3d(collisionBox[3], collisionBox[4], collisionBox[5]);
		} else {
			this->collisionBoxMin_ = Eigen::Vector3d(-0.4, -0.2, -0.3);
			this->collisionBoxMax_ = Eigen::Vector3d(0.4, 0.2, 0.5);
		}
		std::vector<double> safeDist;
		if (this->nh_.getParam("safe_dist", safeDist) && safeDist.size() >= 3) {
			this->safeDistVec_ = Eigen::Vector3d(safeDist[0], safeDist[1], safeDist[2]);
		} else {
			this->safeDistVec_ = Eigen::Vector3d(0.3, 0.3, 0.0);
		}

		std::vector<double> centers;
		this->nh_.param("circle_decomp/centers", centers, std::vector<double>{0.0, 0.0, 0.0});
		for (size_t i = 0; i + 2 < centers.size(); i += 3) {
			this->circleDecompCenters_.emplace_back(centers[i], centers[i + 1], centers[i + 2]);
		}
		if (this->circleDecompCenters_.empty()) {
			this->circleDecompCenters_.emplace_back(0.5 * (this->collisionBoxMin_(0) + this->collisionBoxMax_(0)),
			                                        0.5 * (this->collisionBoxMin_(1) + this->collisionBoxMax_(1)), 0.0);
		}
		this->nh_.param("circle_decomp/radius", this->circleDecompRadius_, 0.5);
		this->nh_.param("circle_decomp/z_min", this->circleDecompZMin_, this->collisionBoxMin_(2) - this->safeDistVec_(2));
		this->nh_.param("circle_decomp/z_max", this->circleDecompZMax_, this->collisionBoxMax_(2) + this->safeDistVec_(2));

		std::vector<double> sampleHeight;
		if (this->nh_.getParam("sample_height_range", sampleHeight) && sampleHeight.size() >= 2) {
			this->sampleHeightRange_ = Eigen::Vector2d(sampleHeight[0], sampleHeight[1]);
		} else {
			this->sampleHeightRange_ = Eigen::Vector2d(0.4, 0.4);
		}

		this->nh_.param<std::string>("range_sensor_type", this->rangeSensorType_, "lidar");
		std::vector<double> body2RangeSensor;
		if (not this->nh_.getParam("body_to_range_sensor", body2RangeSensor) || body2RangeSensor.size() != 16) {
			ROS_ERROR("[SpotPlatform] body_to_range_sensor (16 values, row major) is required");
			exit(0);
		}
		this->body2RangeSensor_ = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(body2RangeSensor.data());

		this->lidarConfig_.setZero();
		this->depthCamConfig_.setZero();
		if (this->rangeSensorType_ == "lidar") {
			double dmin, dmax, fovUpper, fovLower;
			this->nh_.param("sample_range_min", dmin, 0.5);
			this->nh_.param("sample_range_max", dmax, 8.0);
			if (this->nh_.hasParam("range_sensor_fov_upper") && this->nh_.hasParam("range_sensor_fov_lower")) {
				this->nh_.param("range_sensor_fov_upper", fovUpper, 48.0);
				this->nh_.param("range_sensor_fov_lower", fovLower, -48.0);
			} else {
				double fov;
				this->nh_.param("range_sensor_fov", fov, 48.0);
				fovUpper = fov / 2.0;
				fovLower = -fov / 2.0;
			}
			this->lidarConfig_ << dmin, dmax, fovUpper, fovLower;
		} else if (this->rangeSensorType_ == "pinhole") {
			std::vector<double> intrinsics;
			std::vector<int> resolution;
			double depthMin, depthMax, depthScale, filterMargin, skipPixel;
			this->nh_.param("range_sensor_intrinsics", intrinsics, std::vector<double>{320, 320, 320, 240.0});
			this->nh_.param("range_sensor_resolution", resolution, std::vector<int>{1280, 720});
			this->nh_.param("depth_min_value", depthMin, 0.5);
			this->nh_.param("depth_max_value", depthMax, 5.0);
			this->nh_.param("depth_scale_factor", depthScale, 1000.0);
			this->nh_.param("depth_filter_margin", filterMargin, 2.0);
			this->nh_.param("depth_skip_pixel", skipPixel, 2.0);
			this->depthCamConfig_ << intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3],
			                         depthMin, depthMax, depthScale, filterMargin, skipPixel,
			                         static_cast<double>(resolution[0]), static_cast<double>(resolution[1]);
		} else {
			ROS_ERROR("[SpotPlatform] unsupported range_sensor_type %s", this->rangeSensorType_.c_str());
		}
	}

	void SpotPlatform::setRobotConfigMsg(communication_manager::robotConfigMsg& msg) const {
		msg.id = this->robotId_;
		msg.robot_type = 2;  // legged
		msg.linear_speed = this->desiredVel_;
		msg.aggressive_mode = this->aggressiveMode_;
		for (int i = 0; i < 3; ++i) {
			msg.collision_box_min[i] = this->collisionBoxMin_(i);
			msg.collision_box_max[i] = this->collisionBoxMax_(i);
			msg.safe_dist[i] = this->safeDistVec_(i);
		}
		msg.circle_decomp_centers.clear();
		for (const auto& c : this->circleDecompCenters_) {
			msg.circle_decomp_centers.insert(msg.circle_decomp_centers.end(), {c(0), c(1), c(2)});
		}
		msg.circle_decomp_radius = this->circleDecompRadius_;
		msg.circle_decomp_z_min = this->circleDecompZMin_;
		msg.circle_decomp_z_max = this->circleDecompZMax_;
		msg.range_sensor_type = this->rangeSensorType_;
		std::copy(this->body2RangeSensor_.data(), this->body2RangeSensor_.data() + 16, msg.body_to_range_sensor.begin());
		std::copy(this->lidarConfig_.data(), this->lidarConfig_.data() + 4, msg.lidar_config.begin());
		std::copy(this->depthCamConfig_.data(), this->depthCamConfig_.data() + 11, msg.depth_cam_config.begin());
		msg.sample_height_range[0] = this->sampleHeightRange_(0);
		msg.sample_height_range[1] = this->sampleHeightRange_(1);
	}
}

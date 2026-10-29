#ifndef AUTONOMOUS_ROBOT_UTILS_H
#define AUTONOMOUS_ROBOT_UTILS_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>

namespace autonomous_robot {
	const double PI_const = 3.1415926;

	// Quaternion from roll/pitch/yaw; yaw above pi is wrapped to (-pi, pi].
	inline geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw) {
		if (yaw > PI_const) yaw = yaw - 2 * PI_const;
		tf2::Quaternion q;
		q.setRPY(roll, pitch, yaw);
		return tf2::toMsg(q);
	}

	// Yaw of a quaternion in (-pi, pi].
	inline double rpy_from_quaternion(const geometry_msgs::Quaternion& quat) {
		tf2::Quaternion q;
		tf2::convert(quat, q);
		double roll, pitch, yaw;
		tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
		return yaw;
	}
}

#endif

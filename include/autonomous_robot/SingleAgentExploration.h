#ifndef AUTONOMOUS_ROBOT_SINGLE_AGENT_EXPLORATION_H
#define AUTONOMOUS_ROBOT_SINGLE_AGENT_EXPLORATION_H

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <map_manager/occupancyMap.h>
#include <frontiers_manager/frontiers_manager.h>
#include <global_planner/dep.h>
#include <communication_manager/com_manager.h>
#include <autonomous_robot/spotPlatform.h>

namespace autonomous_robot {
	// Exploration state machine for Spot: runs the mapping, frontier and tour-planning modules, executes
	// the planned waypoints, triggers the 5-DoF posture at object viewpoints and decides when exploration ends.
	class saExploration {
	private:
		ros::NodeHandle nh_;
		std::shared_ptr<mapManager::occMap> map_;
		std::shared_ptr<frontiersManager::ftManager> ftManager_;
		std::shared_ptr<globalPlanner::DEP> explorer_;
		std::unique_ptr<SpotPlatform> robotPlatform_;
		std::unique_ptr<communication_manager::ComManager> comManager_;
		std::thread exploreReplanWorker_;
		ros::Timer plannerTimer_, trajExecuteTimer_, replanCheckTimer_;
		ros::Publisher postureScanPub_, vlmQueryPub_;
		ros::Subscriber vlmResponseSub_, coverageStatsSub_;
		mutable std::mutex explorationMutex_;

		// parameters
		double recoverMaxDist_ = 1.0;        // max distance of one recovery move (m)
		bool aggressiveMode_ = true;         // unknown space counts as free in recovery checks
		bool omniDirectionMode_ = false;     // true: the goal heading is not enforced at the path end
		double prearmReplanDist_ = 2.0;      // trigger the next global plan this far before the path end (m)
		double tspReplanExecDist_ = 8.0;     // force a global replan after this much executed path (m)
		bool enable6DofPostureScan_ = false; // posture (roll/pitch) at object viewpoints
		double postureScanDuration_ = 2.5;   // legacy fixed dwell (s), used when posture_sensor_settle_s < 0
		double postureSensorSettle_ = 0.0;   // sensor settle hold after the tilt ramp (s)
		double postureGainImproveThresh_ = 0.15;  // min relative gain over flat to tilt
		std::string postureTopic_ = "/spot/posture";
		int robotTypeParam_ = 0;             // sensor-model key in the frontier manager
		std::string postureMode_ = "geometric";   // "geometric" | "vlm" (external posture server, geometric fallback)
		std::string postureAimMode_ = "free";     // "free": keep arrival heading, aim with roll/pitch | "face": turn to the object
		std::string postureExecMode_ = "resweep"; // free aim: "sampled" executes the sampled posture, "resweep" re-optimises on arrival
		bool postureHonorPredecided_ = false;     // free aim: execute the session's pre-decided pitch
		std::string vlmQueryTopic_ = "/spot/vlm_posture_query";
		std::string vlmResponseTopic_ = "/spot/vlm_posture_response";
		double vlmResponseTimeout_ = 10.0;
		int samePathDoneThresh_ = 45;        // identical global paths near the path end before "done"
		double coveragePlateauDoneSec_ = 90.0;    // plateau window (s); <= 0 disables
		double explorationDoneKnownFracMin_ = 0.7;
		double explorationDoneGrowthFrac_ = 0.005;

		// posture state
		bool postureScanActive_ = false;     // holding a posture / aligning / waiting for the VLM
		ros::Time postureScanEndTime_;
		std::vector<Eigen::Vector3d> postureScannedVps_;  // object viewpoints already evaluated
		bool vlmQueryPending_ = false;
		ros::Time vlmQueryStamp_, vlmQueryDeadline_;
		double vlmFallbackRoll_ = 0.0, vlmFallbackPitch_ = 0.0, vlmFallbackYaw_ = 0.0;
		bool vlmFallbackDoPosture_ = false;
		bool vlmResponseReceived_ = false;
		geometry_msgs::Vector3Stamped vlmResponse_;  // x = roll, y = pitch, z = yaw (rad)
		bool vlmAlignActive_ = false;        // turning to face the object before the VLM query
		double vlmAlignYaw_ = 0.0;
		ros::Time vlmAlignDeadline_, vlmAlignSettleUntil_;
		bool vlmAlignSettled_ = false;

		// exploration state
		geometry_msgs::PoseStamped goal_, lastGoal_;
		bool lastGoalValid_ = false;
		bool explorationReplan_ = false;
		bool newWaypoints_ = false;
		int waypointIdx_ = 1;
		nav_msgs::Path waypoints_, lastWaypoints_;
		// left at its zero-initialised defaults, as in the validated configuration: the recovery and
		// backward checks below are therefore point checks at the robot centre
		communication_manager::RobotConfig robotConfig_;
		bool recoverMode_ = false;
		bool forceExecute_ = false;
		int samePathCount_ = 0;
		bool explorationDone_ = false;
		int recoverHoldCount_ = 0;
		Eigen::Vector3d lastProgressPos_ = Eigen::Vector3d::Zero();
		ros::Time lastProgressTime_;
		double noProgressTimeout_ = 12.0;    // s without 0.1 m of motion towards a goal
		std::deque<Eigen::Vector3d> breadcrumbs_;  // traversed trail for the starvation backtrack
		int replanFailCount_ = 0;
		bool backtrackActive_ = false;
		ros::Time escapeUntil_;              // an escape nudge is executing until then
		ros::Time lastReplanSuccessTime_;
		bool backtrackTriedSinceSuccess_ = false;
		double starveDoneTimeout_ = 120.0;   // s without a routable plan before "done" (after a backtrack)
		double lastKnownFrac_ = 0.0;
		std::deque<std::pair<ros::Time, double>> occHist_;  // (time, occupied voxels) from coverage_stats

		void initParams();
		void initModules();
		void registerPubs();
		void registerCallbacks();
		// Worker thread: runs the global planner whenever a replan is requested and adopts the result.
		void exploreReplan();
		void trajExecuterCB(const ros::TimerEvent&);
		// Termination checks, watchdogs, posture handling and waypoint adoption.
		void replanCheckCB(const ros::TimerEvent&);
		void plannerCB(const ros::TimerEvent&);
		void coverageStatsCB(const std_msgs::Float64MultiArrayConstPtr& msg);
		void vlmResponseCB(const geometry_msgs::Vector3StampedConstPtr& msg);
		// Posture hold in progress (align, VLM wait or dwell); true when the caller must keep holding.
		bool continuePostureHold();
		// Decides and commands the posture when the robot is at an unvisited object viewpoint; true when a hold started.
		bool triggerPostureAtObjectVp();
		// Dwell for a commanded tilt: ramp time plus the sensor settle hold.
		double postureDwell(double roll, double pitch, double yawDelta = 0.0) const;
		double computeWayptExeDist(const nav_msgs::Path& waypoints, int nextIdx);
		// Checks the straight segment from the goal back to the robot and shortens the goal before an obstacle.
		bool backwardTrajectorySafeCheck(bool& recover);
		// Recovery goal: bounded step towards the last goal, or rotate in place (and escape) when no plan exists yet.
		void recover2SafePos();
		void stop() { this->robotPlatform_->stop(); }
		nav_msgs::Odometry getCurrentOdom() const { return this->robotPlatform_->getOdom(); }

	public:
		explicit saExploration(const ros::NodeHandle& nh);
	};
}

#endif

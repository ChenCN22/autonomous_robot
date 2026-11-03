#include <autonomous_robot/SingleAgentExploration.h>
#include <autonomous_robot/utils.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace autonomous_robot {
	namespace {
		Eigen::Vector3d toEigen(const geometry_msgs::Point& p) { return Eigen::Vector3d(p.x, p.y, p.z); }
		double yawOf(const geometry_msgs::Quaternion& q) { return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)); }
		double wrapAngle(double a) { return atan2(sin(a), cos(a)); }
	}

	saExploration::saExploration(const ros::NodeHandle& nh) : nh_(nh) {
		this->initParams();
		this->initModules();
		this->registerPubs();
		this->registerCallbacks();
	}

	void saExploration::initParams() {
		this->nh_.param("recover_max_dist", this->recoverMaxDist_, 1.0);
		this->nh_.param("aggressive_mode", this->aggressiveMode_, true);
		this->nh_.param("omni_direction_mode", this->omniDirectionMode_, false);
		this->nh_.param("prearm_replan_dist", this->prearmReplanDist_, 2.0);
		this->nh_.param("tsp_replan_exec_dist", this->tspReplanExecDist_, 8.0);
		this->nh_.param("enable_6dof_posture_scan", this->enable6DofPostureScan_, false);
		this->nh_.param("posture_scan_duration", this->postureScanDuration_, 2.5);
		this->nh_.param("posture_sensor_settle_s", this->postureSensorSettle_, 0.0);
		this->nh_.param("posture_gain_improve_thresh", this->postureGainImproveThresh_, 0.15);
		this->nh_.param("posture_topic", this->postureTopic_, std::string("/spot/posture"));
		this->nh_.param("posture_mode", this->postureMode_, std::string("geometric"));
		this->nh_.param("posture_aim_mode", this->postureAimMode_, std::string("free"));
		this->nh_.param("posture_exec_mode", this->postureExecMode_, std::string("resweep"));
		this->nh_.param("posture_honor_predecided", this->postureHonorPredecided_, false);
		this->nh_.param("vlm_posture_query_topic", this->vlmQueryTopic_, std::string("/spot/vlm_posture_query"));
		this->nh_.param("vlm_posture_response_topic", this->vlmResponseTopic_, std::string("/spot/vlm_posture_response"));
		this->nh_.param("vlm_response_timeout", this->vlmResponseTimeout_, 10.0);
		this->nh_.param("exploration_done_same_path_repeats", this->samePathDoneThresh_, 45);
		this->nh_.param("exploration_done_coverage_plateau_s", this->coveragePlateauDoneSec_, 90.0);
		this->nh_.param("exploration_done_known_frac_min", this->explorationDoneKnownFracMin_, 0.7);
		this->nh_.param("exploration_done_growth_frac", this->explorationDoneGrowthFrac_, 0.005);
		if (this->postureExecMode_ != "resweep" and this->postureExecMode_ != "sampled") {
			ROS_WARN("[saExploration] unknown posture_exec_mode '%s', using 'resweep'", this->postureExecMode_.c_str());
			this->postureExecMode_ = "resweep";
		}
		if (this->postureAimMode_ != "free" and this->postureAimMode_ != "face") {
			ROS_WARN("[saExploration] unknown posture_aim_mode '%s', using 'face'", this->postureAimMode_.c_str());
			this->postureAimMode_ = "face";
		}
		if (this->postureMode_ != "geometric" and this->postureMode_ != "vlm") {
			ROS_WARN("[saExploration] unknown posture_mode '%s', using 'geometric'", this->postureMode_.c_str());
			this->postureMode_ = "geometric";
		}
		ROS_INFO("[saExploration] posture scan %s, mode %s, aim %s, exec %s, honor pre-decided %s",
		         this->enable6DofPostureScan_ ? "enabled" : "disabled", this->postureMode_.c_str(), this->postureAimMode_.c_str(),
		         this->postureExecMode_.c_str(), this->postureHonorPredecided_ ? "true" : "false");
	}

	void saExploration::initModules() {
		this->robotPlatform_ = std::make_unique<SpotPlatform>(this->nh_);
		this->comManager_ = std::make_unique<communication_manager::ComManager>(this->nh_);
		communication_manager::robotConfigMsg robotConfigMsg;
		this->robotPlatform_->setRobotConfigMsg(robotConfigMsg);
		std::unordered_map<int, communication_manager::RobotConfig> robotConfigs;
		robotConfigs[robotConfigMsg.robot_type] = this->comManager_->deriveRobotConfig(robotConfigMsg);
		if (not this->robotPlatform_->initialize()) {
			ROS_ERROR("[saExploration] robot platform initialisation failed");
			return;
		}

		this->map_ = std::make_shared<mapManager::occMap>(this->nh_);
		this->ftManager_ = std::make_shared<frontiersManager::ftManager>(this->nh_);
		this->ftManager_->setMap(this->map_);

		this->explorationReplan_ = true;
		this->explorer_ = std::make_shared<globalPlanner::DEP>(this->nh_);
		this->explorer_->setMap(this->map_);
		this->explorer_->loadVelocity(this->robotPlatform_->getDesiredVelocity(), this->robotPlatform_->getDesiredAngularVelocity());
		this->explorer_->setFtManager(this->ftManager_);
		this->explorer_->setRobotConfigs(robotConfigs);
		this->goal_.pose.position = this->getCurrentOdom().pose.pose.position;
	}

	void saExploration::registerPubs() {
		// also used by the cold-start rotation when the posture scan is disabled
		this->postureScanPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>(this->postureTopic_, 1);
		if (this->enable6DofPostureScan_ and this->postureMode_ == "vlm") {
			this->vlmQueryPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>(this->vlmQueryTopic_, 1);
			this->vlmResponseSub_ = this->nh_.subscribe(this->vlmResponseTopic_, 1, &saExploration::vlmResponseCB, this);
		}
	}

	void saExploration::registerCallbacks() {
		this->exploreReplanWorker_ = std::thread(&saExploration::exploreReplan, this);
		this->plannerTimer_ = this->nh_.createTimer(ros::Duration(0.1), &saExploration::plannerCB, this);
		this->trajExecuteTimer_ = this->nh_.createTimer(ros::Duration(0.05), &saExploration::trajExecuterCB, this);
		this->replanCheckTimer_ = this->nh_.createTimer(ros::Duration(0.1), &saExploration::replanCheckCB, this);
		this->coverageStatsSub_ = this->nh_.subscribe("coverage_stats", 1, &saExploration::coverageStatsCB, this);
	}

	void saExploration::coverageStatsCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
		// data = [known, free, occupied, total, known_frac]
		if (msg->data.size() < 3) return;
		if (msg->data.size() >= 5) this->lastKnownFrac_ = msg->data[4];
		this->occHist_.emplace_back(ros::Time::now(), msg->data[2]);
	}

	void saExploration::vlmResponseCB(const geometry_msgs::Vector3StampedConstPtr& msg) {
		if (this->vlmQueryPending_ and msg->header.stamp == this->vlmQueryStamp_) {
			this->vlmResponse_ = *msg;
			this->vlmResponseReceived_ = true;
		}
	}

	double saExploration::postureDwell(double roll, double pitch, double yawDelta) const {
		if (this->postureSensorSettle_ < 0.0) return this->postureScanDuration_;
		double angRate = std::max(0.2, this->robotPlatform_->getDesiredAngularVelocity());
		double rampT = std::max({std::abs(roll), std::abs(pitch), std::abs(yawDelta)}) / angRate + 0.3;
		return rampT + this->postureSensorSettle_;
	}

	void saExploration::exploreReplan() {
		ros::Rate rate(3);
		while (ros::ok()) {
			rate.sleep();
			ros::spinOnce();
			if (this->explorationDone_ or not this->explorationReplan_) continue;
			Eigen::Vector3d mapMin, mapMax;
			this->map_->getCurrMapRange(mapMin, mapMax);
			if (mapMax(0) - mapMin(0) < 0.1) continue;  // map not built yet
			this->explorer_->setMap(this->map_);
			bool replanSuccess = this->explorer_->makePlan();

			std::lock_guard<std::mutex> lock(this->explorationMutex_);
			this->lastWaypoints_ = this->waypoints_;
			if (replanSuccess) {
				this->waypoints_ = this->explorer_->getBestPath();
				if (this->waypoints_.poses.size() <= 1) {
					this->waypoints_.poses.clear();
					this->waypointIdx_ = 0;
					this->newWaypoints_ = false;
					continue;
				}
				this->newWaypoints_ = true;
				this->waypointIdx_ = 0;
				this->explorationReplan_ = false;
				this->recoverHoldCount_ = 0;
				this->replanFailCount_ = 0;
				this->backtrackActive_ = false;
				this->lastReplanSuccessTime_ = ros::Time::now();
				this->backtrackTriedSinceSuccess_ = false;

				// identical path: force execution instead of re-planning the same path forever
				bool pathIdentical = this->waypoints_.poses.size() > 1 and this->waypoints_.poses.size() == this->lastWaypoints_.poses.size();
				for (size_t i = 1; i < this->waypoints_.poses.size(); ++i) {
					if (i < this->lastWaypoints_.poses.size() and
					    this->waypoints_.poses[i].pose.position.x == this->lastWaypoints_.poses[i].pose.position.x and
					    this->waypoints_.poses[i].pose.position.y == this->lastWaypoints_.poses[i].pose.position.y and
					    this->waypoints_.poses[i].pose.position.z == this->lastWaypoints_.poses[i].pose.position.z) {
						ROS_WARN("new waypoints same as last waypoints, force execute new waypoints");
						this->forceExecute_ = true;
						continue;
					}
					pathIdentical = false;
				}
				// terminal oscillation: identical paths while the robot lives near the path end
				bool nearPathEnd = false;
				if (pathIdentical) {
					double dEnd = (toEigen(this->waypoints_.poses.back().pose.position) - toEigen(this->getCurrentOdom().pose.pose.position)).norm();
					nearPathEnd = dEnd < 6.0;
				}
				if (pathIdentical and nearPathEnd and not this->postureScanActive_) {
					if (++this->samePathCount_ >= this->samePathDoneThresh_) {
						this->explorationDone_ = true;
						ROS_WARN("[saExploration]: exploration finished: identical global path repeated %d consecutive times", this->samePathCount_);
					}
				} else if (not pathIdentical) {
					this->samePathCount_ = 0;
				}
			} else {
				this->replanFailCount_++;
				// starvation termination: backtracked to the trail origin and still no routable plan
				if (this->backtrackTriedSinceSuccess_ and this->lastKnownFrac_ > 0.5 and not this->lastReplanSuccessTime_.isZero() and
				    (ros::Time::now() - this->lastReplanSuccessTime_).toSec() > this->starveDoneTimeout_ and not this->breadcrumbs_.empty()) {
					Eigen::Vector3d pos = toEigen(this->getCurrentOdom().pose.pose.position);
					if ((pos - this->breadcrumbs_.front()).norm() < 1.5) {
						this->explorationDone_ = true;
						ROS_WARN("[saExploration]: exploration finished: planner starved %.0f s with no routable candidates, backtracked to origin",
						         (ros::Time::now() - this->lastReplanSuccessTime_).toSec());
						continue;
					}
				}
				if (this->backtrackActive_ and this->waypointIdx_ >= static_cast<int>(this->waypoints_.poses.size()) and
				    this->robotPlatform_->isReach(this->goal_, this->robotPlatform_->getReachGoalDistance(), false)) {
					this->backtrackActive_ = false;  // trail fully walked
				}
				if (this->backtrackActive_) {
					// the trail was preempted (no trajectory running and none pending): re-arm the backtrack
					if (not this->robotPlatform_->isTrajectoryReady() and not this->newWaypoints_ and not this->postureScanActive_) {
						ROS_WARN("[saExploration]: backtrack trail was preempted, re-arming starvation backtrack");
						this->backtrackActive_ = false;
					}
				} else if (this->replanFailCount_ >= 30 and this->breadcrumbs_.size() >= 2) {
					// persistent starvation: walk the traversed trail back towards its origin and keep retrying
					Eigen::Vector3d pos = toEigen(this->getCurrentOdom().pose.pose.position);
					int jNear = -1;
					double dBest = std::numeric_limits<double>::max();
					for (int j = 0; j < static_cast<int>(this->breadcrumbs_.size()); ++j) {
						double d = (this->breadcrumbs_[j] - pos).norm();
						if (d < dBest) {
							dBest = d;
							jNear = j;
						}
					}
					if (jNear >= 1) {
						nav_msgs::Path trail;
						for (int j = jNear; j >= 0; --j) {
							geometry_msgs::PoseStamped ps;
							ps.header.frame_id = "map";
							ps.pose.position.x = this->breadcrumbs_[j](0);
							ps.pose.position.y = this->breadcrumbs_[j](1);
							ps.pose.position.z = this->breadcrumbs_[j](2);
							ps.pose.orientation.w = 1.0;
							trail.poses.push_back(ps);
						}
						ROS_WARN("[saExploration]: replan starved %d times: backtracking trail from crumb %d to spawn (%.2f, %.2f)",
						         this->replanFailCount_, jNear, trail.poses.back().pose.position.x, trail.poses.back().pose.position.y);
						this->waypoints_ = trail;
						this->waypointIdx_ = 0;
						this->newWaypoints_ = true;
						this->explorationReplan_ = false;
						this->backtrackActive_ = true;
						this->backtrackTriedSinceSuccess_ = true;
						this->recoverMode_ = false;
						this->replanFailCount_ = 0;
						this->lastProgressTime_ = ros::Time::now();  // re-arm the progress watchdog
					} else {
						ROS_WARN_THROTTLE(5.0, "[saExploration]: replan starved at trail origin, recovering in place");
						this->recover2SafePos();
					}
				} else {
					ROS_WARN("[saExploration]: exploration replan failed, recover to safe position to replan again");
					this->recover2SafePos();
				}
			}
			rate.sleep();
		}
	}

	void saExploration::trajExecuterCB(const ros::TimerEvent&) {
		std::lock_guard<std::mutex> lock(this->explorationMutex_);
		if (this->postureScanActive_) return;  // a waypoint command would preempt the posture
		if (this->robotPlatform_->reachedGoal(this->goal_, this->robotPlatform_->getReachGoalDistance())) {
			this->recoverMode_ = false;
			this->stop();
			return;
		}
		if (not this->robotPlatform_->isTrajectoryReady()) return;
		this->robotPlatform_->executeTraj();
	}

	void saExploration::plannerCB(const ros::TimerEvent&) {
		std::lock_guard<std::mutex> lock(this->explorationMutex_);
		if (this->robotPlatform_->reachedGoal(this->goal_, this->robotPlatform_->getReachGoalDistance())) return;
		if (this->robotPlatform_->isTrajectoryReady()) return;
		nav_msgs::Path waypointPath;
		waypointPath.poses = {this->goal_};
		this->robotPlatform_->setTrajectory(waypointPath);
		this->robotPlatform_->setTrajectoryReady(true);
	}

	void saExploration::replanCheckCB(const ros::TimerEvent&) {
		std::lock_guard<std::mutex> lock(this->explorationMutex_);
		if (this->explorationDone_) {
			this->stop();
			return;
		}

		// coverage-plateau termination: known fraction above the gate and the occupied count grew
		// less than growth_frac over the last plateau_s window
		if (this->coveragePlateauDoneSec_ > 0 and not this->lastReplanSuccessTime_.isZero() and
		    this->lastKnownFrac_ > this->explorationDoneKnownFracMin_ and not this->occHist_.empty()) {
			ros::Time now = ros::Time::now();
			bool windowFull = (now - this->occHist_.front().first).toSec() >= this->coveragePlateauDoneSec_;
			double windowStartOcc = -1.0;
			for (const auto& e : this->occHist_) {
				if ((now - e.first).toSec() <= this->coveragePlateauDoneSec_) {
					windowStartOcc = e.second;
					break;
				}
			}
			double occNow = this->occHist_.back().second;
			double growth = (windowStartOcc > 0.0) ? (occNow - windowStartOcc) / windowStartOcc : 1.0;
			if (windowFull and windowStartOcc > 0.0 and growth < this->explorationDoneGrowthFrac_) {
				this->explorationDone_ = true;
				ROS_WARN("[saExploration]: exploration finished: coverage plateaued (known_frac %.3f > %.2f, occ grew %.3f%% < %.2f%% over %.0f s window, %.0f occupied voxels)",
				         this->lastKnownFrac_, this->explorationDoneKnownFracMin_, growth * 100.0, this->explorationDoneGrowthFrac_ * 100.0,
				         this->coveragePlateauDoneSec_, occNow);
				this->stop();
				return;
			}
		}

		// breadcrumb trail (paused while walking it)
		if (not this->backtrackActive_) {
			Eigen::Vector3d pos = toEigen(this->getCurrentOdom().pose.pose.position);
			if (this->breadcrumbs_.empty() or (pos - this->breadcrumbs_.back()).norm() > 0.5) {
				this->breadcrumbs_.push_back(pos);
				if (this->breadcrumbs_.size() > 400) this->breadcrumbs_.pop_front();
			}
		}

		// no-progress watchdog: a pending goal with no motion for noProgressTimeout_ triggers recovery and a replan;
		// a goal pinned twice in a row that is an object viewpoint is retired through the lookup's visited side effect
		if (not this->postureScanActive_ and not this->backtrackActive_ and not this->explorationReplan_ and not this->waypoints_.poses.empty()) {
			Eigen::Vector3d pos = toEigen(this->getCurrentOdom().pose.pose.position);
			if (this->lastProgressTime_.isZero() or (pos - this->lastProgressPos_).norm() > 0.10) {
				this->lastProgressPos_ = pos;
				this->lastProgressTime_ = ros::Time::now();
			} else if ((ros::Time::now() - this->lastProgressTime_).toSec() > this->noProgressTimeout_ and
			           not this->robotPlatform_->isReach(this->goal_, this->robotPlatform_->getReachGoalDistance(), false)) {
				ROS_WARN("[saExploration]: no-progress watchdog: pinned %.1f s short of goal (%.2f, %.2f), recover and replan",
				         (ros::Time::now() - this->lastProgressTime_).toSec(), this->goal_.pose.position.x, this->goal_.pose.position.y);
				static Eigen::Vector3d lastPinGoal(1e9, 1e9, 0.0);
				static int pinCount = 0;
				Eigen::Vector3d pinGoal(this->goal_.pose.position.x, this->goal_.pose.position.y, 0.0);
				if ((pinGoal.head<2>() - lastPinGoal.head<2>()).norm() < 0.5) {
					++pinCount;
				} else {
					pinCount = 1;
					lastPinGoal = pinGoal;
				}
				if (pinCount >= 2) {
					Eigen::Vector3d vpPos;
					double r, p, y;
					bool hs, pd;
					int oid = -1;
					if (this->ftManager_->lookupObjectVpPosture(pinGoal, 1.0, vpPos, r, p, y, hs, pd, oid)) {
						ROS_WARN("[saExploration]: pin-loop x%d on goal (%.2f, %.2f), retiring unreachable object VP (%.2f, %.2f)",
						         pinCount, pinGoal(0), pinGoal(1), vpPos(0), vpPos(1));
					}
					pinCount = 0;
				}
				this->lastProgressTime_ = ros::Time::now();
				this->recover2SafePos();
				this->explorationReplan_ = true;
				return;
			}
		}

		bool recover = false;
		this->backwardTrajectorySafeCheck(recover);

		// pre-arm replan: request the next global plan shortly before the path end while still moving
		if (not this->waypoints_.poses.empty() and this->waypointIdx_ >= static_cast<int>(this->waypoints_.poses.size()) and
		    not this->robotPlatform_->isReach(this->goal_, this->robotPlatform_->getReachGoalDistance(), false) and
		    not this->explorationReplan_ and not this->recoverMode_) {
			double distToEnd = (toEigen(this->goal_.pose.position) - toEigen(this->getCurrentOdom().pose.pose.position)).norm();
			if (distToEnd <= this->prearmReplanDist_) this->explorationReplan_ = true;
		}

		// inspection window for the object maps: mid-posture or within 1.5 m of an object viewpoint
		Eigen::Vector3d currPos = toEigen(this->getCurrentOdom().pose.pose.position);
		this->map_->setInspectionActive(this->postureScanActive_ or this->ftManager_->hasObjectVpWithin(currPos, 1.5));

		if (this->enable6DofPostureScan_ or this->postureScanActive_) {
			if (this->postureScanActive_) {
				if (this->continuePostureHold()) return;
			} else if (this->triggerPostureAtObjectVp()) {
				return;
			}
		} else {
			// posture scan disabled: still mark the object viewpoint as visited so the sampler retires it
			Eigen::Vector3d vpPos;
			double r, p, y;
			bool hs, pd;
			int oid = -1;
			this->ftManager_->lookupObjectVpPosture(currPos, this->robotPlatform_->getReachGoalDistance() + 0.3, vpPos, r, p, y, hs, pd, oid);
		}

		if (this->newWaypoints_ and not this->explorationReplan_) {
			ROS_INFO("[saExploration]: replan check: new waypoints");
			this->robotPlatform_->setTrajectoryReady(false);
			geometry_msgs::PoseStamped lastGoalTemp;
			lastGoalTemp.pose = this->getCurrentOdom().pose.pose;
			// skip leading waypoints the robot is already standing on (the path starts at the nearest grid node)
			while (this->waypointIdx_ + 1 < static_cast<int>(this->waypoints_.poses.size()) and
			       this->robotPlatform_->isReach(this->waypoints_.poses[this->waypointIdx_], 1.0, false)) {
				++this->waypointIdx_;
			}
			this->goal_ = this->waypoints_.poses[this->waypointIdx_];
			++this->waypointIdx_;
			if (this->robotPlatform_->isReach(this->goal_, this->robotPlatform_->getReachGoalDistance(), false)) {
				this->newWaypoints_ = false;
				return;
			}
			this->lastGoal_ = lastGoalTemp;
			this->lastGoalValid_ = true;
			this->newWaypoints_ = false;
			nav_msgs::Path waypointPath;
			waypointPath.poses = {this->goal_};
			this->robotPlatform_->setTrajectory(waypointPath);
			this->robotPlatform_->setTrajectoryReady(true);
		} else if (not this->waypoints_.poses.empty() and this->robotPlatform_->isReach(this->goal_, this->robotPlatform_->getReachGoalDistance(), false) and
		           this->waypointIdx_ < static_cast<int>(this->waypoints_.poses.size()) and not this->explorationReplan_) {
			// intermediate waypoint reached: execute the next one, or replan after a long executed stretch
			this->stop();
			this->robotPlatform_->setTrajectoryReady(false);
			if (this->waypointIdx_ >= 2 and not this->forceExecute_ and this->computeWayptExeDist(this->waypoints_, this->waypointIdx_) > this->tspReplanExecDist_) {
				this->explorationReplan_ = true;
				return;
			}
			geometry_msgs::PoseStamped lastGoalTemp;
			lastGoalTemp.pose = this->getCurrentOdom().pose.pose;
			this->goal_ = this->waypoints_.poses[this->waypointIdx_];
			if (this->robotPlatform_->isReach(this->goal_, this->robotPlatform_->getReachGoalDistance(), false)) {
				++this->waypointIdx_;
				return;
			}
			this->forceExecute_ = false;
			this->lastGoal_ = lastGoalTemp;
			this->lastGoalValid_ = true;
			nav_msgs::Path waypointPath;
			waypointPath.poses = {this->goal_};
			this->robotPlatform_->setTrajectory(waypointPath);
			this->robotPlatform_->setTrajectoryReady(true);
			++this->waypointIdx_;
		} else if (not this->waypoints_.poses.empty() and this->robotPlatform_->isReach(this->goal_, this->robotPlatform_->getReachGoalDistance(), false) and
		           this->waypointIdx_ >= static_cast<int>(this->waypoints_.poses.size()) and not this->explorationReplan_) {
			// path end reached: optionally wait for the goal heading, then replan
			if (not this->omniDirectionMode_ and not this->robotPlatform_->isReach(this->goal_, this->robotPlatform_->getReachGoalDistance(), true)) {
				this->robotPlatform_->setTrajectoryReady(true);
				return;
			}
			this->stop();
			this->explorationReplan_ = true;
		}
	}

	bool saExploration::continuePostureHold() {
		if (this->vlmAlignActive_) {
			// turning in place to face the object before the VLM query
			nav_msgs::Odometry curOdom = this->getCurrentOdom();
			double curYaw = yawOf(curOdom.pose.pose.orientation);
			double yawErr = std::fabs(wrapAngle(curYaw - this->vlmAlignYaw_));
			bool alignTimedOut = ros::Time::now() > this->vlmAlignDeadline_;
			if (not this->vlmAlignSettled_) {
				if (yawErr < 0.15 or alignTimedOut) {
					this->vlmAlignSettled_ = true;
					this->vlmAlignSettleUntil_ = ros::Time::now() + ros::Duration(0.5);  // let the camera publish a frame at the new heading
				}
				return true;
			}
			if (ros::Time::now() < this->vlmAlignSettleUntil_) return true;
			this->vlmAlignActive_ = false;
			geometry_msgs::PoseStamped query;
			query.header.stamp = ros::Time::now();
			query.header.frame_id = "map";
			query.pose.position = curOdom.pose.pose.position;
			query.pose.orientation = quaternion_from_rpy(0.0, 0.0, this->vlmFallbackYaw_);
			this->vlmQueryStamp_ = query.header.stamp;
			this->vlmQueryDeadline_ = query.header.stamp + ros::Duration(this->vlmResponseTimeout_);
			this->vlmResponseReceived_ = false;
			this->vlmQueryPending_ = true;
			this->vlmQueryPub_.publish(query);
			ROS_WARN("[saExploration]: VLM posture query @ (%.2f, %.2f) after align (yaw err %.1f deg%s), timeout %.1f s (geometric fallback: %s)",
			         curOdom.pose.pose.position.x, curOdom.pose.pose.position.y, yawErr * 180.0 / M_PI, alignTimedOut ? ", align TIMEOUT" : "",
			         this->vlmResponseTimeout_, this->vlmFallbackDoPosture_ ? "tilt" : "flat");
			return true;
		}
		if (this->vlmQueryPending_) {
			// holding at the object viewpoint while the VLM decides; geometric fallback on timeout
			double roll = 0.0, pitch = 0.0, yaw = 0.0;
			bool doPosture = false;
			if (this->vlmResponseReceived_) {
				this->vlmQueryPending_ = false;
				roll = this->vlmResponse_.vector.x;
				pitch = this->vlmResponse_.vector.y;
				yaw = this->vlmResponse_.vector.z;
				doPosture = std::fabs(roll) > 1e-3 or std::fabs(pitch) > 1e-3;
				ROS_WARN("[saExploration]: VLM posture response: roll/pitch = (%.1f, %.1f) deg%s", roll * 180.0 / M_PI, pitch * 180.0 / M_PI,
				         doPosture ? "" : " (flat -> skip tilt)");
			} else if (ros::Time::now() > this->vlmQueryDeadline_) {
				this->vlmQueryPending_ = false;
				roll = this->vlmFallbackRoll_;
				pitch = this->vlmFallbackPitch_;
				yaw = this->vlmFallbackYaw_;
				doPosture = this->vlmFallbackDoPosture_;
				ROS_WARN("[saExploration]: VLM response timeout (%.1f s) -> geometric fallback: roll/pitch = (%.1f, %.1f) deg%s",
				         this->vlmResponseTimeout_, roll * 180.0 / M_PI, pitch * 180.0 / M_PI, doPosture ? "" : " (flat)");
			} else {
				return true;
			}
			if (doPosture) {
				geometry_msgs::PoseStamped postureCmd;
				postureCmd.header.stamp = ros::Time::now();
				postureCmd.header.frame_id = "map";
				postureCmd.pose.position = this->getCurrentOdom().pose.pose.position;
				postureCmd.pose.orientation = quaternion_from_rpy(roll, pitch, yaw);
				this->postureScanPub_.publish(postureCmd);
				this->postureScanEndTime_ = ros::Time::now() + ros::Duration(this->postureDwell(roll, pitch));
				return true;
			}
			this->postureScanActive_ = false;
			return false;
		}
		if (ros::Time::now() < this->postureScanEndTime_) return true;  // dwelling
		this->postureScanActive_ = false;
		return false;
	}

	bool saExploration::triggerPostureAtObjectVp() {
		nav_msgs::Odometry currentOdom = this->getCurrentOdom();
		Eigen::Vector3d currPos = toEigen(currentOdom.pose.pose.position);
		Eigen::Vector3d vpPos;
		double bestRoll = 0.0, bestPitch = 0.0, vpYaw = 0.0;
		bool hasSampledPosture = false, posturePredecided = false;
		int vpObjectId = -1;
		double tol = this->robotPlatform_->getReachGoalDistance() + 0.3;
		if (not this->ftManager_->lookupObjectVpPosture(currPos, tol, vpPos, bestRoll, bestPitch, vpYaw, hasSampledPosture, posturePredecided, vpObjectId)) {
			return false;
		}
		for (const Eigen::Vector3d& done : this->postureScannedVps_) {
			if ((done - vpPos).norm() < 0.3) return false;  // each object viewpoint is evaluated once
		}
		this->postureScannedVps_.push_back(vpPos);

		bool doPosture = false;
		double postureYaw = vpYaw;  // face-object default
		double curYaw = yawOf(currentOdom.pose.pose.orientation);
		const bool freeAim = this->postureAimMode_ == "free" and this->postureMode_ != "vlm";
		if (freeAim and this->postureHonorPredecided_ and posturePredecided) {
			// execute the session's pre-decided pitch inside the free-aim kinematics
			if (std::abs(bestPitch) < 1e-3) {
				ROS_WARN("[saExploration]: aim mode=free (PREDECIDED flat) @ (%.2f, %.2f): posture skipped", vpPos(0), vpPos(1));
			} else {
				double predPitch = bestPitch;
				double cmdRoll = 0.0, cmdPitch = 0.0, cmdYaw = curYaw;
				this->ftManager_->aimPredecidedPosture(currPos, curYaw, vpObjectId, vpYaw, predPitch, cmdRoll, cmdPitch, cmdYaw);
				doPosture = true;
				bestRoll = cmdRoll;
				bestPitch = cmdPitch;
				postureYaw = cmdYaw;
				ROS_WARN("[saExploration]: aim mode=free (PREDECIDED pitch %.0f deg) @ (%.2f, %.2f): dYaw=%.0f deg, roll/pitch = (%.1f, %.1f) deg",
				         predPitch * 180.0 / M_PI, vpPos(0), vpPos(1), wrapAngle(cmdYaw - curYaw) * 180.0 / M_PI, bestRoll * 180.0 / M_PI, bestPitch * 180.0 / M_PI);
			}
		} else if (freeAim and this->postureExecMode_ == "sampled") {
			// execute the sampled posture from the arrival heading with the minimal yaw that respects the limits
			if (not hasSampledPosture) {
				ROS_WARN("[saExploration]: aim mode=free/sampled @ (%.2f, %.2f): sampled posture is flat, posture skipped", vpPos(0), vpPos(1));
			} else {
				double cmdRoll = 0.0, cmdPitch = 0.0, cmdYaw = curYaw;
				this->ftManager_->aimSampledPosture(currPos, curYaw, vpObjectId, vpYaw, bestRoll, bestPitch, cmdRoll, cmdPitch, cmdYaw);
				doPosture = true;
				ROS_WARN("[saExploration]: aim mode=free/sampled @ (%.2f, %.2f): beta=%.0f dYaw=%.0f deg, sampled (%.1f, %.1f) -> exec roll/pitch = (%.1f, %.1f) deg",
				         vpPos(0), vpPos(1), wrapAngle(vpYaw - curYaw) * 180.0 / M_PI, wrapAngle(cmdYaw - curYaw) * 180.0 / M_PI,
				         bestRoll * 180.0 / M_PI, bestPitch * 180.0 / M_PI, cmdRoll * 180.0 / M_PI, cmdPitch * 180.0 / M_PI);
				bestRoll = cmdRoll;
				bestPitch = cmdPitch;
				postureYaw = cmdYaw;
			}
		} else if (freeAim) {
			// re-optimise the gain at the arrival heading; minimal yaw only when the aim envelope cannot reach the object
			int aimGain = -1, aimFlat = -1;
			double cmdRoll = 0.0, cmdPitch = 0.0, cmdYaw = curYaw;
			if (this->ftManager_->selectAimPosture(currPos, curYaw, vpObjectId, vpYaw, cmdRoll, cmdPitch, cmdYaw, aimGain, aimFlat)) {
				doPosture = aimGain > aimFlat;
				bestRoll = cmdRoll;
				bestPitch = cmdPitch;
				postureYaw = cmdYaw;
				ROS_WARN("[saExploration]: aim mode=free @ (%.2f, %.2f): beta=%.0f dYaw=%.0f deg, roll/pitch = (%.1f, %.1f) deg, gain %d vs flat %d%s",
				         vpPos(0), vpPos(1), wrapAngle(vpYaw - curYaw) * 180.0 / M_PI, wrapAngle(cmdYaw - curYaw) * 180.0 / M_PI,
				         bestRoll * 180.0 / M_PI, bestPitch * 180.0 / M_PI, aimGain, aimFlat, doPosture ? "" : " (no gain, posture skipped)");
			} else {
				// object box unresolved: global sweep at the current heading
				int bestGain = -1, flatGain = -1;
				doPosture = this->ftManager_->findBestPosture(currPos, curYaw, bestRoll, bestPitch, bestGain, flatGain) and
				            flatGain >= 0 and bestGain > flatGain * (1.0 + this->postureGainImproveThresh_);
				postureYaw = curYaw;
				if (doPosture) {
					ROS_WARN("[saExploration]: aim mode=free (no bbox) @ (%.2f, %.2f): roll/pitch = (%.1f, %.1f) deg, gain %d vs flat %d",
					         currPos(0), currPos(1), bestRoll * 180.0 / M_PI, bestPitch * 180.0 / M_PI, bestGain, flatGain);
				}
			}
		} else if (hasSampledPosture) {
			// face mode: execute the sampling-time posture
			doPosture = true;
			ROS_WARN("[saExploration]: 5-DoF object VP posture @ (%.2f, %.2f): roll/pitch = (%.1f, %.1f) deg (sampled)",
			         vpPos(0), vpPos(1), bestRoll * 180.0 / M_PI, bestPitch * 180.0 / M_PI);
		} else {
			// arrival-time sweep on the global unknown gain
			int bestGain = -1, flatGain = -1;
			doPosture = this->ftManager_->findBestPosture(currPos, vpYaw, bestRoll, bestPitch, bestGain, flatGain) and
			            flatGain >= 0 and bestGain > flatGain * (1.0 + this->postureGainImproveThresh_);
			if (doPosture) {
				ROS_WARN("[saExploration]: posture scan @ (%.2f, %.2f): roll/pitch = (%.1f, %.1f) deg, gain %d vs flat %d, dwell %.1f s",
				         currPos(0), currPos(1), bestRoll * 180.0 / M_PI, bestPitch * 180.0 / M_PI, bestGain, flatGain, this->postureScanDuration_);
			}
		}

		if (this->postureMode_ == "vlm") {
			// keep the geometric answer as the timeout fallback, turn to face the object, then ask the VLM
			this->vlmFallbackDoPosture_ = doPosture;
			this->vlmFallbackRoll_ = bestRoll;
			this->vlmFallbackPitch_ = bestPitch;
			this->vlmFallbackYaw_ = vpYaw;
			geometry_msgs::PoseStamped alignCmd;
			alignCmd.header.stamp = ros::Time::now();
			alignCmd.header.frame_id = "map";
			alignCmd.pose.position = currentOdom.pose.pose.position;
			alignCmd.pose.orientation = quaternion_from_rpy(0.0, 0.0, vpYaw);
			this->postureScanPub_.publish(alignCmd);
			this->vlmAlignYaw_ = vpYaw;
			this->vlmAlignDeadline_ = ros::Time::now() + ros::Duration(12.0);
			this->vlmAlignSettled_ = false;
			this->vlmAlignActive_ = true;
			this->postureScanActive_ = true;
			ROS_WARN("[saExploration]: VLM align: turning to face object VP @ (%.2f, %.2f) before query", vpPos(0), vpPos(1));
			return true;
		}
		if (doPosture) {
			geometry_msgs::PoseStamped postureCmd;
			postureCmd.header.stamp = ros::Time::now();
			postureCmd.header.frame_id = "map";
			postureCmd.pose.position = currentOdom.pose.pose.position;
			postureCmd.pose.orientation = quaternion_from_rpy(bestRoll, bestPitch, postureYaw);
			this->postureScanPub_.publish(postureCmd);
			this->postureScanActive_ = true;
			this->postureScanEndTime_ = ros::Time::now() + ros::Duration(this->postureDwell(bestRoll, bestPitch, wrapAngle(postureYaw - curYaw)));
			return true;
		}
		return false;
	}

	double saExploration::computeWayptExeDist(const nav_msgs::Path& waypoints, int nextIdx) {
		double dist = 0.0;
		for (int i = 0; i < nextIdx - 1; ++i) {
			dist += (toEigen(waypoints.poses[i + 1].pose.position) - toEigen(waypoints.poses[i].pose.position)).norm();
		}
		dist += (toEigen(waypoints.poses[nextIdx - 1].pose.position) - toEigen(this->getCurrentOdom().pose.pose.position)).norm();
		return dist;
	}

	bool saExploration::backwardTrajectorySafeCheck(bool& recover) {
		if (this->recoverMode_ or (this->goal_.pose.position.x == 0.0 and this->goal_.pose.position.y == 0.0 and this->goal_.pose.position.z == 0.0)) {
			return true;
		}
		if (this->robotPlatform_->getTrajectory().poses.empty()) return true;

		nav_msgs::Odometry currentOdom = this->getCurrentOdom();
		Eigen::Vector3d pGoal = toEigen(this->goal_.pose.position);
		Eigen::Vector3d pCurr = toEigen(currentOdom.pose.pose.position);
		// heading along the segment; Spot does not turn for small heading changes, so keep the current heading then
		double yawTgt = atan2(pGoal(1) - pCurr(1), pGoal(0) - pCurr(0));
		Eigen::Quaterniond q(currentOdom.pose.pose.orientation.w, currentOdom.pose.pose.orientation.x,
		                     currentOdom.pose.pose.orientation.y, currentOdom.pose.pose.orientation.z);
		Eigen::Vector3d pGoalBody = q.toRotationMatrix().transpose() * (pGoal - pCurr);
		if (std::abs(atan2(pGoalBody(1), pGoalBody(0))) <= 0.3) {
			yawTgt = rpy_from_quaternion(currentOdom.pose.pose.orientation);
		}

		// walk from the goal back to the robot and stop before the first occupied check point
		Eigen::Vector3d diff = pCurr - pGoal;
		double dist = diff.norm();
		if (not std::isfinite(dist) or dist > 100.0) {
			ROS_WARN("[saExploration::backwardTrajectorySafeCheck]: absurd goal dist %.1f m, skip check", dist);
			return true;
		}
		const double stepSize = 0.1;
		Eigen::Vector3d unitIncrement = diff / dist * stepSize;
		int stepNum = static_cast<int>(dist / stepSize);
		bool isValid = true;
		Eigen::Vector3d goalSafeVec = pGoal;
		for (int i = 0; i < stepNum; ++i) {
			Eigen::Vector3d pCheck = pGoal + i * unitIncrement;
			if (this->map_->isCollisionBoxOccupied(pCheck, yawTgt, this->robotConfig_.safeDist, this->robotConfig_.collisionBoxMin, this->robotConfig_.collisionBoxMax)) {
				goalSafeVec = pCheck + unitIncrement;
				isValid = false;
				break;
			}
		}
		if (isValid) return true;

		geometry_msgs::PoseStamped goalSafe = this->goal_;
		goalSafe.pose.position.x = goalSafeVec(0);
		goalSafe.pose.position.y = goalSafeVec(1);
		goalSafe.pose.position.z = goalSafeVec(2);
		this->robotPlatform_->setTrajectoryReady(false);
		this->goal_ = goalSafe;
		if (this->robotPlatform_->isReach(goalSafe, this->robotPlatform_->getReachGoalDistance(), false)) {
			ROS_WARN("[saExploration]: backward path check only allows a goal that is already reached");
			this->stop();
			recover = true;
			this->robotPlatform_->setTrajectoryReady(false);
			return false;
		}
		// new two-pose path: current pose -> shortened goal
		this->waypoints_.poses.clear();
		geometry_msgs::PoseStamped current;
		current.pose = currentOdom.pose.pose;
		this->waypoints_.poses.push_back(current);
		this->waypoints_.poses.push_back(this->goal_);
		this->waypointIdx_ = 0;
		this->newWaypoints_ = true;
		this->robotPlatform_->setTrajectoryReady(false);
		this->robotPlatform_->clearTrajectory();
		this->stop();
		ROS_WARN("[saExploration::backwardTrajectorySafeCheck]: backward trajectory check choose: new goal %f %f %f",
		         this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
		return true;
	}

	void saExploration::recover2SafePos() {
		if (ros::Time::now() < this->escapeUntil_) return;            // an escape nudge is executing
		if (this->backtrackActive_ or this->newWaypoints_) return;    // a trail or a fresh plan is waiting to execute

		nav_msgs::Odometry currentOdom = this->getCurrentOdom();
		Eigen::Vector3d pGoal = toEigen(this->lastGoal_.pose.position);
		Eigen::Vector3d pCurr = toEigen(currentOdom.pose.pose.position);
		Eigen::Vector3d diff = pGoal - pCurr;
		double dist = diff.norm();
		if (not this->lastGoalValid_ or dist < 1e-3) {
			// no valid plan yet: hold position but rotate in place so the sensors sweep the surroundings
			this->goal_.pose.position = currentOdom.pose.pose.position;
			this->goal_.pose.orientation = currentOdom.pose.pose.orientation;
			geometry_msgs::PoseStamped rotCmd;
			rotCmd.header.stamp = ros::Time::now();
			rotCmd.header.frame_id = "map";
			rotCmd.pose.position = currentOdom.pose.pose.position;
			rotCmd.pose.orientation = quaternion_from_rpy(0.0, 0.0, yawOf(currentOdom.pose.pose.orientation) + 0.8);
			this->postureScanPub_.publish(rotCmd);
			this->postureScanActive_ = true;  // suppress the trajectory executor, which would preempt the rotation
			this->postureScanEndTime_ = ros::Time::now() + ros::Duration(1.0);
			ROS_WARN_THROTTLE(5.0, "[saExploration]: cold start: rotating in place to scan surroundings (no valid plan yet)");

			// escape nudge: parked where the roadmap start is invalid, step 0.7 m towards inflated-free space
			if (++this->recoverHoldCount_ >= 15) {
				const double escStep = 0.7;
				bool escFound = false;
				for (int k = 0; k < 8; ++k) {
					double ang = k * M_PI / 4.0;
					Eigen::Vector3d pCand = pCurr + escStep * Eigen::Vector3d(cos(ang), sin(ang), 0.0);
					if (this->map_->isInflatedFree(pCand) and this->map_->isInflatedFreeLine(pCurr, pCand)) {
						this->goal_.pose.position.x = pCand(0);
						this->goal_.pose.position.y = pCand(1);
						this->goal_.pose.position.z = pCurr(2);
						this->goal_.pose.orientation = quaternion_from_rpy(0.0, 0.0, atan2(pCand(1) - pCurr(1), pCand(0) - pCurr(0)));
						this->escapeUntil_ = ros::Time::now() + ros::Duration(5.0);
						this->postureScanActive_ = false;
						this->recoverHoldCount_ = 0;
						escFound = true;
						ROS_WARN("[saExploration]: recover escape nudge: stepping %.1f m to (%.2f, %.2f)", escStep, pCand(0), pCand(1));
						break;
					}
				}
				if (not escFound) ROS_WARN_THROTTLE(5.0, "[saExploration]: escape nudge: all 8 directions blocked, keep rotating");
			}
		} else {
			// bounded step along the ray towards the last goal; a near goal is pushed past until the check hits something
			Eigen::Vector3d diffUnit = diff / dist;
			double yaw = atan2(diff(1), diff(0));
			const double stepSize = 0.2;
			double targetDist = std::min(dist, this->recoverMaxDist_);
			if (dist < this->recoverMaxDist_) {
				for (double d = dist + stepSize; d <= this->recoverMaxDist_ + 1e-6; d += stepSize) {
					Eigen::Vector3d pCheck = pCurr + d * diffUnit;
					bool occupied;
					if (this->aggressiveMode_) {
						occupied = this->map_->isCollisionBoxOccupied(pCheck, yaw, this->robotConfig_.safeDist, this->robotConfig_.collisionBoxMin, this->robotConfig_.collisionBoxMax);
					} else {
						occupied = not this->map_->isCollisionBoxFree(pCheck, yaw, this->robotConfig_.safeDist, this->robotConfig_.collisionBoxMin, this->robotConfig_.collisionBoxMax);
					}
					if (occupied) break;
					targetDist = d;
				}
			} else {
				this->goal_ = this->lastGoal_;  // keep its heading; the position is capped below
				ROS_WARN("[saExploration::recover2SafePos]: last goal %.2f m away > recover_max_dist %.2f m, stepping %.2f m toward it",
				         dist, this->recoverMaxDist_, targetDist);
			}
			Eigen::Vector3d pSafe = pCurr + targetDist * diffUnit;
			this->goal_.pose.position.x = pSafe(0);
			this->goal_.pose.position.y = pSafe(1);
			this->goal_.pose.position.z = pSafe(2);
		}

		// two-pose recovery path: current pose -> goal
		this->waypoints_.poses.clear();
		geometry_msgs::PoseStamped current;
		current.pose = currentOdom.pose.pose;
		this->waypoints_.poses.push_back(current);
		this->waypoints_.poses.push_back(this->goal_);
		this->waypointIdx_ = 0;
		this->newWaypoints_ = false;
		this->recoverMode_ = true;
		this->robotPlatform_->setTrajectoryReady(false);
		this->stop();
		ROS_WARN("[saExploration]: from %f %f %f recover to safe position %f %f %f", pCurr(0), pCurr(1), pCurr(2),
		         this->goal_.pose.position.x, this->goal_.pose.position.y, this->goal_.pose.position.z);
	}
}

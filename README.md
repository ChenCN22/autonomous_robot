# autonomous_robot

Exploration state machine of the POSE stack (*5-DoF Legged Robot Semantic Exploration and Inspection in
Crowded Unknown Environments*) for a Boston Dynamics Spot. The node owns the mapping (`map_manager`),
frontier / object-viewpoint (`frontiers_manager`) and tour-planning (`global_planner`) modules, executes the
planned waypoints, triggers the 5-DoF posture at object viewpoints and decides when exploration ends.

## Components

| Class | File | Role |
|---|---|---|
| `saExploration` | `SingleAgentExploration.h/.cpp` | Replan worker thread, waypoint execution, posture handling, recovery and termination |
| `SpotPlatform` | `spotPlatform.h/.cpp` | Odometry in, waypoint poses out, robot configuration for the other modules |

## Behaviour

- **Replanning**: a worker thread runs `DEP::makePlan` whenever a replan is requested. Requests come from the
  path end, a pre-arm distance before it (`prearm_replan_dist`), a long executed stretch (`tsp_replan_exec_dist`),
  the no-progress watchdog and failed recoveries.
- **Execution**: one waypoint at a time is published on `cmd_topic`; a backward line check shortens goals that
  run into obstacles; a bounded recovery move (`recover_max_dist`) or an in-place rotation handles failures;
  a breadcrumb backtrack handles planner starvation.
- **Posture at object viewpoints** (`enable_6dof_posture_scan`): on arrival at an unvisited object viewpoint
  the roll/pitch is chosen by the frontier manager (`posture_aim_mode`, `posture_exec_mode`,
  `posture_honor_predecided`) or asked from an external VLM server (`posture_mode: vlm`, geometric fallback on
  timeout) and published on `posture_topic` for the dwell time.
- **Termination**: identical global paths repeated near the path end, planner starvation after a backtrack,
  or a coverage plateau from `coverage_stats` (`exploration_done_*`).

## Launch

```
roslaunch autonomous_robot sa_exploration_spot.launch
```

`cfg/exploration_spot/` holds the parameters of all four modules, loaded into the robot namespace:
`robot_config.yaml` (platform and state machine), `exploration_param.yaml` (tour planner),
`frontiers_param.yaml` (frontiers and object viewpoints), `multi_robo_occ_map_param.yaml` (map and sensors).

## Topics

| Topic | Type | Direction |
|---|---|---|
| `odom_topic` | `nav_msgs/Odometry` | in |
| `cmd_topic` | `geometry_msgs/PoseStamped` | out, waypoint for the Spot controller |
| `posture_topic` | `geometry_msgs/PoseStamped` | out, body roll/pitch/yaw command |
| `coverage_stats` | `std_msgs/Float64MultiArray` | in, from `map_manager` |
| `vlm_posture_query_topic` / `vlm_posture_response_topic` | `geometry_msgs/PoseStamped` / `Vector3Stamped` | VLM posture server (optional) |

## Build

```
cd ~/catkin_ws/src && git clone <this repo> && cd .. && catkin_make
```
Depends on `map_manager`, `frontiers_manager`, `global_planner`, `communication_manager`, `tf2` and Eigen3.

## License
MIT, see `LICENSE`.

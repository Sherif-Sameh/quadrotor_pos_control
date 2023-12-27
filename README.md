# Quadrotor Postion Control

---

## Summary

This repository contains the source code for the ROS nodes used to perform trajectory tracking for a quadrotor based on the F450 frame, while using ArduCopter's internal attitude controller for tracking desired orientations. Additionally, it contains some additional utilities needed to setup the simulation environment as well as a step-by-step tutorial on setting up the simulator.

## System Overview

For the SITL Gazebo simulation, the ROS environment is made up of two main nodes, namely:
 
* SMC Controller/High-gain Observer
* Trajectory Generator

Moreover, two more nodes are needed, those being:

* MAVROS
* Model Pose Publisher

## First-time Setup:

**Note:** QGroundControl can be used to set the following parameters (just launch it while the simulator is running and it will connect automatically) **OR** MAVProxy can be used to set parameters as follows:

param set param_name param_value

param fetch param_name (to check that it was set correctly)

1. Follow the instructions available in the `install_instructions.md` file in this repo to setup the simulation environment/install any additional dependencies.
2. Mirror the parameters of the attitude controller (ATC) in SITL to those of the ATC on the real drone. For this project, those parameters can be found in the images directory.
3. Set the `GUID_OPTIONS` parameter to 8. This ensures that thrust values send to the ATC are interpreted as thrust and not desired acceleration, which is the default behavior. 

## Launch Commands

Launch Ardupilot SITL in the 1st terminal: `sim_vehicle.py -v ArduCopter -f gazebo-iris --mavproxy-args="--streamrate=-1"`

**Note:** Setting `streamrate=-1` in MAVProxy ensures that the GCS doesn't override the streaming rates set from the MAVROS launch file.

Launch the ROS environment + Gazebo in the 2nd terminal: `roslaunch test_package system.launch`

## 1- SMC Controller/High-gain Observer node:

### Overview
The **SMC Controller/High-gain Observer** node implements, as shown by its name, the SMC position controller as well as the High-gain observer used for estimating the 6 linear states (3 positions and 3 linear velocities). Additonally, as the observer is a continuous-time observer; it must be discretized, which is handled internally inside the node using the 4th order Runga Kutta technique.

### Advertised Topics
1. **Name:** `/mavros/setpoint_raw/attitude`, **Type:** mavros_msgs/AttitudeTarget, **Description:** Publish thrust and desired orientation.
2. **Name:** `/iris_drone/state_estimate`, **Type:** test_package/FullState, **Description:** Publish estimated state for evaluating estimate.

### Subscribed Topics
1. **Name:** `/mavros/imu/data`, **Type:** sensor_msgs/Imu, **Description:** Get current orientation.
2. **Name:** `/iris_demo/pose`, **Type:** geometry_msgs/PoseStamped, **Description:** Get current position.
3. **Name:** `/trajectory_generator/trajectory`, **Type:** test_package/DroneTrajectory, **Description:** Get desired trajectory.

### Additional Information
The normal loop for this nodes upon receiving the latest position measurements ensues as the following:

1. Update the estimated states based on the latest orientation and position measurements using the High-gain observer's equations and the 4th order Runga Kutta technique
2. Calculate the values of the sliding surfaces' variables (Sx, Sy and Sz) based on the tracking errors and the SMC controller's parameters
3. Calculate the total thrust input U1 and the two virtual inputs Ux and Uy
4. Convert the virtual inputs Ux and Uy into desired roll and pitch angles, then perform a transformation for the desired orientation from the inertial frame of Gazebo/mocap system into the inertial frame of Ardupilot.
5. Publish the thrust as well as the desired orientation to MAVROS

**Note:** New orientations and desired trajectories are stored into global varaibles upon arrival to be used by the control loop upon the arrival of new positions.

## 2- Trajectory Generator

### Overview
The **Trajectory Generator** node implements a state machine composed of 5 distinct states for generating different trajectories and handling the different requirements of  transitioning between the different states such as arming or disarming motors. The implementation is based on MatLab StateFlow and uses user-triggered events to trigger some transitions between the different states. The five states are labelled as follows:

* Inactive
* Take-off
* Hover
* Trajectory_Tracking
* Landing

**Note:** Only transiitons to the Take-off, Trajectory_Tracking and Landing states must be triggered by the user through the appropriate ROS Service, other transitions are automatically triggered once their transition guards are bypassed.

### Advertised Topics
1. **Name:** `/trajectory_generator/trajectory` **Type:** test_package/DroneTrajectory, **Description:** Publish desired trajectory which includes: position, linear velocity, linear acceleation and desired yaw angle.

### Advertised Services
1. **Name:** `/trajectory_generator/set_mode`, **Type:** std_srvs/Empty, **Description:** Used to request mode transitions from the state-machine.
2. **Name:** `/trajectory_generator/abort_mission` **Type:** test_package/StringSrv, **Description:** Used to request cancellation of the current mission (Taking-off, Traj. Tracking or Landing).

### Subscribed Topics
1. **Name:** `/iris_demo/pose`, **Type:** geometry_msgs/PoseStamped, **Description:** Get current position.

### Service Clients
1. **Name:** `/mavros/cmd/arming`, **Type:** mavros_msgs/CommandBool, **Description:** Request arming or disarming of motors.
2. **Name:** `/mavros/set_mode`, **Type:** mavros_msgs/SetMode, **Description:** Request mode change from Ardupilot (Stabilize, Guided_NOGPS, etc.).

### Additional Information

The available "set_mode" ROS service can be used from the CLI using the ROSService tool where the only input argument to the service is a string which is parsed and interpreted internally by the different modes. 

To take-off:

* rosservice call /trajectory_generator/set_mode "Take-off Delta_z (m) Duration (sec)"

For example: `rosservice call /trajectory_generator/set_mode "Take-off 0.5 5.0"` would request moving to Z = Z_initial + 0.5 (m) with the total maneuver taking 5.0 seconds.

To land:

* rosservice call /trajectory_generator/set_mode "Landing Final_Z(m) Duration (sec)"

For example: `rosservice call /trajectory_generator/set_mode "Landing 0.1 8.0"` would request moving to Z = 0.1 (m) with the total maneuver taking 8.0 seconds.

To track a trajectory:

* rosservice call /trajectory_generator/set_mode "Trajectory_Tracking helical Helix_radius (m) Theta_init (rad) Delta_theta (rad) Delta_Z (m) Duration (sec)"

For example: `rosservice call /trajectory_generator/set_mode "Trajectory_Tracking helical 1.0 0.0 6.2832 0.5 7.5"` would request to track a helical trajectory whose radius equals to 1.0m, where the drone is initially situated on the helix at theta = 0.0 rad, the total rotation is equal to 1 full rotation (2*Pi), the final Z = Z_curr + 0.5 (m) and the whole manuever taking 7.5 seconds.

**OR**

* rosservice call /trajectory_generator/set_mode "Trajectory_Tracking min_snap Num_of_waypoints X1 Y1 Z1 X2 Y2 Z2 .... XN YN ZN T1 T2 .... TN"

For example: `rosservice call /trajectory_generator/set_mode "Trajectory_Tracking min_snap 3 5.0 5.0 5.0 8.0 2.0 7.0 3.0 0.0 3.0 5.0 10.0 15.0"` would request to track the minimum snap trajectory which starts from the current position and passes though the 3 defined waypoints: P1(5.0,5.0,5.0), P2(8.0,2.0,7.0) and P3(3.0,0.0,3.0). Morever, the trajectory must equal those waypoints at their repsective timestamps T1=5.0s, T2=10.0s and T3=15.0s, where this elapsed time is calculated from the time the trajectory is initiated. Lastly, all derivatives whose order is 1 or greater must be equal to 0.0 at the two endpoints (the starting position and the final waypoint).

**Note 1:** Minimum snap trajectories are calculated using the C++ library available at the following repository: https://github.com/icsl-Jeon/traj_gen.git

**Note 2:** This feature is currently not available when testing on the real drone as the minimum snap library requires C++14 at least, which is not supported by the g++ compiler available on the raspberry pi. The version of the gcc and g++ compilers available is 8.3 which **does not** support C++14 or newer.

## 3- MAVROS

### Overview
The **MAVROS** node is responsible for providing a communication link between the ROS environment and Ardupilot. Specifically, within this project, it is used to retrieve the orientation estimates from the EKF within Ardupilot, send control commands to the attitude controller of Ardupilot and to request certain services such as mode changes, or arming/disarming requests.

### Used Plugins
* **Name:** `sys_status`, **Description:** Essential plugin which provides updates on the FCU's state as well as services to change mode or set the streaming rate of other topics. 
* **Name:** `sys_time`, **Description:** Essential plugin used for time synchronization.
* **Name:** `imu`, **Description:** Provides both orientation + scaled imu readings and raw imu readings only.
* **Name:** `setpoint_raw`, **Description:** Provides the ability to send desired setpoints.
* **Name:** `command`, **Description:** Provides a set of ROS services for sending MAVLink Commands to Ardupilot as well as other services for arming/disarming, setting home position, etc.

## 4- Model Pose Publisher

### Overview
The **model_pose_publisher** node is a simple ROS node that replaces the real Mocap system within the simulation environment by retreiving the pose of the model in Gazebo and publishing it through a ROS topic at a fixed rate.

### Advertised Topics
1. **Name:** `/iris_demo/pose`, **Type:** geometry_msgs/PoseStamped, **Description:** Publish the model's current pose.
2. **Name:** `/iris_demo/twist`, **Type:** geometry_msgs/TwistStamped, **Description:** Publish the model's current twist (for evaluating the state estimate only).

### Service Clients
1. **Name:** `/gazebo/get_model_state`, **Type:** gazebo_msgs/GetModelState, **Description:** Request the model's current state (pose + twist) from Gazebo.

## To-do List:

* Setup a new branch for the real drone which varies slightly from the SITL setup
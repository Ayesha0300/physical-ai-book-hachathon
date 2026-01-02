# Chapter 4 Exercises: ROS 2 Nodes in Practice

## Exercise 4.1: Node Responsibility Design

**Difficulty**: Basic

Design a node for each of the following functions. For each node, specify:
- Primary responsibility
- Topics it would publish and subscribe to
- Services it would provide or call
- Why it should be single-purpose or multi-purpose

A) Camera image processing
B) Joint state aggregation
C) Robot health monitoring
D) Path planning

**Solution**:
A) Image Processing Node
- Responsibility: Process raw camera images for object detection
- Publish: /image_processed, /object_detections
- Subscribe: /camera/image_raw
- Single-purpose for focused functionality and testability

B) Joint State Aggregator
- Responsibility: Collect and aggregate joint states from multiple controllers
- Publish: /joint_states (standard message)
- Subscribe: /left_arm/joint_states, /right_arm/joint_states, etc.
- Single-purpose for reliability and standardization

C) Health Monitoring Node
- Responsibility: Monitor system health and publish alerts
- Publish: /system_alerts, /health_status
- Subscribe: various system status topics
- Single-purpose for critical safety function

D) Path Planning Node
- Responsibility: Generate navigation paths from goals
- Provide: /plan_path service
- Subscribe: /map, /robot_pose, /goal_pose
- Single-purpose for focused planning algorithm

## Exercise 4.2: Node Type Classification

**Difficulty**: Basic

Classify each of the following nodes as sensor, control, or AI node, and explain your reasoning:

A) IMU driver node
B) Navigation planner
C) Joint trajectory controller
D) Object detection node
E) Camera driver
F) Behavior manager

**Solution**:
A) Sensor Node - Interfaces with physical IMU hardware and publishes sensor data
B) AI Node - Implements intelligent planning algorithms
C) Control Node - Controls joint actuators based on trajectory commands
D) AI Node - Implements intelligent perception algorithms
E) Sensor Node - Interfaces with physical camera hardware
F) AI Node - Manages high-level robot behaviors and decision making

## Exercise 4.3: Data Flow Analysis

**Difficulty**: Intermediate

For a humanoid robot walking task, identify the data flow from sensors to actuators. List the nodes involved and the topics passed between them.

**Solution**:
1. Sensors: IMU, joint encoders, force/torque sensors
   - Publish: /sensors/imu, /joint_states, /ft_sensors/force_torque

2. State Estimation: Robot state estimation node
   - Subscribe: sensor topics
   - Publish: /robot_state, /center_of_mass, /balance_state

3. Walking Pattern Generation: Walking controller
   - Subscribe: /robot_state, /walking_goal
   - Publish: /walking_pattern, /foot_trajectories

4. Trajectory Planning: Trajectory generator
   - Subscribe: /walking_pattern
   - Publish: /joint_trajectories, /balance_adjustments

5. Balance Control: Balance controller
   - Subscribe: /joint_trajectories, /balance_state
   - Publish: /corrected_trajectories

6. Low-level Control: Joint controllers
   - Subscribe: /corrected_trajectories
   - Publish: commands to actuators

## Exercise 4.4: Fault Isolation Design

**Difficulty**: Intermediate

Design a fault isolation strategy for a mobile robot system with the following nodes:
- Camera sensor node
- Navigation planner
- Path follower
- Mobile base controller
- Battery monitor

Explain how failures in each node would be isolated and handled.

**Solution**:
- Camera node failure: Navigation system switches to alternative sensors (LiDAR) or degraded mode
- Navigation planner failure: Robot stops safely and waits for recovery or manual control
- Path follower failure: Robot stops and alerts system, path planning can be retried
- Mobile base controller failure: Emergency stop activated, robot enters safe state
- Battery monitor failure: System uses backup power estimation, alerts operator

Each node runs in separate process, so failures don't cascade. Clear interfaces allow graceful degradation.

## Exercise 4.5: Node Architecture Pattern

**Difficulty**: Advanced

Design a node architecture for a pick-and-place task using a robotic arm. Specify the nodes needed, their responsibilities, and the communication between them.

**Solution**:
1. Camera Node: Publishes /camera/image_raw
2. Object Detection Node: Subscribes to image, publishes /detected_objects
3. Pick Planning Node: Subscribes to objects, publishes /pick_pose
4. Arm Controller Node: Subscribes to pick pose, executes pick action
5. Grasp Verification Node: Checks grasp success, publishes /grasp_status
6. Place Planning Node: Subscribes to grasp status, publishes /place_pose
7. Place Controller Node: Executes placement action

Each node has clear inputs/outputs and can be tested independently.

## Exercise 4.6: Multi-Purpose vs Single-Purpose Decision

**Difficulty**: Advanced

For each scenario, decide whether to use multi-purpose or single-purpose nodes and justify your choice:

A) A small robot with limited computational resources
B) A safety-critical application
C) A research robot with rapidly changing algorithms
D) A production robot requiring maximum reliability

**Solution**:
A) Multi-purpose - To reduce resource overhead and process management
B) Single-purpose - To isolate safety-critical functions and prevent failure propagation
C) Single-purpose - To allow independent development and testing of changing algorithms
D) Single-purpose - To maximize fault isolation and system reliability
# Chapter 3 Exercises: Communication Primitives in ROS 2

## Exercise 3.1: Communication Pattern Matching

**Difficulty**: Basic

Match each robotic scenario with the most appropriate communication pattern (Topic, Service, or Action) and explain why:

A) Camera publishing images for computer vision processing
B) Requesting robot to move to a specific location
C) Continuous joint position feedback for control
D) Requesting current robot battery level
E) Performing a complex manipulation task
F) Publishing laser scan data for navigation
G) Requesting emergency stop

**Solution**:
A) Topic - Continuous streaming of image data
B) Action - Long-running task with potential for feedback and cancellation
C) Topic - Continuous streaming of sensor data
D) Service - Simple request-response for current status
E) Action - Long-running operation with feedback and potential cancellation
F) Topic - Continuous streaming of sensor data
G) Service - Immediate, guaranteed execution of command

## Exercise 3.2: Pattern Characteristics Comparison

**Difficulty**: Basic

Complete the following table comparing the three communication patterns:

| Pattern | Synchronization | Duration | Feedback | Cancelable |
|---------|----------------|----------|----------|------------|
| Topic   | ?              | ?        | ?        | ?          |
| Service | ?              | ?        | ?        | ?          |
| Action  | ?              | ?        | ?        | ?          |

**Solution**:
| Pattern | Synchronization | Duration | Feedback | Cancelable |
|---------|----------------|----------|----------|------------|
| Topic   | Asynchronous    | Continuous| None    | No         |
| Service | Synchronous     | Immediate | One response| No       |
| Action  | Asynchronous    | Long-running| Continuous| Yes     |

## Exercise 3.3: Humanoid Robot Communication Design

**Difficulty**: Intermediate

Design the communication architecture for a humanoid robot performing the following tasks. For each task, specify the appropriate communication pattern and explain your choice:

1. Head tracking a moving object
2. Walking to a specific location
3. Grasping an object
4. Updating robot configuration parameters
5. Monitoring robot health status
6. Performing a dance routine

**Solution**:
1. Topic - Continuous streaming of object position to head controller
2. Action - Long-running task with feedback on progress, can be canceled
3. Action - Complex manipulation with feedback and potential for interruption
4. Service - Simple parameter update with confirmation response
5. Topic - Continuous streaming of health data for monitoring
6. Action - Long-running sequence with feedback and cancel capability

## Exercise 3.4: QoS Considerations for Topics

**Difficulty**: Intermediate

For each topic scenario, explain the appropriate QoS settings and why:

A) IMU sensor data for balance control
B) Camera images for SLAM
C) Joint position feedback for control
D) Diagnostic information for monitoring
E) Map updates for navigation

**Solution**:
A) Reliable, keep last - Balance control needs guaranteed delivery and only latest data
B) Best effort, keep last few - SLAM can work with some dropped frames, needs recent images
C) Reliable, keep last - Control systems need guaranteed delivery of latest joint positions
D) Best effort, keep all - Monitoring can work with best effort, may want all diagnostic data
E) Reliable, transient local - Navigation needs guaranteed delivery, new nodes need current map

## Exercise 3.5: Service vs. Action Decision Tree

**Difficulty**: Advanced

Create a decision tree that helps determine whether to use a Service or Action for a given robotic task. Include the key questions to ask.

**Solution**:
Start: "Does the task take a long time to complete?"
- Yes → "Does the user need feedback during execution?"
  - Yes → Action
  - No → "Does the task need to be cancelable?"
    - Yes → Action
    - No → Service
- No → Service

## Exercise 3.6: Topic Design for Sensor Fusion

**Difficulty**: Advanced

Design a topic-based communication system for sensor fusion in a humanoid robot with the following sensors:
- 2 cameras (stereo vision)
- 1 IMU
- Joint encoders for 28 joints
- 4 force/torque sensors (feet and hands)

Specify the topics, message types, and QoS considerations for each.

**Solution**:
- `/cameras/left/image_raw` - sensor_msgs/Image, Best effort, keep last
- `/cameras/right/image_raw` - sensor_msgs/Image, Best effort, keep last
- `/sensors/imu/data` - sensor_msgs/Imu, Reliable, keep last
- `/joint_states` - sensor_msgs/JointState, Reliable, keep last
- `/sensors/left_foot/force_torque` - geometry_msgs/WrenchStamped, Reliable, keep last
- `/sensors/right_foot/force_torque` - geometry_msgs/WrenchStamped, Reliable, keep last
- `/sensors/left_hand/force_torque` - geometry_msgs/WrenchStamped, Reliable, keep last
- `/sensors/right_hand/force_torque` - geometry_msgs/WrenchStamped, Reliable, keep last
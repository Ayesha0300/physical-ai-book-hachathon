# Chapter 6 Exercises: Robot Description with URDF

## Exercise 6.1: URDF Structure Understanding

**Difficulty**: Basic

Explain the relationship between links, joints, and coordinate frames in a URDF model. Provide an example of how they work together in a simple robot arm.

**Solution**:
Links represent rigid bodies in the robot structure. Joints define the connection between two links and specify how they can move relative to each other. Coordinate frames are attached to each link to define position and orientation. In a simple robot arm: the base is a link, the shoulder is a joint connecting base to upper arm link, the elbow is a joint connecting upper arm to lower arm link, and each has an associated coordinate frame for spatial relationships.

## Exercise 6.2: URDF Components Analysis

**Difficulty**: Basic

For each of the following URDF components, explain its purpose and why it's important:

A) Visual properties
B) Collision properties
C) Inertial properties

**Solution**:
A) Visual properties define how the robot appears in visualization tools like RViz; important for understanding robot appearance and debugging.
B) Collision properties define geometry used for collision detection; important for preventing self-collisions and environmental collisions.
C) Inertial properties define mass distribution; important for physics simulation and dynamics calculations in control systems.

## Exercise 6.3: Joint Types Application

**Difficulty**: Intermediate

For each joint in a humanoid robot, specify the appropriate joint type and explain why:

A) Shoulder joint
B) Elbow joint
C) Knee joint
D) Ankle joint
E) Neck joint
F) Waist joint

**Solution**:
A) Revolute (or 3 revolute joints) - Allows rotation in multiple directions for arm movement
B) Revolute - Allows flexion/extension of elbow
C) Revolute - Allows flexion/extension of knee
D) Revolute (2 DOF) - Allows dorsiflexion/plantarflexion and inversion/eversion
E) Revolute (2-3 DOF) - Allows head rotation and tilting
F) Revolute (3 DOF) - Allows torso movement in multiple directions

## Exercise 6.4: URDF Model Design

**Difficulty**: Intermediate

Design a simplified URDF structure for a mobile manipulator robot with:
- Mobile base (differential drive)
- 4-DOF arm with gripper
- RGB-D camera on top

Specify the links, joints, and their connections.

**Solution**:
Links: base_link, wheel_left, wheel_right, arm_base, arm_link1, arm_link2, arm_link3, arm_link4, gripper_left, gripper_right, camera_mount
Joints: base_to_wheel_left (continuous), base_to_wheel_right (continuous), base_to_arm_base (fixed), arm_base_to_link1 (revolute), link1_to_link2 (revolute), link2_to_link3 (revolute), link3_to_link4 (revolute), link4_to_gripper_left/right (prismatic), base_to_camera (fixed)

## Exercise 6.5: Coordinate Frame Convention

**Difficulty**: Advanced

Explain the right-hand rule for coordinate frames in URDF. Design a simple 2-link planar arm and specify the coordinate frame orientations for each link, ensuring proper transformation calculations.

**Solution**:
Right-hand rule: thumb = X-axis, index finger = Y-axis, middle finger = Z-axis. For a 2-link planar arm: base_link frame has Z up, X forward; first link frame rotates about Z-axis from base; second link frame also rotates about Z-axis from first link. All frames follow right-hand rule for consistent transformations.

## Exercise 6.6: Simulation Integration

**Difficulty**: Advanced

Explain how a URDF model integrates with Gazebo simulation. What aspects of the URDF are used for physics simulation, and what aspects are used for visualization?

**Solution**:
For physics simulation: inertial properties (mass, center of mass, inertia matrix) and collision geometry define physical behavior. For visualization: visual geometry and materials define appearance. Gazebo uses collision geometry for physics calculations and visual geometry for rendering. Joint limits and dynamics properties from URDF are also used for realistic simulation.

## Exercise 6.7: Humanoid Robot Modeling

**Difficulty**: Advanced

Design the URDF structure for a simple humanoid robot with 2 legs, 2 arms, and a head. Specify the minimal set of joints needed for basic locomotion and manipulation, and explain the kinematic chains.

**Solution**:
Kinematic chains: Torso (base) → legs (hip-knee-ankle) for locomotion, Torso → arms (shoulder-elbow-wrist) for manipulation, Torso → head for perception. Minimal joints: 6 DOF per arm (3 shoulder, 1 elbow, 2 wrist), 6 DOF per leg (3 hip, 1 knee, 2 ankle), 3 DOF for head (pan/tilt/roll). This provides basic mobility and manipulation capabilities.
---
sidebar_position: 11
title: "Concept: URDF Models"
---

# Concept: URDF Models

## Overview

URDF (Unified Robot Description Format) is a fundamental component of the ROS ecosystem that provides a standardized way to describe robot models. For humanoid robots, which have complex kinematic structures with dozens of joints and links, URDF serves as the blueprint that defines the robot's physical properties, kinematic relationships, and visual representation.

## Purpose of Robot Models

Robot models serve several critical functions in the robotics ecosystem:

### Visualization
- Show the robot in RViz or simulation environments
- Enable debugging and monitoring of robot state
- Provide intuitive understanding of robot structure

### Kinematics
- Calculate forward and inverse kinematics
- Determine positions of end-effectors
- Understand robot workspace and capabilities

### Physics Simulation
- Accurately model robot behavior in simulation
- Enable collision detection and prevention
- Support dynamics calculations for control

### Motion Planning
- Generate collision-free paths for robot movement
- Consider robot geometry in planning algorithms
- Ensure kinematic feasibility of motions

## Core Components of URDF

### Links

A **link** represents a rigid body part of the robot. Each link has:

#### Physical Properties
- **Mass**: Total mass of the link
- **Center of mass**: Location of the center of mass relative to the link frame
- **Inertia matrix**: 3x3 matrix describing how mass is distributed

#### Geometric Properties
- **Visual geometry**: How the link appears in visualization tools
- **Collision geometry**: Geometry used for collision detection
- **Coordinate frame**: A reference frame attached to the link

**Example link definition:**
```xml
<link name="link_name">
  <inertial>
    <!-- Mass and inertia properties -->
  </inertial>
  <visual>
    <!-- Visual appearance -->
  </visual>
  <collision>
    <!-- Collision geometry -->
  </collision>
</link>
```

### Joints

A **joint** defines the connection between two links and specifies how they can move relative to each other.

#### Joint Types
- **Fixed**: No motion allowed (welded connection)
- **Revolute**: Rotational motion around an axis (hinge)
- **Prismatic**: Linear motion along an axis (slider)
- **Continuous**: Continuous rotation (like a revolute without limits)
- **Floating**: 6 degrees of freedom
- **Planar**: Motion in a plane

#### Joint Properties
- **Parent and child**: The two links the joint connects
- **Axis**: The axis of motion (for revolute and prismatic joints)
- **Limits**: Range of motion and velocity/torque limits
- **Origin**: Position and orientation of the joint

**Example joint definition:**
```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="x y z" rpy="roll pitch yaw"/>
  <axis xyz="x y z"/>
  <limit lower="min" upper="max" effort="max_effort" velocity="max_velocity"/>
</joint>
```

### Coordinate Frames

Each link has an associated coordinate frame that defines its position and orientation in space. Coordinate frames follow the right-hand rule:

- **X-axis**: Forward (or in the direction of the joint axis for revolute joints)
- **Y-axis**: Left
- **Z-axis**: Up

These frames are essential for:
- **Transform calculations**: Computing positions of end-effectors
- **Sensor data interpretation**: Understanding where sensor readings come from
- **Motion planning**: Planning paths in 3D space

## URDF Properties in Detail

### Visual Properties

Visual properties define how the robot appears in visualization tools:

#### Geometry
- **Primitive shapes**: Box, cylinder, sphere
- **Meshes**: Complex 3D models (STL, DAE, OBJ files)
- **Dimensions**: Size and scale parameters

#### Material
- **Color**: RGB values for coloring the link
- **Texture**: Image files for detailed appearance
- **Transparency**: Alpha values for see-through materials

### Collision Properties

Collision properties define the geometry used for collision detection:

#### Design Considerations
- **Simpler than visual**: Often uses simpler shapes for efficiency
- **Conservative**: May be slightly larger than visual to ensure safety
- **Multiple shapes**: Complex links may have multiple collision elements

### Inertial Properties

Inertial properties define the mass distribution of each link:

#### Components
- **Mass**: Total mass of the link
- **Center of mass**: Location of the center of mass relative to the link frame
- **Inertia matrix**: 3x3 matrix describing how mass is distributed

#### Importance
- **Physics simulation**: Affects how the robot behaves in simulation
- **Control calculations**: Influences dynamics in control systems
- **Stability**: Critical for balance and motion planning

## Modeling Humanoid Robots

### Kinematic Structure

Humanoid robots typically have a kinematic structure that mimics human anatomy:

#### Body Segments
- **Torso**: The base of the robot, often containing the main computer and batteries
- **Head**: Usually has cameras and sensors for perception
- **Arms**: Each with shoulder, elbow, and wrist joints for manipulation
- **Legs**: Each with hip, knee, and ankle joints for locomotion
- **Hands**: Often with multiple fingers for fine manipulation

#### Joint Configuration
- **Shoulder joints**: Usually 3 degrees of freedom (flexion/extension, abduction/adduction, internal/external rotation)
- **Elbow joints**: Typically 1 degree of freedom (flexion/extension)
- **Wrist joints**: Often 2-3 degrees of freedom for orientation
- **Hip joints**: 3 degrees of freedom (similar to shoulders)
- **Knee joints**: 1 degree of freedom (flexion/extension)
- **Ankle joints**: 2 degrees of freedom (dorsiflexion/plantarflexion, inversion/eversion)

## Integration with Simulation and Control

### Physics Simulation

URDF serves as the input for physics simulation engines like Gazebo:

#### Physics Properties
- **Mass and inertia**: Determine physical behavior
- **Collision geometry**: Affects collision detection
- **Joint dynamics**: Joint limits and friction properties affect simulation

### ROS 2 Tool Integration

URDF integrates with various ROS 2 tools:

#### Core Tools
- **robot_state_publisher**: Publishes transforms for visualization
- **TF2**: Provides coordinate transformations between frames
- **MoveIt**: Uses URDF for motion planning and collision checking
- **RViz**: Displays the robot model in 3D

#### Control Systems
- **Forward kinematics**: Calculate end-effector positions from joint angles
- **Inverse kinematics**: Calculate joint angles to achieve desired positions
- **Dynamics**: Consider robot mass and inertia in control calculations

## Best Practices for URDF Development

### 1. Accurate Physical Properties
- Measure real robot dimensions and masses
- Use CAD models when available for accurate geometry
- Verify inertial properties through physical measurements

### 2. Proper Frame Conventions
- Follow consistent frame conventions throughout the model
- Use the right-hand rule consistently
- Align joint axes with actual robot joint axes

### 3. Efficient Geometry
- Use simpler collision geometry for better performance
- Use detailed visual geometry for appearance
- Consider using meshes for complex shapes

### 4. Validation and Testing
- Test the URDF model in simulation
- Verify kinematic solutions match expectations
- Check for self-collisions and singularities

### 5. Organization and Maintenance
- Organize URDF files logically (separate files for different parts)
- Use Xacro for parameterization and cleaner code
- Document the model structure and conventions

## Related Concepts

- [ROS 2 Architecture](./ros2-architecture.md)
- [Communication Primitives](./communication-primitives.md)
- [Robot Description](./chapter6-robot-description-with-urdf.md)
- [AI Agents](./ai-agents.md)
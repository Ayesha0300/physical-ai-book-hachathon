---
sidebar_position: 6
title: "Chapter 6: Robot Description with URDF"
---

# Chapter 6: Robot Description with URDF

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the purpose and structure of URDF (Unified Robot Description Format)
- Understand how URDF describes links, joints, and coordinate frames for humanoid robots
- Identify the key components of URDF: visual, collision, and inertial properties
- Recognize URDF as the foundation for simulation and control
- Apply URDF concepts to model humanoid robot structures
- Understand how URDF integrates with ROS 2 and simulation environments

## Introduction

The Unified Robot Description Format (URDF) is a fundamental component of the ROS ecosystem that provides a standardized way to describe robot models. For humanoid robots, which have complex kinematic structures with dozens of joints and links, URDF serves as the blueprint that defines the robot's physical properties, kinematic relationships, and visual representation.

Understanding URDF is essential for anyone working with humanoid robots, as it forms the foundation for simulation, control, visualization, and interaction with various ROS tools. Without an accurate URDF model, many core ROS functionalities like forward kinematics, inverse kinematics, and collision detection would not be possible.

## Why Robots Need Formal Descriptions

### The Need for Robot Models

Physical robots exist in the real world with specific physical properties and kinematic constraints. However, software systems need a digital representation of the robot to:

- **Visualize the robot**: Show the robot in RViz or simulation environments
- **Understand kinematics**: Calculate forward and inverse kinematics
- **Detect collisions**: Prevent self-collisions and environmental collisions
- **Simulate physics**: Accurately model robot behavior in simulation
- **Plan motions**: Generate collision-free paths for robot movement
- **Control dynamics**: Implement advanced control algorithms that consider robot dynamics

### Challenges in Robot Description

Robot description faces several challenges:

- **Complexity**: Humanoid robots can have 20+ joints and complex kinematic chains
- **Accuracy**: Small errors in model parameters can lead to significant control issues
- **Standardization**: Different tools and libraries need to understand the same model
- **Maintenance**: Robot models must be updated when hardware changes

## Links, Joints, and Coordinate Frames

### Links

In URDF, a **link** represents a rigid body part of the robot. Each link has:

- **Physical properties**: Mass, center of mass, and inertia matrix
- **Geometric properties**: Visual and collision shapes
- **Coordinate frame**: A reference frame attached to the link

**Example links in a humanoid robot:**
- Base link (torso)
- Head
- Upper arms
- Lower arms
- Hands
- Upper legs
- Lower legs
- Feet

**URDF link structure:**
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

A **joint** defines the connection between two links and specifies how they can move relative to each other. Joints have:

- **Type**: The kind of motion allowed (revolute, prismatic, fixed, etc.)
- **Axis**: The axis of motion (for revolute and prismatic joints)
- **Limits**: Range of motion and velocity/torque limits
- **Parent and child**: The two links the joint connects

**Joint types:**
- **Fixed**: No motion allowed (welded connection)
- **Revolute**: Rotational motion around an axis (hinge)
- **Prismatic**: Linear motion along an axis (slider)
- **Continuous**: Continuous rotation (like a revolute without limits)
- **Floating**: 6 degrees of freedom
- **Planar**: Motion in a plane

**URDF joint structure:**
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

## Modeling Humanoid Structure

### Humanoid Robot Kinematics

Humanoid robots typically have a kinematic structure that mimics human anatomy:

**Torso**: The base of the robot, often containing the main computer and batteries
**Head**: Usually has cameras and sensors for perception
**Arms**: Each with shoulder, elbow, and wrist joints for manipulation
**Legs**: Each with hip, knee, and ankle joints for locomotion
**Hands**: Often with multiple fingers for fine manipulation

### Typical Humanoid URDF Structure

A humanoid robot URDF typically has a tree structure starting from a base link:

```
base_link
├── head
├── left_arm (chain: shoulder → elbow → wrist)
├── right_arm (chain: shoulder → elbow → wrist)
├── left_leg (chain: hip → knee → ankle)
└── right_leg (chain: hip → knee → ankle)
```

### Joint Configuration for Humanoid Robots

Humanoid robots require careful joint configuration to achieve human-like motion:

**Shoulder joints**: Usually 3 degrees of freedom (flexion/extension, abduction/adduction, internal/external rotation)
**Elbow joints**: Typically 1 degree of freedom (flexion/extension)
**Wrist joints**: Often 2-3 degrees of freedom for orientation
**Hip joints**: 3 degrees of freedom (similar to shoulders)
**Knee joints**: 1 degree of freedom (flexion/extension)
**Ankle joints**: 2 degrees of freedom (dorsiflexion/plantarflexion, inversion/eversion)

## Visual, Collision, and Inertial Properties

### Visual Properties

Visual properties define how the robot appears in visualization tools:

- **Geometry**: Shape and dimensions (box, cylinder, sphere, or mesh)
- **Material**: Color, texture, and visual properties
- **Origin**: Position and orientation relative to the link frame

**Example visual definition:**
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://robot_description/meshes/link_name.dae"/>
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

### Collision Properties

Collision properties define the geometry used for collision detection:

- **Simpler than visual**: Often uses simpler shapes for efficiency
- **Conservative**: May be slightly larger than visual to ensure safety
- **Multiple shapes**: Complex links may have multiple collision elements

**Example collision definition:**
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <cylinder length="0.1" radius="0.05"/>
  </geometry>
</collision>
```

### Inertial Properties

Inertial properties define the mass distribution of each link:

- **Mass**: Total mass of the link
- **Center of mass**: Location of the center of mass relative to the link frame
- **Inertia matrix**: 3x3 matrix describing how mass is distributed

**Example inertial definition:**
```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```

## URDF as the Foundation for Simulation and Control

### Integration with Gazebo and Simulation

URDF serves as the input for physics simulation engines like Gazebo:

- **Physics properties**: Mass, inertia, and collision geometry determine physical behavior
- **Joint dynamics**: Joint limits and friction properties affect simulation
- **Sensor placement**: URDF defines where sensors are mounted on the robot

### Integration with ROS 2 Tools

URDF integrates with various ROS 2 tools:

- **robot_state_publisher**: Publishes transforms for visualization
- **TF2**: Provides coordinate transformations between frames
- **MoveIt**: Uses URDF for motion planning and collision checking
- **RViz**: Displays the robot model in 3D

### Control System Integration

URDF is essential for control systems:

- **Forward kinematics**: Calculate end-effector positions from joint angles
- **Inverse kinematics**: Calculate joint angles to achieve desired positions
- **Dynamics**: Consider robot mass and inertia in control calculations

### Example URDF for a Simple Robot

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.6"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.6"/>
      </geometry>
    </collision>
  </link>

  <!-- Head link connected by a fixed joint -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting base to head -->
  <joint name="neck_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
  </joint>
</robot>
```

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

## Key Takeaways

- URDF provides a standardized format for describing robot models
- Links represent rigid bodies, joints define connections between them
- Visual, collision, and inertial properties define different aspects of robot behavior
- URDF is fundamental for simulation, visualization, and control
- Humanoid robots require careful kinematic modeling due to their complexity
- Accurate URDF models are essential for reliable robot operation

## Next Steps

Continue to [Chapter 7: From Digital Brain to Physical Body](./chapter7-digital-brain-to-physical-body.md) to understand the complete signal flow from AI decision to robot action.

## Prerequisites for Next Chapter

Before proceeding to Chapter 7, ensure you understand:
- The purpose and structure of URDF
- How links and joints define robot kinematics
- The importance of visual, collision, and inertial properties
- How URDF integrates with simulation and control systems
- Best practices for creating accurate robot models

## References

ROS 2 Documentation. (2023). *URDF overview and tutorials*. ROS Documentation. https://docs.ros.org/en/rolling/Tutorials/URDF/Using-URDF-With-Robot-State-Publisher.html

Macenski, S. (2022). *Robot Operating System 2: Design, architecture, and uses in research*. Applied Sciences, 12(1), 381. https://doi.org/10.3390/app12010381

Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). *Introduction to autonomous mobile robots*. MIT press.
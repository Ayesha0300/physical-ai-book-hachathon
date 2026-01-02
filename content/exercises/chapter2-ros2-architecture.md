# Chapter 2 Exercises: ROS 2 Architecture Overview

## Exercise 2.1: Architecture Components Identification

**Difficulty**: Basic

List the five main architectural components of ROS 2 and briefly explain the purpose of each.

**Solution**:
1. RMW (ROS Middleware) Layer - Abstracts underlying DDS implementation
2. Client Libraries (rclcpp, rclpy) - Provide language-specific interfaces
3. Launch System - Manages starting multiple nodes together
4. Parameter System - Unified configuration management
5. Tools and Visualization - Monitoring and debugging capabilities

## Exercise 2.2: Node Communication Patterns

**Difficulty**: Intermediate

Explain the difference between direct function calls in traditional software and the communication patterns used in ROS 2 nodes. Why is this difference important for robotics?

**Solution**:
Traditional software uses direct function calls which create tight coupling between components. ROS 2 uses topics (publish-subscribe), services (request-response), and actions (goal-feedback-result) which create loose coupling. This is important for robotics because components may run on different machines, need to handle failures gracefully, and require real-time performance without blocking.

## Exercise 2.3: Quality of Service Scenarios

**Difficulty**: Intermediate

For each scenario, explain which QoS policy would be most appropriate and why:
A) Safety-critical emergency stop messages
B) Camera image streaming for navigation
C) Parameter updates that must persist
D) Joint position feedback for control

**Solution**:
A) Reliable, transient local - Must be delivered, need historical values
B) Best effort, volatile - Can tolerate some loss, only need current images
C) Reliable, transient local - Must be delivered, need to persist for new subscribers
D) Reliable, keep last - Must be delivered, only need most recent values

## Exercise 2.4: Namespace Design

**Difficulty**: Basic

Design a namespace structure for a humanoid robot with the following subsystems:
- Left arm with 7 joints
- Right arm with 7 joints
- Mobile base with differential drive
- Head with pan/tilt mechanism
- Stereo vision system

Explain your naming convention and why it's beneficial.

**Solution**:
- `/left_arm/joint_states`, `/left_arm/commands`
- `/right_arm/joint_states`, `/right_arm/commands`
- `/mobile_base/velocity`, `/mobile_base/odometry`
- `/head/pan_tilt`, `/head/state`
- `/vision/left/image_raw`, `/vision/right/image_raw`

This structure provides clear organization, prevents naming conflicts, and allows for modular development.

## Exercise 2.5: Lifecycle Management

**Difficulty**: Basic

Explain the four main states in the ROS 2 lifecycle node state machine and when a node transitions between them.

**Solution**:
1. Unconfigured - Initial state, no resources acquired
2. Inactive - Configuration complete, but not active
3. Active - Running and processing data
4. Finalized - Resources released, ready for destruction

Transitions occur during node initialization, activation, deactivation, and cleanup processes.
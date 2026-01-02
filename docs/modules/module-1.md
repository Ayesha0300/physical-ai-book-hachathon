---
sidebar_position: 6
title: Module 1 - ROS 2
---

# Module 1: ROS 2 – Robotic Nervous System

import Breadcrumb from '@site/src/components/Breadcrumb';

<Breadcrumb items={[
  { label: 'Home', href: '/' },
  { label: 'Modules', href: '/modules' },
  { label: 'Module 1: ROS 2' }
]} />

## Learning Objectives
- Understand the fundamental concepts of Robot Operating System 2 (ROS 2)
- Learn about distributed robotics architecture and communication patterns
- Master ROS 2 nodes, topics, services, and actions
- Implement basic communication between robotic components
- Bridge AI agents with ROS 2 for integrated systems

## Concepts Covered
- **ROS 2 Architecture**: DDS-based communication, nodes, and distributed systems
- **Communication Primitives**: Topics for pub/sub, services for request/response, actions for goal-oriented tasks
- **Middleware Integration**: Connecting AI agents with physical robots using rclpy
- **Robot Description**: Using URDF for robot modeling and description
- **System Integration**: Connecting digital brain to physical body

## Tools & Frameworks
- **ROS 2 (Humble Hawksbill)**: Latest LTS version for robotics applications
- **rclpy**: Python client library for ROS 2
- **rclcpp**: C++ client library for ROS 2
- **RViz**: 3D visualization tool for robotics
- **Gazebo**: Physics simulation environment
- **URDF**: Unified Robot Description Format

## Mini-Projects
- **Basic Publisher/Subscriber**: Create a simple communication system between nodes
- **Robot State Publisher**: Implement a system to broadcast robot joint states
- **AI-ROS Bridge**: Connect a simple AI agent with ROS 2 nodes
- **Sensor Integration**: Publish sensor data and subscribe to it in another node
- **Action Server**: Implement a goal-oriented task with feedback

## Readiness for Capstone
This module provides the foundational communication infrastructure for the capstone project. You'll understand how to:
- Connect different components of your humanoid robot system
- Enable communication between perception, planning, and control systems
- Integrate AI components with physical robot systems
- Implement distributed control architectures necessary for humanoid robots
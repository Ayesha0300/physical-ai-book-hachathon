---
sidebar_position: 9
title: "Concept: Communication Primitives"
---

# Concept: Communication Primitives

## Overview

Communication primitives are the fundamental patterns for exchanging information in ROS 2. There are three primary communication patterns that enable different types of interactions between nodes: topics for continuous data streams, services for request-response interactions, and actions for long-running goals with feedback.

## Topics: Continuous Data Streams

Topics provide a publish-subscribe communication pattern where data flows continuously from publishers to subscribers. This pattern is ideal for:

- Sensor data (camera images, LIDAR scans, IMU readings)
- Robot state (joint positions, battery levels, system status)
- Continuous control signals (velocity commands, servo positions)
- System monitoring data (CPU usage, memory consumption)

### Characteristics
- **Asynchronous**: Publishers and subscribers don't need to be synchronized
- **Many-to-many**: Multiple publishers can publish to a topic, multiple subscribers can subscribe
- **Fire-and-forget**: Publishers don't know if anyone is listening
- **Real-time friendly**: Low latency, high throughput
- **Data-driven**: Communication is triggered by data availability

## Services: Request-Response Interactions

Services provide a synchronous request-response communication pattern similar to remote procedure calls (RPC). This pattern is ideal for:

- Configuration changes (setting parameters, calibration)
- One-time queries (current position, system status)
- Command execution (take picture, move to position)
- Error recovery (reset system, clear fault)

### Characteristics
- **Synchronous**: The client waits for a response from the server
- **One-to-one**: One client requests from one server at a time
- **Request-response**: Client sends request, server sends response
- **Blocking**: Client is blocked until response is received
- **Stateless**: Each request is independent

## Actions: Long-Running Goals with Feedback

Actions provide a goal-oriented communication pattern for long-running operations that require feedback and the ability to cancel. This pattern is ideal for:

- Motion planning and execution (walking, manipulation)
- Complex tasks with multiple stages (assembly, inspection)
- Operations that can be monitored and canceled (data collection, cleaning)
- Tasks with intermediate feedback (progress, status)

### Characteristics
- **Goal-oriented**: Client sends a goal to achieve
- **Feedback**: Server provides continuous feedback during execution
- **Result**: Server sends final result when complete
- **Cancelability**: Client can cancel the goal at any time
- **Non-blocking**: Client doesn't block while goal is executing

## Quality of Service (QoS)

All communication primitives in ROS 2 support Quality of Service policies that affect performance:

- **Reliability**: Reliable (guaranteed delivery) vs. Best effort (no guarantee)
- **Durability**: Volatile (only new messages) vs. Transient local (all messages)
- **History**: Keep all messages vs. Keep last N messages
- **Depth**: Size of the message queue

## When to Use Each Pattern

### Use Topics When:
- Data is continuously generated (sensors, state)
- Multiple nodes need the same information
- Real-time performance is critical
- It's okay if some messages are missed
- The communication is one-way (no response needed)

### Use Services When:
- You need guaranteed delivery of request and response
- The operation is quick and deterministic
- You need a clear success/failure result
- The operation is stateless
- You can afford to block the client

### Use Actions When:
- The operation takes a long time to complete
- You need to monitor progress during execution
- The operation might need to be canceled
- You need intermediate feedback
- The operation has multiple stages or steps

## Related Concepts

- [ROS 2 Architecture](./ros2-architecture.md)
- [Nodes in Practice](./chapter4-nodes-in-practice.md)
- [Python AI Agents](./chapter5-bridging-python-ai-agents.md)
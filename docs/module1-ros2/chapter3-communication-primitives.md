---
sidebar_position: 3
title: "Chapter 3: Communication Primitives in ROS 2"
---

# Chapter 3: Communication Primitives in ROS 2

## Learning Objectives

By the end of this chapter, you will be able to:
- Distinguish between the three main ROS 2 communication patterns: topics, services, and actions
- Explain when to use each communication pattern for different robotic scenarios
- Understand the characteristics and use cases for continuous data streams, request-response interactions, and long-running goals
- Map communication patterns to specific humanoid robot behaviors
- Identify the appropriate communication primitive for a given robotic task

## Introduction

Communication is the lifeblood of any robotic system. In ROS 2, three primary communication primitives enable different types of interactions between nodes: topics for continuous data streams, services for request-response interactions, and actions for long-running goals with feedback. Understanding when and how to use each primitive is essential for designing effective robotic systems.

## Topics: Continuous Data Streams

### Overview

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

### Example: Sensor Data Streaming

Consider a camera node that continuously publishes images:

```
Camera Node (Publisher) → /camera/image_raw → Image Processing Node (Subscriber)
```

The camera publishes images at a regular rate (e.g., 30 Hz), and any number of other nodes can subscribe to process these images for object detection, navigation, or other purposes.

### Quality of Service (QoS) Considerations

Topics support various QoS policies that affect performance:

- **Reliability**: Reliable (guaranteed delivery) vs. Best effort (no guarantee)
- **Durability**: Volatile (only new messages) vs. Transient local (all messages)
- **History**: Keep all messages vs. Keep last N messages
- **Depth**: Size of the message queue

For sensor data, you typically want "best effort" reliability with "keep last" history to avoid overwhelming the system with old data.

## Services: Request-Response Interactions

### Overview

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

### Example: Navigation Service

A navigation service might receive a goal position and return whether navigation was successful:

```
Navigation Client → Request: "Go to (x=5, y=3)" → Navigation Server
Navigation Server → Response: "Success" or "Failed: Obstacle detected" → Navigation Client
```

### When to Use Services

Use services when you need:
- Guaranteed delivery of the request and response
- Synchronous behavior where the client must wait for completion
- Simple, quick operations that don't take long to complete
- Clear success/failure responses

## Actions: Long-Running Goals with Feedback

### Overview

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

### Example: Robot Arm Action

A robot arm action for grasping an object:

```
Arm Client → Goal: "Grasp object at (x, y, z)" → Arm Server
Arm Server → Feedback: "Moving to approach position" → Arm Client
Arm Server → Feedback: "Approaching object" → Arm Client
Arm Server → Feedback: "Gripper closing" → Arm Client
Arm Server → Result: "Grasp successful" → Arm Client
```

### Action States

Actions have a well-defined state machine:
- **PENDING**: Goal received, not yet processing
- **ACTIVE**: Currently processing the goal
- **PREEMPTED**: Goal canceled by another goal
- **SUCCEEDED**: Goal completed successfully
- **ABORTED**: Goal failed to complete
- **RECALLED**: Goal canceled by client

## Comparison of Communication Patterns

| Aspect | Topics | Services | Actions |
|--------|--------|----------|---------|
| **Pattern** | Publish-Subscribe | Request-Response | Goal-Feedback-Result |
| **Synchronization** | Asynchronous | Synchronous | Asynchronous |
| **Duration** | Continuous | Immediate | Long-running |
| **Feedback** | None (data only) | Yes (one response) | Continuous |
| **Cancelability** | No | No | Yes |
| **Use Case** | Streaming data | Simple queries | Complex operations |

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

## Humanoid Robot Behavior Examples

### Walking Behavior
- **Communication Pattern**: Action
- **Reason**: Walking is a long-running operation that requires feedback (step progress, balance status) and can be canceled
- **Implementation**: Walk action server provides feedback on each step and can be interrupted for safety

### Head Tracking
- **Communication Pattern**: Topic
- **Reason**: Continuously streaming target position to head controller
- **Implementation**: Vision system publishes target coordinates, head controller subscribes and follows

### Emergency Stop
- **Communication Pattern**: Service
- **Reason**: Immediate, guaranteed execution of safety command
- **Implementation**: Emergency stop service that all safety-critical nodes provide

### Joint Calibration
- **Communication Pattern**: Service
- **Reason**: One-time calibration operation with success/failure result
- **Implementation**: Calibration service for each joint group

### Speech Recognition
- **Communication Pattern**: Topic
- **Reason**: Continuous streaming of recognized text from speech system
- **Implementation**: Speech recognition node publishes recognized text, other nodes subscribe

### Path Planning
- **Communication Pattern**: Action
- **Reason**: Planning can take time, user might want to cancel, progress feedback is useful
- **Implementation**: Path planning action with feedback on planning progress

## Best Practices

### Topic Best Practices
- Use appropriate QoS settings for your use case
- Don't overload the network with too many high-frequency topics
- Use latching for static data that new subscribers need
- Consider message size and frequency for performance

### Service Best Practices
- Keep service calls fast (under 100ms if possible)
- Don't use services for operations that might take a long time
- Design services to be idempotent when possible
- Handle service call failures gracefully

### Action Best Practices
- Provide meaningful feedback during long operations
- Implement proper cancellation handling
- Use appropriate goal timeouts
- Design goals to be interruptible when safety requires

## Key Takeaways

- Topics are for continuous data streams and sensor information
- Services are for quick, synchronous request-response interactions
- Actions are for long-running operations with feedback and cancellation
- Choose the right communication pattern for your specific use case
- Consider performance, reliability, and user experience when selecting patterns
- Humanoid robot behaviors map naturally to different communication patterns

## Next Steps

Continue to [Chapter 4: ROS 2 Nodes in Practice](./chapter4-nodes-in-practice.md) to learn how to implement effective ROS 2 nodes that follow best practices for robot systems.

## Prerequisites for Next Chapter

Before proceeding to Chapter 4, ensure you understand:
- The differences between topics, services, and actions
- When to use each communication pattern
- The characteristics and trade-offs of each pattern
- How these patterns apply to humanoid robot behaviors

## References

Macenski, S. (2022). *Robot Operating System 2: Design, architecture, and uses in research*. Applied Sciences, 12(1), 381. https://doi.org/10.3390/app12010381

ROS 2 Documentation. (2023). *ROS 2 communication patterns*. ROS Documentation. https://docs.ros.org/en/rolling/Concepts/About-Topics-Services-Actions.html

Quigley, M., Gerkey, B., & Smart, W. D. (2009). Programming robots with ROS: A practical introduction to the Robot Operating System. *IEEE Intelligent Systems*, 24(2), 68-75.
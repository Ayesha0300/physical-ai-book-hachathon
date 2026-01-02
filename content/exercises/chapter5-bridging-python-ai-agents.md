# Chapter 5 Exercises: Bridging Python AI Agents with ROS 2 (rclpy)

## Exercise 5.1: rclpy Node Implementation

**Difficulty**: Intermediate

Write a Python ROS 2 node that implements a simple AI agent. The agent should:
- Subscribe to a sensor topic (e.g., /sensor_data)
- Process the data using a simple AI algorithm (e.g., threshold detection)
- Publish a decision to an action topic (e.g., /ai_decision)

Include proper node initialization, subscription, and publishing.

**Solution**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String

class SimpleAIAgent(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')

        # Create subscription to sensor data
        self.subscription = self.create_subscription(
            Float32,
            '/sensor_data',
            self.sensor_callback,
            10
        )

        # Create publisher for AI decisions
        self.publisher = self.create_publisher(
            String,
            '/ai_decision',
            10
        )

        self.get_logger().info('Simple AI Agent node initialized')

    def sensor_callback(self, msg):
        # Simple AI decision: if sensor value > threshold, trigger action
        if msg.data > 5.0:
            decision_msg = String()
            decision_msg.data = 'HIGH_VALUE_DETECTED'
            self.publisher.publish(decision_msg)
            self.get_logger().info(f'Published decision: {decision_msg.data}')
        else:
            decision_msg = String()
            decision_msg.data = 'NORMAL_VALUE'
            self.publisher.publish(decision_msg)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = SimpleAIAgent()
    rclpy.spin(ai_agent)
    ai_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 5.2: Perception-Decision-Action Flow

**Difficulty**: Intermediate

Design the ROS 2 topics and nodes for a complete perception → decision → action flow for a robot that needs to avoid obstacles. Specify the message types, node responsibilities, and the flow of information.

**Solution**:
**Perception Phase:**
- LiDAR Node: Publishes sensor_msgs/LaserScan to /scan
- Image Node: Publishes sensor_msgs/Image to /camera/image_raw

**Decision Phase:**
- Obstacle Detection Node:
  - Subscribes to /scan and /camera/image_raw
  - Processes data to identify obstacles
  - Publishes geometry_msgs/Twist to /collision_avoidance/cmd_vel

**Action Phase:**
- Navigation Node:
  - Subscribes to /collision_avoidance/cmd_vel
  - Combines with navigation goals
  - Publishes to /cmd_vel for robot base controller

The flow allows for real-time obstacle detection and avoidance while maintaining navigation goals.

## Exercise 5.3: AI Agent Node Classification

**Difficulty**: Basic

Classify each of the following AI agents as primarily perception-focused, decision-focused, or action-focused, and explain your reasoning:

A) Object recognition node
B) Path planning node
C) Behavior tree executor
D) Natural language understanding node
E) State estimator
F) Task scheduler

**Solution**:
A) Perception-focused - Processes sensor data to recognize objects
B) Decision-focused - Makes planning decisions based on environment and goals
C) Action-focused - Executes planned behaviors and controls robot actions
D) Perception-focused - Processes natural language input to extract meaning
E) Perception-focused - Estimates system state from sensor data
F) Decision-focused - Makes scheduling decisions based on priorities and constraints

## Exercise 5.4: LLM Integration Design

**Difficulty**: Advanced

Design an architecture for integrating a Large Language Model (LLM) with a ROS 2 robot system. Specify how natural language commands would be processed and converted to robot actions.

**Solution**:
**LLM Interface Node:**
- Service: /llm_process_command (NaturalLanguageCommand service)
- Input: Natural language command string
- Output: Structured task plan

**Command Processing Flow:**
1. User sends: "Robot, please go to the kitchen and bring me a cup"
2. LLM Service parses into: [NAVIGATE, DETECT_OBJECT, GRASP_OBJECT, NAVIGATE, DELIVER]
3. Task Manager Node converts to ROS 2 actions:
   - NavigateToPose action for kitchen location
   - ObjectDetection service for cup identification
   - GraspObject action for cup pickup
   - NavigateToPose action for return
   - ReleaseObject action for delivery

**Safety Integration:**
- All LLM-generated plans validated against safety constraints
- Human confirmation for certain actions
- Emergency stop capability maintained

## Exercise 5.5: Python AI Performance Considerations

**Difficulty**: Intermediate

Identify three performance considerations when implementing Python AI agents in ROS 2 and suggest solutions for each.

**Solution**:
1. **Computation Time**: Python AI algorithms may be slower than C++
   - Solution: Use optimized libraries (NumPy, Cython), offload to GPU, or implement critical parts in C++

2. **Message Processing Rate**: High-frequency sensor data may overwhelm Python processing
   - Solution: Use message filters to reduce frequency, implement threading, or use message queues

3. **Memory Usage**: AI models can consume significant memory
   - Solution: Model optimization, memory management, or separate nodes for different AI tasks

## Exercise 5.6: Error Handling in AI-Robot Integration

**Difficulty**: Advanced

Design an error handling strategy for an AI agent that controls robot navigation. Consider what happens when:
- AI model fails to process sensor data
- Robot cannot execute the planned action
- Communication between AI agent and robot controller fails

**Solution**:
**AI Model Failure:**
- Implement timeout for AI processing
- Provide fallback behaviors (stop, use simple rules)
- Log error and attempt recovery

**Action Execution Failure:**
- Monitor action execution status
- Implement retry mechanisms
- Fall back to alternative plans
- Alert human operator if needed

**Communication Failure:**
- Implement heartbeat monitoring
- Use ROS 2's built-in communication reliability features
- Implement safe state when communication lost
- Attempt reconnection with timeout

All strategies maintain robot safety as the top priority.
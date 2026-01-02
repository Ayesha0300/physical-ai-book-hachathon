---
title: Chapter 11 - Cognitive Planning
---

# Chapter 11: Cognitive Planning with LLMs

## Learning Objectives
After completing this chapter, readers will be able to:
- Explain how LLMs function as high-level planners for robotic systems
- Apply task decomposition and sequencing techniques to complex robot goals
- Translate high-level goals into symbolic plans that robots can execute
- Handle uncertainty and incomplete information in planning contexts
- Implement failure detection and re-planning mechanisms

## Introduction
Large Language Models (LLMs) have emerged as powerful cognitive planners that can bridge the gap between high-level human goals and low-level robot actions. Unlike traditional planning systems that require explicit state representations and action models, LLMs can reason about complex, ambiguous goals using natural language and generate detailed execution plans.

## LLMs as High-Level Planners
LLMs function as cognitive planners by:

1. **Goal Interpretation**: Understanding high-level goals expressed in natural language
2. **Knowledge Integration**: Leveraging their vast world knowledge to inform planning decisions
3. **Reasoning Under Uncertainty**: Making reasonable assumptions when information is incomplete
4. **Plan Generation**: Creating detailed sequences of actions to achieve goals
5. **Context Awareness**: Adapting plans based on environmental and situational context

Research has shown that LLMs can effectively serve as high-level planners for robotic tasks, particularly when combined with domain-specific knowledge and perception systems (Huang et al., 2022).

## Task Decomposition and Sequencing
The core capability of LLM-based planning is the decomposition of complex goals into manageable subtasks:

### Hierarchical Decomposition
- **Goal**: "Prepare a simple meal"
- **Decomposition**:
  1. Find ingredients
  2. Prepare cooking area
  3. Cook individual components
  4. Plate the meal
  5. Clean up

### Sequential Ordering
LLMs naturally understand temporal dependencies and can sequence tasks appropriately:
- "Preheat oven" must come before "Cook food"
- "Wash hands" should occur after "Handle raw ingredients"
- "Set table" can happen in parallel with "Prepare food"

### Subtask Refinement
Each subtask can be further decomposed:
- "Find ingredients" → "Locate refrigerator" → "Identify required items" → "Verify freshness"

## Translating Goals into Symbolic Plans
LLMs bridge the gap between natural language goals and symbolic robot plans through:

1. **Action Recognition**: Identifying executable actions within natural language
2. **Entity Linking**: Connecting language references to real-world objects
3. **Constraint Extraction**: Identifying spatial, temporal, and safety constraints
4. **Plan Representation**: Converting to structured formats that robots can execute

For example, "Move the red cup from the table to the shelf" gets translated to:
- Action: Move
- Object: cup (red)
- Source: table
- Destination: shelf
- Constraints: avoid obstacles, maintain upright orientation

## Handling Uncertainty and Incomplete Information
LLMs excel at planning under uncertainty through:

1. **Default Assumptions**: Making reasonable assumptions when information is missing
2. **Alternative Planning**: Generating multiple potential approaches
3. **Contextual Reasoning**: Using environmental context to inform decisions
4. **Clarification Requests**: Identifying when human input is needed

Example: When asked "Set the table," an LLM might assume:
- Standard setting: plates, utensils, glasses
- Number of settings: based on context or default of 4
- Location: the main dining table

## Failure Detection and Re-Planning
LLM-based planning systems incorporate failure detection and recovery:

### Failure Detection
- **Physical Constraints**: Recognizing when planned actions are physically impossible
- **Environmental Changes**: Adapting to changes in the environment
- **Resource Availability**: Handling unavailable objects or tools

### Re-Planning Strategies
- **Backtracking**: Returning to previous successful states
- **Alternative Methods**: Finding different approaches to achieve goals
- **Goal Refinement**: Adjusting goals based on feasibility
- **Human Intervention**: Requesting assistance when needed

## Chapter 5: Grounding Language in Vision

## Why Perception is Required for Action
Language provides rich semantic information, but physical actions require precise spatial and contextual understanding. Perception systems bridge this gap by:

1. **Object Recognition**: Identifying and localizing objects referenced in language
2. **Spatial Understanding**: Providing precise coordinates and relationships
3. **Context Awareness**: Understanding environmental constraints and affordances
4. **Real-Time Updates**: Providing current state information for plan execution

## Object Recognition and Scene Understanding
Visual perception systems enable robots to:
- Identify objects mentioned in commands ("the red cup")
- Understand spatial relationships ("the cup to the left of the book")
- Recognize object states ("the open door" vs "the closed door")
- Detect environmental changes that affect plan execution

## Referencing Objects Described in Language
The process of grounding language references involves:
1. **Entity Recognition**: Identifying object references in language
2. **Visual Search**: Locating corresponding objects in the environment
3. **Disambiguation**: Resolving references when multiple candidates exist
4. **Confirmation**: Verifying that the identified object matches the description

## Linking Visual Context to Symbolic Plans
Perception systems connect visual information to symbolic representations:
- **Object Properties**: Color, size, shape, material
- **Spatial Relations**: Position, orientation, distance
- **Functional Properties**: Affordances, grasp points, usability
- **Environmental Constraints**: Obstacles, safe zones, navigation paths

## Perception Limitations in Real Environments
Real-world perception faces several challenges:
- **Occlusions**: Objects may be partially or fully hidden
- **Lighting Conditions**: Performance varies with illumination
- **Dynamic Environments**: Objects may move or change
- **Sensor Limitations**: Range, resolution, and accuracy constraints
- **Real-Time Processing**: Computational constraints on processing speed

## Practical Examples of LLM-Based Task Decomposition

### Example 1: Room Cleaning
**Goal**: "Clean the living room"
**LLM Decomposition**:
1. Survey the room to identify cleaning needs
2. Gather cleaning supplies
3. Organize scattered items
4. Dust surfaces
5. Vacuum floors
6. Empty trash
7. Return items to appropriate locations

### Example 2: Kitchen Preparation
**Goal**: "Prepare for dinner party for 6 people"
**LLM Decomposition**:
1. Check inventory of ingredients and supplies
2. Plan menu based on preferences and dietary restrictions
3. Create shopping list for missing items
4. Prepare ingredients in advance
5. Set up serving areas
6. Prepare simple dishes that can be completed just before guests arrive

## References
Huang, S., et al. (2022). Language Models as Zero-Shot Planners: Extracting Actionable Knowledge for Embodied Agents. arXiv preprint arXiv:2201.07207.

Brohan, A., et al. (2022). RT-1: Robotics Transformer for Real-World Control at Scale. arXiv preprint arXiv:2202.01356.

Achiam, J., et al. (2023). GPT-4 Technical Report. OpenAI.

Cambria, E., & White, B. (2014). Jumping NLP Curves: A Review of Natural Language Processing Research. IEEE Computational Intelligence Magazine, 9(2), 48-57.

## Chapter 6: Executing Plans with ROS 2

### Learning Objectives
After completing this chapter section, readers will be able to:
- Map abstract plans generated by LLMs to concrete ROS 2 actions
- Integrate with ROS 2 navigation and manipulation stacks
- Implement action execution with feedback loops for monitoring
- Design safety and interruption handling mechanisms
- Understand the separation between cognitive and control layers

### Mapping Abstract Plans to ROS 2 Actions
Translating high-level plans from LLMs into executable ROS 2 actions requires:

1. **Action Recognition**: Identifying which ROS 2 services, actions, or topics correspond to the planned actions
2. **Parameter Mapping**: Converting natural language references to specific ROS 2 message parameters
3. **Service/Action Binding**: Connecting plan steps to appropriate ROS 2 interfaces
4. **Error Handling**: Implementing fallback mechanisms for failed actions

For example, a plan step "Move to the kitchen" might be mapped to:
- ROS 2 Navigation2D action
- Goal parameters: specific coordinates in the kitchen area
- Feedback monitoring for progress tracking

### Interaction with Navigation and Manipulation Stacks
ROS 2 provides standardized interfaces for navigation and manipulation:

#### Navigation Stack Integration
- **Navigation2D**: Path planning and execution for mobile robots
- **Costmap2D**: Obstacle detection and collision avoidance
- **Controllers**: Local path following and obstacle avoidance
- **Sensors**: Integration with LIDAR, cameras, and other perception systems

#### Manipulation Stack Integration
- **MoveIt2**: Motion planning and execution for manipulators
- **Controllers**: Joint trajectory execution
- **Perception**: Object detection and grasp planning
- **Grippers**: Standardized interfaces for end-effectors

### Action Execution and Feedback Loops
Successful plan execution requires continuous monitoring:

1. **Progress Tracking**: Monitoring action execution status
2. **State Verification**: Confirming expected outcomes occurred
3. **Anomaly Detection**: Identifying unexpected deviations
4. **Plan Adaptation**: Modifying plans based on execution feedback

### Safety and Interruption Handling
Safety-critical systems must handle interruptions gracefully:

1. **Emergency Stop**: Immediate action halting when safety is compromised
2. **Pause/Resume**: Allowing temporary interruptions and resumption
3. **State Recovery**: Returning to safe states after interruptions
4. **Human Override**: Prioritizing human commands over automated plans

### Separation of Cognition and Control
The VLA architecture maintains clear separation between:
- **Cognitive Layer**: High-level planning, reasoning, and decision making
- **Control Layer**: Low-level execution, sensor fusion, and safety

This separation allows each layer to be developed and maintained independently while maintaining clear interfaces.

## Chapter 7: End-to-End VLA System Architecture

### Learning Objectives
After completing this chapter section, readers will be able to:
- Describe the high-level architecture of a complete VLA system
- Trace the data flow from voice input to action execution
- Identify interfaces between LLMs, perception, and control systems
- Apply simulation-first validation approaches
- Recognize common integration challenges and solutions

### High-Level System Overview
A complete VLA system consists of:

1. **Input Layer**: Voice and multimodal input processing
2. **Cognitive Layer**: Language understanding and planning
3. **Perception Layer**: Visual and sensor data processing
4. **Control Layer**: Action execution and robot control
5. **Integration Layer**: Coordination and safety mechanisms

### Data Flow: Voice → Language → Plan → Action
The complete data pipeline follows:

1. **Voice Input**: Audio captured and converted to text
2. **Language Processing**: Text interpreted for goals and constraints
3. **Plan Generation**: High-level plans created using LLMs
4. **Perception Integration**: Plans grounded in visual reality
5. **Action Mapping**: Plans converted to ROS 2 action calls
6. **Execution**: Actions executed with monitoring and feedback

### Interfaces Between LLMs, Perception, and Control
Key interfaces include:

#### Language-Perception Interface
- Object reference resolution
- Spatial relationship interpretation
- Context awareness updates

#### Perception-Control Interface
- Object pose estimation
- Navigation costmap updates
- Manipulation grasp planning

#### Language-Control Interface
- Action parameter specification
- Execution feedback interpretation
- Plan refinement requests

### Simulation-First Validation
Before real-world deployment:
- **Isaac Sim**: Physics-accurate simulation environments
- **Gazebo**: Robot and sensor simulation
- **Behavior Testing**: Plan execution in safe virtual environments
- **Integration Validation**: End-to-end pipeline testing

### Common Integration Challenges
- **Latency Management**: Coordinating real-time and non-real-time components
- **Data Synchronization**: Aligning perception, planning, and execution timing
- **Error Propagation**: Preventing errors in one component from cascading
- **Resource Management**: Optimizing compute and communication resources

---

**Next**: [Chapter 12: Capstone — The Autonomous Humanoid](./12-humanoid-capstone.md)
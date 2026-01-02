# Unity Scene Examples for Robotics Simulation

## Example 1: Basic Unity-Gazebo Integration Scene

### Scene: `robot_integration_scene.unity`
- **Robot Model**: Imported URDF/SDF robot model with proper joint configuration
- **Environment**: Simple room with obstacles for navigation testing
- **Cameras**: Multiple camera angles for different perception tasks
- **Lighting**: Realistic lighting setup with shadows
- **Physics**: Appropriate physics materials for realistic interactions
- **ROS Connection**: Unity Robotics Hub components for ROS communication

### Key Components:
- `RobotController.cs`: Script for handling ROS message communication
- `SensorManager.cs`: Manages Unity sensors to mimic real robot sensors
- `EnvironmentManager.cs`: Controls environmental parameters for testing

## Example 2: Perception Training Scene

### Scene: `perception_training_scene.unity`
- **Variety of Objects**: Diverse objects with different shapes, textures, and materials
- **Lighting Variations**: Multiple lighting conditions (day, night, indoor, outdoor)
- **Camera Setup**: Multiple camera angles and positions for comprehensive training
- **Annotation System**: Automatic bounding box and segmentation mask generation
- **Domain Randomization**: Randomized textures, colors, and lighting

### Key Components:
- `SyntheticDataGenerator.cs`: Generates training datasets with ground truth
- `ObjectRandomizer.cs`: Randomizes object properties for domain randomization
- `CameraController.cs`: Manages multiple camera viewpoints
- `AnnotationSystem.cs`: Creates automatic annotations for training data

## Example 3: Human-Robot Interaction Scene

### Scene: `hri_scene.unity`
- **Human Avatars**: Animated human models with realistic movement
- **Interaction Space**: Environment designed for human-robot collaboration
- **Gesture Recognition**: Systems for recognizing human gestures
- **Social Cues**: Visual elements for appropriate social interaction
- **Safety Boundaries**: Visual indicators of safe/unsafe interaction zones

### Key Components:
- `HumanController.cs`: Manages human avatar behavior and animation
- `InteractionManager.cs`: Handles human-robot interaction protocols
- `SafetySystem.cs`: Ensures safe interaction distances and behaviors
- `GestureRecognition.cs`: Simulates gesture recognition capabilities

## Example 4: Physics vs Perception Tradeoff Scene

### Scene: `physics_perception_tradeoff.unity`
- **Physics-Accurate Area**: Region with detailed physics simulation
- **Perception-Optimized Area**: Region with enhanced visual quality
- **Comparison Elements**: Same objects rendered with different priorities
- **Performance Metrics**: Tools for measuring physics vs rendering performance
- **Hybrid Simulation**: Example of external physics engine integration

### Key Components:
- `PhysicsSwitcher.cs`: Toggles between physics accuracy and visual quality
- `PerformanceMonitor.cs`: Tracks physics and rendering performance
- `ExternalPhysicsBridge.cs`: Connects to external physics engines
- `QualityController.cs`: Adjusts quality settings based on simulation needs

## Example 5: Multi-Sensor Simulation Scene

### Scene: `multi_sensor_scene.unity`
- **RGB Camera**: Standard color camera simulation
- **Depth Camera**: Depth sensor simulation with noise models
- **LiDAR Simulation**: Raycasting-based LiDAR with realistic returns
- **Thermal Camera**: Thermal imaging simulation
- **IMU Simulation**: Inertial measurement unit simulation

### Key Components:
- `MultiSensorManager.cs`: Coordinates multiple sensor outputs
- `SensorCalibration.cs`: Simulates sensor calibration procedures
- `NoiseGenerator.cs`: Adds appropriate noise models to sensors
- `SensorFusion.cs`: Combines data from multiple sensors
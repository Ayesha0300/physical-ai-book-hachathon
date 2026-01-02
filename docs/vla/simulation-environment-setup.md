# Simulation Environment Setup: Gazebo and Isaac Sim for VLA Systems

## Overview
This document provides detailed setup instructions and requirements for simulation environments used in Vision-Language-Action (VLA) systems, focusing on Gazebo and Isaac Sim platforms.

## Gazebo Simulation Setup

### System Requirements

#### Minimum Requirements
- **OS**: Ubuntu 22.04 LTS or Windows 10/11 (with WSL2)
- **CPU**: Intel i5 or equivalent with 4+ cores
- **RAM**: 8GB minimum, 16GB recommended
- **GPU**: Graphics card with OpenGL 3.3+ support
- **Storage**: 10GB free space for basic Gazebo installation
- **Network**: Internet connection for model downloads

#### Recommended Requirements
- **OS**: Ubuntu 22.04 LTS (native installation preferred)
- **CPU**: Intel i7 or equivalent with 8+ cores
- **RAM**: 32GB for complex scenarios
- **GPU**: NVIDIA RTX series with CUDA support for accelerated rendering
- **Storage**: 50GB+ for multiple robot models and environments
- **Network**: Stable connection for real-time collaboration

### Installation Steps

#### 1. Install ROS 2 Humble with Gazebo
```bash
# Update system packages
sudo apt update

# Install ROS 2 Humble desktop
sudo apt install ros-humble-desktop

# Install Gazebo Harmonic
sudo apt install ros-humble-gazebo-*

# Install Gazebo simulation packages
sudo apt install gz-harmonic
```

#### 2. Verify Installation
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Check Gazebo version
gz --version

# Launch basic simulation
gz sim
```

#### 3. Install Additional Dependencies
```bash
# Install simulation utilities
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-ros-gz

# Install robot models and environments
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-universal-robot
```

### Gazebo Configuration for VLA Systems

#### 1. Configure Physics Engine
Create or modify `~/.gz/sim/gz_sim/config.yaml`:
```yaml
physics:
  engine: "libignition-physics-ode-plugin.so"
  step_size: 0.001
  real_time_factor: 1.0
  real_time_update_rate: 1000

rendering:
  engine: "ogre2"
  enable_realtime_rendering: true

gui:
  plugins:
    - name: "WorldControl"
    - name: "WorldStats"
    - name: "Scene3D"
```

#### 2. Set Up Simulation Worlds
Create VLA-specific simulation environments in `~/vla_ws/src/vla_simulation/worlds/`:

**Office World Example** (`office.world`):
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="office">
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- GUI -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>5 -5 3 0 0.4 2.356</pose>
      </camera>
    </gui>

    <!-- Light -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Office Furniture -->
    <include>
      <uri>model://table</uri>
      <pose>2 0 0 0 0 0</pose>
    </include>

    <include>
      <uri>model://cabinet</uri>
      <pose>-2 1 0 0 0 0</pose>
    </include>

    <!-- Objects for VLA Testing -->
    <model name="red_cup">
      <pose>2.1 0.1 0.8 0 0 0</pose>
      <include>
        <uri>model://coke_can</uri>
      </include>
    </model>

    <model name="blue_bowl">
      <pose>-1.9 1.1 0.8 0 0 0</pose>
      <include>
        <uri>model://bowl</uri>
      </include>
    </model>
  </world>
</sdf>
```

#### 3. Configure Sensor Plugins for VLA
For vision-language integration, ensure proper camera and microphone configurations:

**Example Robot with RGB-D Camera**:
```xml
<model name="vla_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass>10</mass>
      <inertia>
        <ixx>0.1</ixx> <ixy>0</ixy> <ixz>0</ixz>
        <iyy>0.1</iyy> <iyz>0</iyz> <izz>0.1</izz>
      </inertia>
    </inertial>

    <visual name="base_visual">
      <geometry>
        <box>
          <size>0.5 0.5 0.8</size>
        </box>
      </geometry>
    </visual>

    <collision name="base_collision">
      <geometry>
        <box>
          <size>0.5 0.5 0.8</size>
        </box>
      </geometry>
    </collision>
  </link>

  <!-- RGB-D Camera -->
  <link name="camera_link">
    <pose>0.2 0 0.3 0 0 0</pose>
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
    </sensor>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent>base_link</parent>
    <child>camera_link</child>
  </joint>

  <!-- Depth Camera -->
  <link name="depth_camera_link">
    <pose>0.2 0.05 0.3 0 0 0</pose>
    <sensor name="depth_camera" type="depth_camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
    </sensor>
  </link>

  <joint name="depth_camera_joint" type="fixed">
    <parent>base_link</parent>
    <child>depth_camera_link</child>
  </joint>
</model>
```

### Isaac Sim Setup

#### System Requirements for Isaac Sim
- **OS**: Ubuntu 20.04 LTS or 22.04 LTS
- **GPU**: NVIDIA RTX 3080 or better (Compute Capability 6.0+)
- **RAM**: 32GB minimum, 64GB recommended
- **CUDA**: CUDA 11.8 or later
- **Storage**: 20GB+ for Isaac Sim installation
- **Display**: 2K+ display recommended for UI

#### Installation Steps

##### 1. Install NVIDIA Drivers and CUDA
```bash
# Install NVIDIA drivers
sudo apt install nvidia-driver-535

# Install CUDA
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
sudo sh cuda_11.8.0_520.61.05_linux.run

# Add CUDA to PATH
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

##### 2. Download and Install Isaac Sim
```bash
# Create workspace
mkdir ~/isaac_sim_workspace
cd ~/isaac_sim_workspace

# Download Isaac Sim (requires NVIDIA Developer account)
# Follow instructions at: https://docs.omniverse.nvidia.com/isaacsim/latest/installation_guide/index.html

# Extract and install
tar -xzf isaac-sim-*.tar.gz
cd isaac-sim-*
./install.sh
```

##### 3. Verify Isaac Sim Installation
```bash
# Source the environment
source ~/isaac-sim/isaac-sim-*.env

# Launch Isaac Sim
./isaac-sim.launch.sh
```

### Isaac Sim Configuration for VLA Systems

#### 1. Configure Extensions for Robotics
Enable essential extensions in Isaac Sim:
- Isaac ROS Bridge
- Isaac Sim Sensors
- Isaac Navigation
- Isaac Manipulation

#### 2. Create VLA Simulation Environment
Create a new stage with VLA-specific objects and configurations:

**Example Isaac Sim Python Script** (`vla_scene.py`):
```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.sensor import Camera
import numpy as np

# Create world
world = World(stage_units_in_meters=1.0)

# Add robot
add_reference_to_stage(
    usd_path="/Isaac/Robots/TurtleBot3/turtlebot3.usd",
    prim_path="/World/Robot"
)

# Add objects for VLA testing
create_prim(
    prim_path="/World/RedCup",
    prim_type="Cylinder",
    position=np.array([2.0, 0.0, 0.0]),
    attributes={"radius": 0.05, "height": 0.1}
)

# Configure RGB camera
camera = Camera(
    prim_path="/World/Robot/base_link/camera",
    frequency=30,
    resolution=(640, 480)
)

# Add lighting
create_prim(
    prim_path="/World/Light",
    prim_type="DistantLight",
    position=np.array([0, 0, 10]),
    orientation=np.array([0, 0, 0, 1])
)

# Reset world
world.reset()
```

### ROS 2 Integration with Simulation

#### 1. Gazebo ROS 2 Bridge Configuration
```bash
# Install ROS 2 bridge
sudo apt install ros-humble-ros-gz-bridge

# Create bridge launch file (`launch/vla_gazebo_bridge.launch.py`):
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Gazebo simulation
        ExecuteProcess(
            cmd=['gz sim', '-r', 'office.sdf'],
            output='screen'
        ),

        # ROS 2 to Gazebo bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/robot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/model/robot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
            ],
            output='screen'
        ),

        # VLA processing nodes
        Node(
            package='vla_processing',
            executable='language_processor',
            output='screen'
        )
    ])
```

#### 2. Isaac Sim ROS 2 Bridge Setup
```bash
# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-* ros-humble-rosbridge-suite

# Configure ROS bridge for Isaac Sim
# Create launch file for Isaac Sim integration
```

### Performance Optimization

#### 1. Gazebo Performance Settings
- **Reduce physics update rate** for less critical simulations
- **Disable visualization** when running headless
- **Use simplified collision models** for faster physics
- **Limit number of active sensors** during development

#### 2. Isaac Sim Performance Settings
- **Enable multi-GPU rendering** if available
- **Adjust shadow quality** based on requirements
- **Use proxy shapes** during scene setup
- **Optimize USD stage complexity**

### Testing and Validation

#### 1. Basic Functionality Tests
```bash
# Test Gazebo simulation
cd ~/vla_ws
source install/setup.bash
ros2 launch vla_simulation vla_gazebo_bridge.launch.py

# Test camera feed
ros2 run image_view image_view image:=/camera/image

# Test robot control
ros2 topic pub /model/robot/cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```

#### 2. VLA Integration Tests
```bash
# Test voice input simulation
ros2 launch vla_simulation voice_simulation.launch.py

# Test perception pipeline
ros2 run vla_perception object_detection_node

# Test full VLA pipeline
ros2 launch vla_system vla_full_system.launch.py
```

### Troubleshooting Common Issues

#### 1. Gazebo Issues
- **Black screen**: Check graphics drivers and OpenGL support
- **Slow performance**: Reduce physics complexity or update rate
- **Model loading errors**: Verify model paths and permissions
- **ROS bridge failures**: Check topic names and message types

#### 2. Isaac Sim Issues
- **Crashes on startup**: Verify GPU drivers and CUDA installation
- **Missing extensions**: Reinstall or update Isaac Sim
- **Python import errors**: Check Python environment and paths
- **Performance issues**: Adjust rendering settings and complexity

### Best Practices

1. **Start simple**: Begin with basic worlds and gradually add complexity
2. **Validate regularly**: Test each component individually before integration
3. **Document configurations**: Keep track of working configurations
4. **Version control**: Use Git to track simulation environment changes
5. **Performance monitoring**: Monitor frame rates and resource usage
6. **Safety checks**: Implement safety constraints in simulation
7. **Reproducible environments**: Use containerization for consistent setups
8. **Regular backups**: Preserve working simulation configurations

This setup provides a solid foundation for developing and testing VLA systems in simulation environments before real-world deployment.
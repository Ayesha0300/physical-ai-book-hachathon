---
sidebar_position: 2
title: Modules
---

# Learning Modules

Explore our comprehensive curriculum designed to guide you through Physical AI and Humanoid Robotics. Each module builds upon the previous one, providing a structured learning path from foundational concepts to advanced applications.

import ModulesGrid from '@site/src/components/ModulesGrid';

<ModulesGrid modules={[
  {
    id: 'module-1',
    number: 1,
    title: 'ROS 2 – Robotic Nervous System',
    focusStatement: 'Understanding the Robot Operating System for distributed robotics',
    technologies: ['ROS 2', 'Python', 'C++', 'Middleware'],
    description: 'This module covers the fundamentals of ROS 2 for building distributed robotic systems...'
  },
  {
    id: 'module-2',
    number: 2,
    title: 'Gazebo – Physics Simulation',
    focusStatement: 'Creating realistic physics simulations for robot testing and development',
    technologies: ['Gazebo', 'Simulation', 'Physics', 'Robotics'],
    description: 'Learn to create realistic physics simulations for testing and validating your robots...'
  },
  {
    id: 'module-3',
    number: 3,
    title: 'Isaac Sim – High-Fidelity Robotics',
    focusStatement: 'Advanced simulation with NVIDIA Isaac Sim for high-fidelity robotics',
    technologies: ['Isaac Sim', 'NVIDIA', 'Simulation', 'AI'],
    description: 'Explore high-fidelity robotics simulation with NVIDIA Isaac Sim...'
  },
  {
    id: 'module-4',
    number: 4,
    title: 'Vision-Language-Action (VLA)',
    focusStatement: 'Integrating vision, language, and action for embodied AI systems',
    technologies: ['VLA', 'AI', 'Computer Vision', 'NLP'],
    description: 'Learn to build embodied AI systems that can perceive, understand, and act...'
  },
  {
    id: 'module-5',
    number: 5,
    title: 'Manipulation & Locomotion',
    focusStatement: 'Advanced robotics for manipulation and movement in physical space',
    technologies: ['Manipulation', 'Locomotion', 'Control', 'Planning'],
    description: 'Master the fundamentals of robot manipulation and locomotion...'
  },
  {
    id: 'module-6',
    number: 6,
    title: 'Embodied Agents',
    focusStatement: 'Creating intelligent agents that interact with the physical world',
    technologies: ['Embodied AI', 'Agents', 'RL', 'Navigation'],
    description: 'Build intelligent embodied agents that can interact with the physical world...'
  }
]} />
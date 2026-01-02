// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS2 Nervous System',
      items: [
        'module1-ros2/chapter1-why-robots-need-nervous-system',
        'module1-ros2/chapter2-ros2-architecture',
        'module1-ros2/chapter3-communication-primitives',
        'module1-ros2/chapter4-nodes-in-practice',
        'module1-ros2/chapter5-bridging-python-ai-agents',
        'module1-ros2/chapter6-robot-description-with-urdf',
        'module1-ros2/chapter7-digital-brain-to-physical-body',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      items: [
        'module2-digital-twin/chapter1-intro',
        'module2-digital-twin/chapter2-gazebo',
        'module2-digital-twin/chapter3-unity',
        'module2-digital-twin/chapter4-sensors',
        'module2-digital-twin/chapter5-environments',
        'module2-digital-twin/chapter6-calibration',
        'module2-digital-twin/chapter7-integration',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module3-isaac/chapter1-from-middleware-to-intelligence',
        'module3-isaac/chapter2-nvidia-isaac-platform-overview',
        'module3-isaac/chapter3-isaac-sim-photorealistic-simulation',
        'module3-isaac/chapter4-synthetic-data-generation',
        'module3-isaac/chapter5-visual-slam-humanoid-robots',
        'module3-isaac/chapter6-isaac-ros-hardware-accelerated-perception',
        'module3-isaac/chapter7-navigation-with-nav2',
        'module3-isaac/chapter8-coordinating-perception-localization-planning',
        'module3-isaac/module-summary',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'vla/vla-fundamentals',
        'vla/cognitive-planning',
        'vla/humanoid-capstone',
      ],
      collapsed: false,
    },
    'glossary',
  ],
};

export default sidebars;
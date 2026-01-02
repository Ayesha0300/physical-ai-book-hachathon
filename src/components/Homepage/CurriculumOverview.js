import React from 'react';
import clsx from 'clsx';
import './CurriculumOverview.module.css';

const CurriculumOverview = () => {
  const features = [
    {
      title: 'ROS 2 Control Systems',
      description: 'Learn to build robust control systems for humanoid robots using ROS 2',
      icon: '🤖',
    },
    {
      title: 'Simulation with Gazebo & Isaac',
      description: 'Master physics-based simulation environments for robot testing and development',
      icon: '🎮',
    },
    {
      title: 'Vision-Language-Action (VLA)',
      description: 'Implement multimodal AI systems that connect perception with action',
      icon: '👁️',
    },
    {
      title: 'Autonomous Humanoid Capstone',
      description: 'Integrate all concepts in a comprehensive humanoid robotics project',
      icon: '🎯',
    },
  ];

  return (
    <section className="curriculum-overview">
      <div className="container">
        <h2 className="curriculum-overview__title">Curriculum Overview</h2>
        <p className="curriculum-overview__subtitle">
          Master the essential skills needed to develop intelligent humanoid robots
        </p>
        <div className="curriculum-overview__features">
          {features.map((feature, index) => (
            <div key={index} className="curriculum-overview__feature">
              <div className="curriculum-overview__icon">{feature.icon}</div>
              <h3 className="curriculum-overview__feature-title">{feature.title}</h3>
              <p className="curriculum-overview__feature-description">
                {feature.description}
              </p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default CurriculumOverview;
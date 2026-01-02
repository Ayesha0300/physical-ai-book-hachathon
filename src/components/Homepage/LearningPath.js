import React from 'react';
import clsx from 'clsx';
import './LearningPath.module.css';

const LearningPath = () => {
  const steps = [
    {
      id: 1,
      title: 'Module 1: ROS 2 Control Systems',
      description: 'Learn to build robust control systems for humanoid robots using ROS 2',
    },
    {
      id: 2,
      title: 'Module 2: Simulation with Gazebo & Isaac',
      description: 'Master physics-based simulation environments for robot testing and development',
    },
    {
      id: 3,
      title: 'Module 3: Vision-Language-Action (VLA)',
      description: 'Implement multimodal AI systems that connect perception with action',
    },
    {
      id: 4,
      title: 'Module 4: Autonomous Humanoid Capstone',
      description: 'Integrate all concepts in a comprehensive humanoid robotics project',
    },
  ];

  return (
    <section className="learning-path">
      <div className="container">
        <h2 className="learning-path__title">Learning Path</h2>
        <p className="learning-path__subtitle">
          Follow the structured path to master Physical AI and Humanoid Robotics
        </p>
        <div className="learning-path__steps">
          {steps.map((step, index) => (
            <div key={step.id} className="learning-path__step">
              <div className="learning-path__step-number">
                {step.id}
              </div>
              <div className="learning-path__step-content">
                <h3 className="learning-path__step-title">{step.title}</h3>
                <p className="learning-path__step-description">{step.description}</p>
              </div>
              {index < steps.length - 1 && (
                <div className="learning-path__arrow">→</div>
              )}
            </div>
          ))}
        </div>
      </div>
    </section>
  );
};

export default LearningPath;
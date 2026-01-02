import React from 'react';
import clsx from 'clsx';
import './VisualDiagram.module.css';

const VisualDiagram = () => {
  const steps = [
    {
      id: 'ai',
      title: 'AI',
      description: 'Artificial Intelligence processing',
      icon: '🧠',
    },
    {
      id: 'perception',
      title: 'Perception',
      description: 'Sensory input and understanding',
      icon: '👁️',
    },
    {
      id: 'planning',
      title: 'Planning',
      description: 'Decision making and pathfinding',
      icon: '🧠',
    },
    {
      id: 'action',
      title: 'Action',
      description: 'Physical execution',
      icon: '🦾',
    },
  ];

  return (
    <section className="visual-diagram">
      <div className="container">
        <h2 className="visual-diagram__title">Physical AI Process</h2>
        <p className="visual-diagram__subtitle">
          How intelligent systems bridge the digital and physical worlds
        </p>
        <div className="visual-diagram__steps">
          {steps.map((step, index) => (
            <React.Fragment key={step.id}>
              <div className="visual-diagram__step">
                <div className="visual-diagram__icon">{step.icon}</div>
                <h3 className="visual-diagram__step-title">{step.title}</h3>
                <p className="visual-diagram__step-description">{step.description}</p>
              </div>
              {index < steps.length - 1 && (
                <div className="visual-diagram__arrow">→</div>
              )}
            </React.Fragment>
          ))}
        </div>
      </div>
    </section>
  );
};

export default VisualDiagram;
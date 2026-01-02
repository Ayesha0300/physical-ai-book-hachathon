import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import './ModuleCard.module.css';

const ModuleCard = ({ module, className, ...props }) => {
  const getDifficultyColor = (difficulty) => {
    switch (difficulty.toLowerCase()) {
      case 'beginner':
        return 'beginner';
      case 'intermediate':
        return 'intermediate';
      case 'advanced':
        return 'advanced';
      default:
        return 'beginner';
    }
  };

  return (
    <div className={clsx('module-card', className)} {...props}>
      <div className="module-card__header">
        <h3 className="module-card__title">{module.title}</h3>
        <span className={`module-card__badge module-card__badge--${getDifficultyColor(module.difficulty)}`}>
          {module.difficulty}
        </span>
      </div>
      <p className="module-card__description">{module.description}</p>
      <div className="module-card__tools">
        <strong>Tools:</strong>
        <div className="module-card__tools-list">
          {module.tools.map((tool, index) => (
            <span key={index} className="module-card__tool-badge">
              {tool}
            </span>
          ))}
        </div>
      </div>
      <Link
        className="module-card__link"
        to={module.route}>
        Explore Module
        <svg
          className="module-card__link-icon"
          style={{ width: '1rem', height: '1rem', marginLeft: '0.375rem' }}
          fill="none"
          stroke="currentColor"
          viewBox="0 0 24 24"
        >
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
        </svg>
      </Link>
    </div>
  );
};

export default ModuleCard;
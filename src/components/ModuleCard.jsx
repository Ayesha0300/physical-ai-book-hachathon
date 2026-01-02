import React from 'react';
import Link from '@docusaurus/Link';

const ModuleCard = ({ module }) => {
  const getTechnologyBadge = (tech) => {
    const techMap = {
      'ROS 2': 'ros',
      'ROS': 'ros',
      'Gazebo': 'gazebo',
      'Isaac Sim': 'isaac',
      'Isaac': 'isaac',
      'VLA': 'vla',
      'Computer Vision': 'vla',
      'LLMs': 'vla'
    };
    return techMap[tech] || 'ros';
  };

  const getDifficultyBadge = (difficulty) => {
    const diffMap = {
      'Beginner': 'beginner',
      'Intermediate': 'intermediate',
      'Advanced': 'advanced'
    };
    return diffMap[difficulty] || 'beginner';
  };

  return (
    <div className="card">
      <div style={{ display: 'flex', alignItems: 'flex-start', justifyContent: 'space-between', marginBottom: '1rem' }}>
        <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem' }}>
          {module.number && (
            <span style={{
              display: 'inline-flex',
              alignItems: 'center',
              justifyContent: 'center',
              width: '2rem',
              height: '2rem',
              borderRadius: '50%',
              background: 'var(--ifm-color-primary)',
              color: 'white',
              fontWeight: '700',
              fontSize: '1rem'
            }}>
              {module.number}
            </span>
          )}
          <h3 style={{
            fontSize: '1.375rem',
            fontWeight: '700',
            margin: 0,
            color: 'var(--ifm-color-content)'
          }}>
            {module.title}
          </h3>
        </div>
      </div>

      <p style={{
        color: 'var(--ifm-color-content-secondary)',
        marginBottom: '1rem',
        lineHeight: '1.6'
      }}>
        {module.focusStatement || module.description}
      </p>

      {(module.technologies || module.tools) && (
        <div style={{ marginBottom: '1rem' }}>
          <div style={{ display: 'flex', flexWrap: 'wrap', gap: '0.5rem' }}>
            {(module.technologies || module.tools).map((tech, index) => (
              <span
                key={index}
                className={`badge badge--${getTechnologyBadge(tech)}`}
              >
                {tech}
              </span>
            ))}
          </div>
        </div>
      )}

      {module.difficulty && (
        <div style={{ marginBottom: '1rem' }}>
          <span className={`badge badge--${getDifficultyBadge(module.difficulty)}`}>
            {module.difficulty}
          </span>
        </div>
      )}

      <div style={{ marginTop: 'auto', paddingTop: '1rem' }}>
        <Link
          to={module.route || `/modules/${module.id}`}
          className="button button--primary"
          style={{ textDecoration: 'none' }}
        >
          View Module
          <svg
            style={{ width: '1rem', height: '1rem', marginLeft: '0.375rem' }}
            fill="none"
            stroke="currentColor"
            viewBox="0 0 24 24"
          >
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth={2}
              d="M9 5l7 7-7 7"
            />
          </svg>
        </Link>
      </div>
    </div>
  );
};

export default ModuleCard;

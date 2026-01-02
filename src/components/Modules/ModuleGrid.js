import React from 'react';
import clsx from 'clsx';
import ModuleCard from './ModuleCard';
import './ModuleGrid.module.css';

const ModuleGrid = ({ modules, className, ...props }) => {
  return (
    <div className={clsx('module-grid-container', className)} {...props}>
      <div className="module-grid">
        {modules && modules.length > 0 ? (
          modules.map((module, index) => (
            <ModuleCard key={index} module={module} />
          ))
        ) : (
          <div className="module-grid__empty">
            <p>No modules available at this time.</p>
          </div>
        )}
      </div>
    </div>
  );
};

export default ModuleGrid;
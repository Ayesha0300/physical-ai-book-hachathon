import React from 'react';
import ModuleCard from './ModuleCard';

function ModuleGrid({ modules = [] }) {
  return (
    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-6">
      {modules.map((module, index) => (
        <ModuleCard
          key={module.id || index}
          moduleNumber={module.number}
          title={module.title}
          description={module.description}
          technologies={module.technologies}
          duration={module.duration}
          difficulty={module.difficulty}
          progress={module.progress}
          onClick={module.onClick}
        />
      ))}
    </div>
  );
}

export default ModuleGrid;
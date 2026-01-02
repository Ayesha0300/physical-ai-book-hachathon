import React from 'react';
import clsx from 'clsx';
import ModuleCard from './ModuleCard';

const ModulesGrid = ({ modules }) => {
  return (
    <div className="container mx-auto px-4 py-8">
      <div className="mb-8 text-center">
        <h1 className="text-4xl font-bold text-gray-900 dark:text-white mb-4">
          Learning Modules
        </h1>
        <p className="text-xl text-gray-600 dark:text-gray-300 max-w-3xl mx-auto">
          Explore our comprehensive curriculum designed to guide you through Physical AI and Humanoid Robotics
        </p>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
        {modules.map((module, index) => (
          <ModuleCard key={index} module={module} />
        ))}
      </div>
    </div>
  );
};

export default ModulesGrid;
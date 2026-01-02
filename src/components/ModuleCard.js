import React from 'react';

function ModuleCard({
  moduleNumber,
  title,
  description,
  technologies = [],
  duration,
  difficulty,
  progress,
  onClick
}) {
  // Determine difficulty color
  const getDifficultyColor = (difficulty) => {
    switch (difficulty?.toLowerCase()) {
      case 'easy':
        return 'bg-green-500';
      case 'medium':
        return 'bg-yellow-500';
      case 'hard':
        return 'bg-red-500';
      default:
        return 'bg-gray-500';
    }
  };

  // Determine technology badge color
  const getTechColor = (tech) => {
    switch (tech?.toLowerCase()) {
      case 'ros 2':
      case 'ros':
        return 'bg-tech-ros-500 text-white';
      case 'gazebo':
        return 'bg-tech-gazebo-500 text-white';
      case 'isaac':
      case 'nvidia isaac':
        return 'bg-tech-isaac-500 text-white';
      case 'vla':
      case 'vision-language-action':
        return 'bg-tech-vla-500 text-white';
      default:
        return 'bg-gray-200 dark:bg-gray-600';
    }
  };

  return (
    <div
      className="bg-dark-bg-800/50 backdrop-blur-lg border border-dark-bg-700/50 rounded-xl p-6 transition-all duration-300 ease-in-out hover:shadow-lg hover:shadow-accent-primary-500/20 hover:border-accent-primary-400/50 hover:scale-[1.02] cursor-pointer"
      onClick={onClick}
    >
      <div className="flex justify-between items-start mb-4">
        <div>
          <span className="text-accent-primary-400 font-mono text-sm font-semibold">
            Module {moduleNumber}
          </span>
          <h3 className="text-xl font-bold text-light-text-50 mt-1">{title}</h3>
        </div>
        {progress !== undefined && (
          <div className="text-right">
            <span className="text-sm text-light-text-200">{progress}%</span>
          </div>
        )}
      </div>

      <p className="text-light-text-200 mb-4">{description}</p>

      <div className="flex flex-wrap gap-2 mb-4">
        {technologies.map((tech, index) => (
          <span
            key={index}
            className={`px-2 py-1 rounded-full text-xs font-medium ${getTechColor(tech)}`}
          >
            {tech}
          </span>
        ))}
      </div>

      <div className="flex justify-between items-center">
        {duration && (
          <span className="text-sm text-light-text-200">
            {duration} read
          </span>
        )}

        {difficulty && (
          <div className="flex items-center">
            <span className="text-sm text-light-text-200 mr-2">Difficulty:</span>
            <div className={`w-3 h-3 rounded-full ${getDifficultyColor(difficulty)}`}></div>
          </div>
        )}
      </div>

      {progress !== undefined && (
        <div className="mt-4">
          <div className="w-full bg-dark-bg-700 rounded-full h-2">
            <div
              className="bg-accent-primary-500 h-2 rounded-full transition-all duration-300 ease-out"
              style={{ width: `${progress}%` }}
            ></div>
          </div>
        </div>
      )}
    </div>
  );
}

export default ModuleCard;
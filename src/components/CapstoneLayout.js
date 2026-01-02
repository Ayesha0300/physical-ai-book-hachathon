import React from 'react';

function CapstoneLayout({ title, children, steps = [] }) {
  return (
    <div className="max-w-4xl mx-auto p-6">
      {title && (
        <h1 className="text-3xl font-bold text-gray-900 dark:text-white mb-8">{title}</h1>
      )}

      {steps.length > 0 && (
        <div className="mb-8">
          <h2 className="text-xl font-semibold text-gray-900 dark:text-white mb-4">Steps</h2>
          <ol className="space-y-4">
            {steps.map((step, index) => (
              <li key={index} className="flex items-start">
                <span className="flex-shrink-0 w-8 h-8 rounded-full bg-accent-blue dark:bg-accent-cyan flex items-center justify-center text-white font-bold mr-3">
                  {index + 1}
                </span>
                <div className="ml-2">
                  <h3 className="font-medium text-gray-900 dark:text-white">{step.title}</h3>
                  {step.description && (
                    <p className="text-gray-600 dark:text-gray-300 mt-1">{step.description}</p>
                  )}
                </div>
              </li>
            ))}
          </ol>
        </div>
      )}

      <div className="bg-white dark:bg-dark-bg rounded-lg shadow-md p-6">
        {children}
      </div>
    </div>
  );
}

export default CapstoneLayout;
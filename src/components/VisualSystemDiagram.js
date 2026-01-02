import React from 'react';

function VisualSystemDiagram({ title, description, children }) {
  return (
    <div className="my-8 p-6 bg-gray-50 dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded-lg">
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-semibold text-gray-900 dark:text-white">
          {title || 'System Diagram'}
        </h3>
        <div className="bg-gray-200 dark:bg-gray-700 border-2 border-dashed rounded-xl w-16 h-16" />
      </div>

      {description && (
        <p className="text-gray-700 dark:text-gray-300 mb-4">
          {description}
        </p>
      )}

      <div className="border-t border-gray-200 dark:border-gray-700 pt-4">
        {children || (
          <div className="text-center py-8 text-gray-500 dark:text-gray-400">
            <p>System diagram visualization would appear here</p>
            <p className="text-sm mt-2">This is a placeholder for the visual system diagram</p>
          </div>
        )}
      </div>
    </div>
  );
}

export default VisualSystemDiagram;
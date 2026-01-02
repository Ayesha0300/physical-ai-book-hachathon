import React from 'react';

function LoadingSpinner({ size = 'md', className = '' }) {
  const sizeClasses = {
    sm: 'w-4 h-4',
    md: 'w-8 h-8',
    lg: 'w-12 h-12',
    xl: 'w-16 h-16'
  };

  const spinnerSize = sizeClasses[size] || sizeClasses.md;

  return (
    <div className={`flex justify-center items-center ${className}`}>
      <div
        className={`${spinnerSize} border-4 border-gray-200 dark:border-gray-700 rounded-full animate-spin`}
        style={{
          borderTopColor: 'currentColor',
          borderRightColor: 'currentColor',
          borderLeftColor: 'transparent',
          borderBottomColor: 'transparent',
        }}
      ></div>
    </div>
  );
}

export default LoadingSpinner;
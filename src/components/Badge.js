import React from 'react';

function Badge({
  children,
  variant = 'default',
  size = 'md',
  className = '',
  ...props
}) {
  const baseClasses = 'inline-flex items-center rounded-full font-medium';

  const variants = {
    default: 'bg-dark-bg-700 text-light-text-50',
    primary: 'bg-accent-primary-500 text-white',
    secondary: 'bg-accent-secondary-500 text-white',
    success: 'bg-accent-success-500 text-white',
    warning: 'bg-accent-warning-500 text-white',
    danger: 'bg-accent-error-500 text-white',
    // Technology-specific variants
    ros: 'bg-tech-ros-500 text-white',
    gazebo: 'bg-tech-gazebo-500 text-white',
    isaac: 'bg-tech-isaac-500 text-white',
    vla: 'bg-tech-vla-500 text-white',
    easy: 'bg-green-500 text-white',
    medium: 'bg-yellow-500 text-white',
    hard: 'bg-red-500 text-white'
  };

  const sizes = {
    sm: 'px-2 py-0.5 text-xs',
    md: 'px-2.5 py-0.5 text-sm',
    lg: 'px-3 py-1 text-sm'
  };

  const classes = [
    baseClasses,
    variants[variant],
    sizes[size],
    className
  ].join(' ');

  return (
    <span className={classes} {...props}>
      {children}
    </span>
  );
}

export default Badge;
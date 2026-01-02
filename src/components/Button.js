import React from 'react';

function Button({
  children,
  variant = 'primary',
  size = 'md',
  className = '',
  onClick,
  disabled = false,
  ...props
}) {
  const baseClasses = 'font-semibold rounded-lg transition-all duration-200 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-offset-dark-bg-900';

  const variants = {
    primary: 'bg-accent-primary-500 hover:bg-accent-primary-600 text-white focus:ring-accent-primary-500',
    secondary: 'bg-dark-bg-700 hover:bg-dark-bg-600 text-light-text-50 focus:ring-accent-primary-500 border border-dark-bg-600',
    ghost: 'bg-transparent hover:bg-dark-bg-700 text-light-text-50 focus:ring-accent-primary-500',
    success: 'bg-accent-success-500 hover:bg-accent-success-600 text-white focus:ring-accent-success-500',
    warning: 'bg-accent-warning-500 hover:bg-accent-warning-600 text-white focus:ring-accent-warning-500',
    danger: 'bg-accent-error-500 hover:bg-accent-error-600 text-white focus:ring-accent-error-500'
  };

  const sizes = {
    sm: 'px-3 py-1.5 text-sm',
    md: 'px-4 py-2 text-sm',
    lg: 'px-6 py-3 text-base',
    xl: 'px-8 py-4 text-lg'
  };

  const disabledClasses = disabled
    ? 'opacity-50 cursor-not-allowed pointer-events-none'
    : 'cursor-pointer transform hover:scale-[1.02] active:scale-[0.98]';

  const classes = [
    baseClasses,
    variants[variant],
    sizes[size],
    disabledClasses,
    className
  ].join(' ');

  return (
    <button
      className={classes}
      onClick={onClick}
      disabled={disabled}
      {...props}
    >
      {children}
    </button>
  );
}

export default Button;
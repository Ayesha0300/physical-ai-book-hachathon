import React from 'react';
import clsx from 'clsx';
import './Button.module.css';

const Button = ({ children, variant = 'primary', size = 'md', className, ...props }) => {
  return (
    <button
      className={clsx(
        'button',
        `button--${variant}`,
        `button--${size}`,
        className
      )}
      {...props}
    >
      {children}
    </button>
  );
};

export default Button;
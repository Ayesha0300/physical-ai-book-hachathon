import React from 'react';
import clsx from 'clsx';
import './Card.module.css';

const Card = ({ children, className, ...props }) => {
  return (
    <div className={clsx('card', className)} {...props}>
      {children}
    </div>
  );
};

export default Card;
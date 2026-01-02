import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';

const Logo = ({ logo, onClick, className = '' }) => {
  const handleClick = (e) => {
    if (onClick) {
      onClick(e);
    }
  };

  return (
    <Link
      to="/"
      onClick={handleClick}
      className={clsx(
        'flex items-center space-x-2',
        className
      )}
      aria-label={logo.altText || 'Physical AI Book Logo'}
    >
      <div className="w-8 h-8 flex items-center justify-center">
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 100 100"
          className="w-8 h-8"
          fill="none"
          stroke="currentColor"
        >
          {/* Humanoid silhouette */}
          <circle cx="50" cy="30" r="10" stroke={logo.colors?.primary || '#0ea5e9'} strokeWidth="2"/>
          <path d="M50 40 L50 70" stroke={logo.colors?.primary || '#0ea5e9'} strokeWidth="2"/>
          <path d="M40 55 L60 55" stroke={logo.colors?.primary || '#0ea5e9'} strokeWidth="2"/>
          <path d="M40 70 L45 85" stroke={logo.colors?.primary || '#0ea5e9'} strokeWidth="2"/>
          <path d="M60 70 L55 85" stroke={logo.colors?.primary || '#0ea5e9'} strokeWidth="2"/>

          {/* Neural network motif */}
          <circle cx="30" cy="20" r="3" fill={logo.colors?.accent || '#06b6d4'}/>
          <circle cx="70" cy="20" r="3" fill={logo.colors?.accent || '#06b6d4'}/>
          <circle cx="25" cy="40" r="3" fill={logo.colors?.accent || '#06b6d4'}/>
          <circle cx="75" cy="40" r="3" fill={logo.colors?.accent || '#06b6d4'}/>
          <circle cx="50" cy="50" r="3" fill={logo.colors?.accent || '#06b6d4'}/>

          {/* Connecting lines */}
          <line x1="30" y1="20" x2="25" y2="40" stroke={logo.colors?.accent || '#06b6d4'} strokeWidth="1"/>
          <line x1="30" y1="20" x2="50" y2="50" stroke={logo.colors?.accent || '#06b6d4'} strokeWidth="1"/>
          <line x1="70" y1="20" x2="75" y2="40" stroke={logo.colors?.accent || '#06b6d4'} strokeWidth="1"/>
          <line x1="70" y1="20" x2="50" y2="50" stroke={logo.colors?.accent || '#06b6d4'} strokeWidth="1"/>
          <line x1="25" y1="40" x2="50" y2="50" stroke={logo.colors?.accent || '#06b6d4'} strokeWidth="1"/>
          <line x1="75" y1="40" x2="50" y2="50" stroke={logo.colors?.accent || '#06b6d4'} strokeWidth="1"/>
        </svg>
      </div>
      <span className="text-xl font-bold text-light-text-50">
        Physical AI Book
      </span>
    </Link>
  );
};

export default Logo;
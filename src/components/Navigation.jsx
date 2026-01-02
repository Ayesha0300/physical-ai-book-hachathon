import React, { useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';

const Navigation = ({ items = [], activeRoute = '' }) => {
  const location = useLocation();
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);

  const isActive = (route) => {
    // For exact match
    if (route === '/') {
      return location.pathname === route;
    }
    // For other routes, check if location starts with the route
    return location.pathname.startsWith(route);
  };

  return (
    <nav className="bg-dark-bg-900 border-b border-dark-bg-700">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="flex justify-between h-16">
          <div className="flex items-center">
            <div className="flex-shrink-0 flex items-center">
              <span className="text-accent-primary-400 font-bold text-xl">Physical AI Book</span>
            </div>
            <div className="hidden md:ml-6 md:flex md:space-x-8">
              {items.map((item, index) => (
                <Link
                  key={index}
                  to={item.route}
                  className={clsx(
                    'inline-flex items-center px-1 pt-1 border-b-2 text-sm font-medium',
                    isActive(item.route)
                      ? 'border-accent-primary-500 text-light-text-50'
                      : 'border-transparent text-light-text-200 hover:border-dark-bg-600 hover:text-light-text-100'
                  )}
                >
                  {item.label}
                </Link>
              ))}
            </div>
          </div>
          <div className="-mr-2 flex items-center md:hidden">
            <button
              onClick={() => setMobileMenuOpen(!mobileMenuOpen)}
              className="inline-flex items-center justify-center p-2 rounded-md text-light-text-200 hover:text-light-text-100 hover:bg-dark-bg-700 focus:outline-none"
            >
              <svg
                className="h-6 w-6"
                xmlns="http://www.w3.org/2000/svg"
                fill="none"
                viewBox="0 0 24 24"
                stroke="currentColor"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d={mobileMenuOpen ? 'M6 18L18 6M6 6l12 12' : 'M4 6h16M4 12h16M4 18h16'}
                />
              </svg>
            </button>
          </div>
        </div>
      </div>

      {mobileMenuOpen && (
        <div className="md:hidden">
          <div className="pt-2 pb-3 space-y-1">
            {items.map((item, index) => (
              <Link
                key={index}
                to={item.route}
                className={clsx(
                  'block pl-3 pr-4 py-2 border-l-4 text-base font-medium',
                  isActive(item.route)
                    ? 'bg-dark-bg-800/50 border-accent-primary-500 text-accent-primary-400'
                    : 'border-transparent text-light-text-200 hover:bg-dark-bg-700 hover:border-dark-bg-600 hover:text-light-text-100'
                )}
              >
                {item.label}
              </Link>
            ))}
          </div>
        </div>
      )}
    </nav>
  );
};

export default Navigation;
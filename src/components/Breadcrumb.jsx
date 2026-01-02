import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useColorMode } from '@docusaurus/theme-common';

const Breadcrumb = ({ items }) => {
  const { colorMode } = useColorMode();

  return (
    <nav className="mb-6" aria-label="Breadcrumb">
      <ol className={clsx(
        'flex items-center text-sm',
        colorMode === 'dark' ? 'text-gray-300' : 'text-gray-600'
      )}>
        {items.map((item, index) => (
          <li key={index} className="flex items-center">
            {index > 0 && (
              <svg
                className={clsx(
                  'w-4 h-4 mx-2',
                  colorMode === 'dark' ? 'text-gray-400' : 'text-gray-400'
                )}
                fill="currentColor"
                viewBox="0 0 20 20"
              >
                <path
                  fillRule="evenodd"
                  d="M7.293 14.707a1 1 0 010-1.414L10.586 10 7.293 6.707a1 1 0 011.414-1.414l4 4a1 1 0 010 1.414l-4 4a1 1 0 01-1.414 0z"
                  clipRule="evenodd"
                />
              </svg>
            )}
            {item.href ? (
              <Link
                href={item.href}
                className={clsx(
                  'hover:underline',
                  index === items.length - 1
                    ? colorMode === 'dark'
                      ? 'text-white font-medium'
                      : 'text-gray-900 font-medium'
                    : colorMode === 'dark'
                      ? 'text-blue-400'
                      : 'text-blue-600'
                )}
              >
                {item.label}
              </Link>
            ) : (
              <span
                className={clsx(
                  'font-medium',
                  colorMode === 'dark' ? 'text-white' : 'text-gray-900'
                )}
              >
                {item.label}
              </span>
            )}
          </li>
        ))}
      </ol>
    </nav>
  );
};

export default Breadcrumb;
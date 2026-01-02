import React from 'react';

function SectionAnchor({ id, title, children }) {
  return (
    <div id={id} className="relative">
      {children}
      {id && (
        <a
          href={`#${id}`}
          className="absolute -left-6 no-underline opacity-0 hover:opacity-100 transition-opacity duration-200"
          aria-label={`Link to section: ${title || id}`}
        >
          <svg
            className="w-5 h-5 text-gray-400 dark:text-gray-500"
            fill="none"
            stroke="currentColor"
            viewBox="0 0 24 24"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth="2"
              d="M13.828 10.172a4 4 0 00-5.656 0l-4 4a4 4 0 105.656 5.656l1.102-1.101m-.758-4.899a4 4 0 005.656 0l4-4a4 4 0 00-5.656-5.656l-1.1 1.1"
            ></path>
          </svg>
        </a>
      )}
    </div>
  );
}

export default SectionAnchor;
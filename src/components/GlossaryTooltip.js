import React, { useState, useRef, useEffect } from 'react';

function GlossaryTooltip({ term, definition, children }) {
  const [showTooltip, setShowTooltip] = useState(false);
  const tooltipRef = useRef(null);

  useEffect(() => {
    const handleClickOutside = (event) => {
      if (tooltipRef.current && !tooltipRef.current.contains(event.target)) {
        setShowTooltip(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  return (
    <div className="relative inline-block" ref={tooltipRef}>
      <button
        type="button"
        className="text-accent-blue dark:text-accent-cyan hover:underline font-medium focus:outline-none focus:ring-2 focus:ring-blue-500 rounded"
        onMouseEnter={() => setShowTooltip(true)}
        onMouseLeave={() => setShowTooltip(false)}
        onClick={() => setShowTooltip(!showTooltip)}
        aria-describedby="glossary-tooltip"
        aria-expanded={showTooltip}
      >
        {children || term}
      </button>
      {showTooltip && (
        <div
          id="glossary-tooltip"
          className="absolute z-10 w-64 p-3 text-sm text-white bg-gray-800 rounded-lg shadow-lg dark:bg-gray-900 mt-1"
          role="tooltip"
          style={{ left: '50%', transform: 'translateX(-50%)', top: '100%' }}
        >
          <div className="font-bold text-accent-cyan">{term}</div>
          <div className="mt-1">{definition}</div>
          <div className="absolute bottom-full left-1/2 transform -translate-x-1/2 w-0 h-0 border-l-4 border-r-4 border-b-4 border-l-transparent border-r-transparent border-b-gray-800 dark:border-b-gray-900"></div>
        </div>
      )}
    </div>
  );
}

export default GlossaryTooltip;
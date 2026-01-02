import React, { useState } from 'react';

function CodeBlock({ children, language = '', showLineNumbers = false }) {
  const [copied, setCopied] = useState(false);

  const copyToClipboard = () => {
    navigator.clipboard.writeText(children).then(() => {
      setCopied(true);
      setTimeout(() => setCopied(false), 2000); // Reset after 2 seconds
    });
  };

  const codeString = typeof children === 'string' ? children : children.props.children;

  return (
    <div className="relative group">
      <pre className={`language-${language} rounded-lg overflow-x-auto p-4 bg-gray-800 dark:bg-gray-900 text-gray-100 text-sm`}>
        <code className={`language-${language}`}>
          {codeString}
        </code>
      </pre>
      <button
        type="button"
        className="absolute top-2 right-2 p-2 bg-gray-700 dark:bg-gray-800 text-gray-200 hover:bg-gray-600 dark:hover:bg-gray-700 rounded-md text-xs opacity-0 group-hover:opacity-100 transition-opacity duration-200 focus:outline-none focus:ring-2 focus:ring-accent-blue"
        onClick={copyToClipboard}
        aria-label={copied ? "Copied!" : "Copy code to clipboard"}
      >
        {copied ? (
          <span className="text-green-400">✓</span>
        ) : (
          <svg
            className="w-4 h-4"
            fill="none"
            stroke="currentColor"
            viewBox="0 0 24 24"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth="2"
              d="M8 16H6a2 2 0 01-2-2V6a2 2 0 012-2h8a2 2 0 012 2v2m-6 12h8a2 2 0 002-2v-8a2 2 0 00-2-2h-8a2 2 0 00-2 2v8a2 2 0 002 2z"
            ></path>
          </svg>
        )}
      </button>
    </div>
  );
}

export default CodeBlock;
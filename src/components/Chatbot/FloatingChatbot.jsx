import React, { useState, useEffect } from 'react';
import Chatbot from './Chatbot';
import './FloatingChatbot.css';

/**
 * Floating Chatbot Component
 * Provides a persistent chatbot widget that appears on all pages
 */
const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isMounted, setIsMounted] = useState(false);

  // Initialize component and manage body class
  useEffect(() => {
    setIsMounted(true);

    // Set initial body class based on isOpen state
    if (isOpen) {
      document.body.classList.add('chatbot-open');
    }

    // Clean up on unmount
    return () => {
      document.body.classList.remove('chatbot-open');
    };
  }, []);

  // Manage body class when isOpen changes
  useEffect(() => {
    if (isOpen) {
      document.body.classList.add('chatbot-open');
    } else {
      document.body.classList.remove('chatbot-open');
    }
  }, [isOpen]);

  const handleClose = () => {
    setIsOpen(false);
  };

  const handleToggle = () => {
    setIsOpen(!isOpen);
  };

  // Close chatbot when clicking outside (on backdrop)
  const handleBackdropClick = (e) => {
    if (e.target.classList.contains('floating-chatbot-backdrop')) {
      handleClose();
    }
  };

  // Handle Escape key to close chatbot
  useEffect(() => {
    const handleEscKey = (e) => {
      if (e.key === 'Escape' && isOpen) {
        handleClose();
      }
    };

    document.addEventListener('keydown', handleEscKey);
    return () => document.removeEventListener('keydown', handleEscKey);
  }, [isOpen]);

  return (
    <>
      {/* Floating chat button */}
      {!isOpen && isMounted && (
        <button
          className="floating-chat-button"
          onClick={handleToggle}
          aria-label="Open chatbot"
          title="Ask questions about the book content"
        >
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              d="M21 15C21 15.5304 20.7893 16.0391 20.4142 16.4142C20.0391 16.7893 19.5304 17 19 17H16L12.5 20.5C12.3011 20.6989 12.0315 20.817 11.7499 20.83C11.4684 20.843 11.1982 20.7499 10.9999 20.573C10.8017 20.396 10.691 20.149 10.691 19.887C10.691 19.625 10.8017 19.378 10.9999 19.201L14.5 15.5H15V10C15 8.67392 14.4732 7.40215 13.5355 6.46447C12.5979 5.52678 11.3261 5 10 5C8.67392 5 7.40215 5.52678 6.46447 6.46447C5.52678 7.40215 5 8.67392 5 10V11.5H5.5C5.63261 11.5 5.75979 11.5527 5.8536 11.6464C5.94741 11.7402 6 11.8674 6 12C6 12.1326 5.94741 12.2598 5.8536 12.3536C5.75979 12.4474 5.63261 12.5 5.5 12.5H5C4.46957 12.5 3.96086 12.7107 3.58579 13.0858C3.21071 13.4609 3 13.9696 3 14.5V17C3 17.5304 3.21071 18.0391 3.58579 18.4142C3.96086 18.7893 4.46957 19 5 19H11.5C12.0304 19 12.5391 18.7893 12.9142 18.4142C13.2893 18.0391 13.5 17.5304 13.5 17V16.5H14V16C14 15.4696 14.2107 14.9609 14.5858 14.5858C14.9609 14.2107 15.4696 14 16 14H19C19.5304 14 20.0391 13.7893 20.4142 13.4142C20.7893 13.0391 21 12.5304 21 12V15ZM9.5 10C9.5 9.33696 9.23661 8.70107 8.76777 8.23223C8.29893 7.76339 7.66304 7.5 7 7.5C6.33696 7.5 5.70107 7.76339 5.23223 8.23223C4.76339 8.70107 4.5 9.33696 4.5 10V11.5H9.5V10Z"
              fill="currentColor"
            />
          </svg>
          <span className="chat-unread-badge">1</span>
        </button>
      )}

      {/* Chatbot modal */}
      {isOpen && isMounted && (
        <div
          className={`floating-chatbot-backdrop ${isMounted ? 'show' : ''}`}
          onClick={handleBackdropClick}
        >
          <div className="floating-chatbot-container">
            <div className="floating-chatbot-header">
              <h3>Physical AI Assistant</h3>
              <button
                className="floating-chatbot-close"
                onClick={handleClose}
                aria-label="Close chat"
              >
                ×
              </button>
            </div>
            <div className="floating-chatbot-content">
              <Chatbot />
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default FloatingChatbot;
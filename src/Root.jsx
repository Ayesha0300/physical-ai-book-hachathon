import React from 'react';
import FloatingChatbot from './components/Chatbot/FloatingChatbot';

// Root component that wraps the entire app
export default function Root({ children }) {
  return (
    <>
      {children}
      <FloatingChatbot />
    </>
  );
}
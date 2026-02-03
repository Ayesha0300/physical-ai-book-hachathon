import React, { useState, useRef, useEffect } from 'react';
import './Chatbot.css';

/**
 * Chatbot Component for the RAG Chatbot frontend
 * Provides a full-page chat interface for interacting with the RAG backend
 */
const Chatbot = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [restrictToSelection, setRestrictToSelection] = useState(false);
  const [conversationId, setConversationId] = useState(null);
  const messagesEndRef = useRef(null);

  // Scroll to bottom of messages when new messages are added
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  /**
   * Handle sending a message to the backend
   */
  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message to chat
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request payload
      const requestBody = {
        query: inputValue,
        selected_text: selectedText || null,
        restrict_to_selection: restrictToSelection,
        conversation_id: conversationId || null
      };

      // Send request to backend
      const response = await fetch('https://ayeshabashir030-deploy.hf.space/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
      });

      const responseData = await response.json();

      if (responseData.success) {
        // Add bot response to chat
        const botMessage = {
          id: Date.now() + 1,
          text: responseData.data.response,
          sender: 'bot',
          sources: responseData.data.sources || [],
          confidence: responseData.data.confidence,
          timestamp: responseData.data.timestamp,
          conversationId: responseData.data.conversation_id
        };

        setMessages(prev => [...prev, botMessage]);

        // Update conversation ID if it was returned
        if (responseData.data.conversation_id && !conversationId) {
          setConversationId(responseData.data.conversation_id);
        }
      } else {
        // Handle error response
        const errorMessage = {
          id: Date.now() + 1,
          text: `Error: ${responseData.error.message}`,
          sender: 'system',
          timestamp: new Date().toISOString()
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        id: Date.now() + 1,
        text: `Connection error: Could not reach the chatbot backend. Please ensure the backend server is running.`,
        sender: 'system',
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Handle form submission
   */
  const handleSubmit = (e) => {
    e.preventDefault();
    sendMessage();
  };

  /**
   * Handle key press in input field
   */
  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  /**
   * Handle text selection on the page
   */
  const handleTextSelection = () => {
    const selectedText = window.getSelection().toString().trim();
    if (selectedText) {
      setSelectedText(selectedText);
    }
  };

  /**
   * Clear the current conversation
   */
  const clearConversation = () => {
    setMessages([]);
    setConversationId(null);
    setSelectedText('');
  };

  return (
    <div className="chatbot-container">
      <div className="chatbot-body">
        <div className="messages-container">
          {messages.length === 0 ? (
            <div className="welcome-message">
              <h3>Welcome to the RAG Chatbot!</h3>
              <p>Ask me questions about the book content and I'll find relevant information for you.</p>
              {selectedText && (
                <div className="selected-text-preview">
                  <strong>Selected text:</strong> "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
                </div>
              )}
            </div>
          ) : (
            messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.sender}-message`}
              >
                <div className="message-content">
                  <div className="message-text">{message.text}</div>
                  {message.sources && message.sources.length > 0 && (
                    <div className="message-sources">
                      <strong>Sources:</strong> {message.sources.slice(0, 3).join(', ')}
                      {message.sources.length > 3 && ` +${message.sources.length - 3} more`}
                    </div>
                  )}
                  {message.confidence !== undefined && (
                    <div className="message-confidence">
                      Confidence: {(message.confidence * 100).toFixed(1)}%
                    </div>
                  )}
                </div>
                <div className="message-timestamp">
                  {new Date(message.timestamp).toLocaleTimeString()}
                </div>
              </div>
            ))
          )}
          {isLoading && (
            <div className="message bot-message">
              <div className="message-content">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <div className="input-container">
          {selectedText && (
            <div className="selection-controls">
              <label>
                <input
                  type="checkbox"
                  checked={restrictToSelection}
                  onChange={(e) => setRestrictToSelection(e.target.checked)}
                />
                <span>Restrict to selected text only</span>
              </label>
              <button
                onClick={() => setSelectedText('')}
                className="remove-selection-btn"
              >
                Remove Selection
              </button>
            </div>
          )}

          <form onSubmit={handleSubmit} className="input-form">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about the book content..."
              rows="3"
              disabled={isLoading}
              onSelect={handleTextSelection}
            />
            <button type="submit" disabled={isLoading || !inputValue.trim()}>
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </form>

          <div className="instructions">
            <p>Select text on the page to ask specific questions about it</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Chatbot;
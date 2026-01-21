# RAG Chatbot Component Documentation

## Overview
The RAG (Retrieval-Augmented Generation) Chatbot component provides an interactive interface for users to ask questions about book content and receive AI-generated responses based on retrieved information.

## Features
- **Natural Language Queries**: Users can ask questions in plain English about the book content
- **Context-Aware Responses**: AI responses are grounded in the actual book content
- **Text Selection**: Users can select specific text on the page and ask targeted questions
- **Multi-turn Conversations**: Support for follow-up questions with conversation context
- **Source Attribution**: Responses include sources from the book content
- **Confidence Indicators**: Shows confidence level of the AI responses

## Installation
The chatbot component is integrated into the existing Docusaurus documentation site. No additional installation is required.

## Usage
1. Navigate to the page where the chatbot is embedded
2. Type your question in the input field
3. Press Enter or click Send to submit
4. Review the AI-generated response
5. Optionally select text on the page and check "Restrict to selected text only" to ask specific questions about that content

## Props
The Chatbot component accepts no props as it's designed to be self-contained.

## API Integration
The component communicates with the backend API at `http://localhost:8000/api/v1/chat` using the following request format:
```json
{
  "query": "User's question",
  "selected_text": "Text selected by user (optional)",
  "restrict_to_selection": true/false,
  "conversation_id": "ID of current conversation (optional)"
}
```

## Error Handling
- Network errors are displayed to the user with instructions to check backend status
- Invalid queries are rejected with helpful error messages
- Timeout errors are handled gracefully with retry suggestions

## Styling
The component uses its own CSS file (Chatbot.css) and follows the site's design principles for consistency.

## Browser Compatibility
- Modern browsers supporting ES6+ features
- Chrome, Firefox, Safari, Edge (latest versions)

## Accessibility
- Keyboard navigation support
- Screen reader compatibility
- High contrast mode support
- Proper ARIA labels for interactive elements

## Performance
- Messages are rendered efficiently using React virtualization
- API calls are debounced to prevent excessive requests
- Loading states provide user feedback during processing

## Security
- All API communications are done over HTTPS in production
- User queries are sanitized before processing
- No sensitive data is stored locally without consent
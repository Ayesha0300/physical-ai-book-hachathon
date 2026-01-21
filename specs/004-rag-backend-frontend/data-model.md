# Data Model: RAG Backend-Frontend Integration

## Core Entities

### 1. ChatQuery
**Description**: Represents a user's input to the chat system

**Fields**:
- `query`: string (required) - The user's question or request
- `selected_text`: string (optional) - Specific text selected by the user for context
- `restrict_to_selection`: boolean (optional, default: false) - Whether to limit responses to selected text
- `conversation_id`: string (optional) - ID for maintaining conversation context
- `timestamp`: datetime (auto-generated) - When the query was submitted

**Validation**:
- `query` must be 1-2000 characters
- `selected_text` must be 1-10000 characters if provided
- `query` cannot be empty or whitespace only

### 2. ChatResponse
**Description**: The system's response to a user's query

**Fields**:
- `response`: string (required) - The AI-generated answer
- `sources`: Array<string> (optional) - List of document sources used
- `confidence`: number (optional) - Confidence score 0-1
- `conversation_id`: string (required) - ID for conversation continuity
- `timestamp`: datetime (auto-generated) - When the response was generated
- `error`: object (optional) - Error information if request failed

**Validation**:
- `response` must be 1-10000 characters
- `sources` length must be 0-20 items
- `confidence` must be between 0 and 1 if provided

### 3. VectorSearchResult
**Description**: Results from the vector database search

**Fields**:
- `content`: string (required) - The retrieved text content
- `score`: number (required) - Similarity score 0-1
- `metadata`: object (optional) - Additional document metadata
- `document_id`: string (required) - Unique identifier for the document chunk

**Validation**:
- `content` must be 1-5000 characters
- `score` must be between 0 and 1
- `document_id` must be unique and non-empty

### 4. ConversationContext
**Description**: Maintains state for multi-turn conversations

**Fields**:
- `conversation_id`: string (required) - Unique identifier for the conversation
- `messages`: Array<object> (required) - History of messages in the conversation
- `created_at`: datetime (auto-generated) - When conversation started
- `updated_at`: datetime (auto-generated) - When last updated
- `max_messages`: number (optional, default: 10) - Maximum messages to retain

**Validation**:
- `conversation_id` must be unique
- `messages` length must be 1-50 items
- `max_messages` must be 1-100 if provided

## Relationships

### ChatQuery → ChatResponse
- One-to-one relationship for each query-response pair
- Linked by `conversation_id`

### ChatQuery → VectorSearchResult
- One-to-many relationship during retrieval phase
- Query may match multiple vector results
- Results filtered based on `restrict_to_selection` flag

### ChatQuery → ConversationContext
- Many-to-one relationship
- Multiple queries can belong to the same conversation context
- Enables multi-turn conversations

## State Transitions

### Chat Processing Flow
1. **Query Received**: ChatQuery created and validated
2. **Context Retrieved**: ConversationContext loaded (if exists)
3. **Vector Search**: VectorSearchResult(s) retrieved from Qdrant
4. **Agent Processing**: AI generates response using context and search results
5. **Response Formed**: ChatResponse created with sources and metadata
6. **Context Updated**: ConversationContext updated with new messages
7. **Response Returned**: ChatResponse sent to frontend

## API-Specific Models

### ChatRequest (API Input)
Extends ChatQuery with API-specific fields:
- `user_id`: string (optional) - Identifier for the requesting user
- `request_id`: string (auto-generated) - Unique request identifier

### ChatSuccessResponse (API Output)
- `success`: boolean (always true for success)
- `data`: ChatResponse (required) - The response data
- `request_id`: string (required) - Echo of the request ID

### ChatErrorResponse (API Output)
- `success`: boolean (always false for error)
- `error`: object (required) - Error details
  - `code`: string (required) - Error code
  - `message`: string (required) - Human-readable error message
  - `details`: object (optional) - Additional error details
- `request_id`: string (required) - Echo of the request ID
# Data Model: Global Chatbot UI Integration for Docusaurus Frontend

## Entity: Chat Message

### Description
Represents a single message in the conversation, including text content, sender type (user/bot), and timestamp.

### Fields
- `id`: Unique identifier for the message (string/UUID)
- `text`: The content of the message (string, required, 1-4000 characters)
- `sender`: The type of sender (enum: "user" | "bot" | "system", required)
- `timestamp`: The time when the message was created/sent (ISO 8601 datetime string, required)
- `sources` (optional): Array of source references for bot responses (string[], max 10 items)
- `confidence` (optional): Confidence level for bot responses (number between 0-1, nullable)

### Relationships
- Belongs to a Chat Session (via conversationId)

### Validation Rules
- Text must be between 1 and 4000 characters
- Sender must be one of the allowed enum values
- Timestamp must be in valid ISO 8601 format
- Sources array must not exceed 10 items
- Confidence must be between 0 and 1 if provided

## Entity: Chat Session

### Description
Represents a conversation context that may persist across page navigations.

### Fields
- `id`: Unique identifier for the session (string/UUID, required)
- `createdAt`: The time when the session was created (ISO 8601 datetime string, required)
- `lastActiveAt`: The time when the session was last active (ISO 8601 datetime string, required)
- `isActive`: Whether the session is currently active (boolean, default: true)

### Relationships
- Contains multiple Chat Messages (one-to-many)

### Validation Rules
- ID must be unique across all sessions
- createdAt must be before or equal to lastActiveAt
- isActive must be boolean value

## State Transitions

### Chat Session States
- `active`: Session is currently in use
- `inactive`: Session exists but not currently in use
- `expired`: Session has exceeded time limits and is no longer valid

### Transition Rules
- `created` → `active`: When user starts a new conversation
- `active` → `inactive`: When user closes the chat or navigates away
- `inactive` → `active`: When user reopens the chat
- `inactive` → `expired`: When session exceeds time-to-live threshold

## Data Flow

### Client-Side Storage
- Chat messages stored temporarily in component state
- Session ID persisted in memory during browser session
- Conversation history maintained until user clears or session expires

### Server-Side Storage
- Backend maintains conversation context using session ID
- Message history stored temporarily for continuity
- Session state managed server-side with expiration
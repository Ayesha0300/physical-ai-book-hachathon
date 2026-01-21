# Quickstart Guide: Global Chatbot UI Integration for Docusaurus Frontend

## Overview
This guide will help you set up and run the persistent chatbot UI across all pages of the Docusaurus documentation site.

## Prerequisites
- Node.js 18+ installed
- Python 3.11+ installed
- npm/yarn package manager

## Backend Setup
1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Create a virtual environment:
   ```bash
   python -m venv venv
   ```

3. Activate the virtual environment:
   - On Windows:
     ```bash
     venv\Scripts\activate
     ```
   - On macOS/Linux:
     ```bash
     source venv/bin/activate
     ```

4. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

5. Set up environment variables (copy `.env.example` to `.env` and update values):
   ```bash
   cp .env.example .env
   # Edit .env with your specific configurations
   ```

6. Start the backend server:
   ```bash
   uvicorn main:app --reload
   ```
   The backend will run on `http://localhost:8000`

## Frontend Setup
1. Install frontend dependencies:
   ```bash
   npm install
   ```

2. Start the Docusaurus development server:
   ```bash
   npm start
   ```
   The frontend will run on `http://localhost:3000`

## Integration Points
The chatbot UI is integrated via the Root component (`src/Root.jsx`) which ensures it appears on all pages. The component is injected through the Docusaurus `clientModules` configuration in `docusaurus.config.js`.

## Key Files
- `src/Root.jsx` - Global component wrapper
- `src/components/Chatbot/FloatingChatbot.jsx` - Floating UI component
- `src/components/Chatbot/Chatbot.jsx` - Main chat interface
- `backend/api.py` - Backend API endpoint
- `docusaurus.config.js` - Docusaurus configuration with client modules

## Verification
1. Visit any page on your Docusaurus site
2. Look for the floating chatbot button at the bottom right corner
3. Click the button to open the chat interface
4. Type a message and submit to test the backend connection
5. Verify that responses are received and displayed properly

## Troubleshooting
- If the chatbot button doesn't appear, ensure `Root.jsx` is properly configured in `docusaurus.config.js`
- If API calls fail, verify the backend server is running on the correct port
- Check browser console for any JavaScript errors
- Verify CORS settings if serving from different domains
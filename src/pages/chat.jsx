import React from 'react';
import Layout from '@theme/Layout';
import Chatbot from '../components/Chatbot/Chatbot';

function ChatPage() {
  return (
    <Layout title="RAG Chatbot" description="Interactive chatbot for the Physical AI & Humanoid Robotics book">
      <div style={{ padding: '20px', maxWidth: '1200px', margin: '0 auto' }}>
        <header style={{ marginBottom: '2rem' }}>
          <h1>Physical AI & Humanoid Robotics Chatbot</h1>
          <p>Ask questions about the book content and get AI-powered responses based on the material.</p>
        </header>

        <main>
          <div style={{
            border: '1px solid #1710d6ff',
            borderRadius: '8px',
            overflow: 'hidden',
            height: 'calc(100vh - 200px)',
            minHeight: '600px'
          }}>
            <Chatbot />
          </div>
        </main>
      </div>
    </Layout>
  );
}

export default ChatPage;
import React from 'react';
import Layout from '@theme/Layout';

function ChatbotPage() {
  return (
    <Layout
      title="Chatbot"
      description="Interactive AI assistant for Physical AI and Humanoid Robotics">
      <main className="container mx-auto px-4 py-8">
        <div className="max-w-4xl mx-auto">
          <h1 className="text-3xl font-bold text-light-text-50 mb-6">Physical AI Chatbot</h1>
          <div className="bg-dark-bg-800/50 backdrop-blur-lg border border-dark-bg-700/50 rounded-xl p-6">
            <p className="text-light-text-200 mb-4">
              This is the Physical AI Chatbot page where you can interact with our AI assistant
              to learn about robotics, AI, and humanoid systems.
            </p>
            <p className="text-light-text-200">
              The chatbot is currently under development and will be available soon.
            </p>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default ChatbotPage;
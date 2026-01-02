import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import clsx from 'clsx';

function CapstonePage() {
  return (
    <Layout
      title="Autonomous Humanoid Capstone"
      description="The comprehensive capstone project integrating all Physical AI concepts">
      <main className="container mx-auto px-4 py-8">
        <div className="max-w-5xl mx-auto">
          <div className="text-center mb-12">
            <h1 className="text-4xl font-bold text-light-text-50 mb-4">
              Autonomous Humanoid Capstone
            </h1>
            <p className="text-xl text-light-text-200">
              The culmination of your Physical AI journey - where all concepts integrate into a complete system
            </p>
          </div>

          <div className="bg-dark-bg-800/50 backdrop-blur-lg border border-dark-bg-700/50 rounded-xl p-8 mb-8">
            <h2 className="text-2xl font-bold text-light-text-50 mb-6">Mission Briefing</h2>

            <div className="prose prose-invert max-w-none">
              <h3 className="text-xl font-semibold text-light-text-50 mb-4">Project Objectives</h3>
              <ul className="list-disc list-inside text-light-text-200 mb-6 space-y-2">
                <li>Design an autonomous humanoid robot system capable of understanding natural language commands</li>
                <li>Navigate complex indoor environments safely with multiple users</li>
                <li>Manipulate objects with dexterity and precision</li>
                <li>Respond appropriately to dynamic environmental changes</li>
                <li>Maintain safe and effective operation over extended periods</li>
              </ul>

              <h3 className="text-xl font-semibold text-light-text-50 mb-4">System Architecture</h3>
              <div className="bg-dark-bg-700/50 rounded-lg p-6 mb-6">
                <h4 className="font-semibold text-light-text-50 mb-3">Integrated Components</h4>
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                  <div className="bg-dark-bg-600/50 rounded-lg p-4">
                    <h5 className="font-medium text-accent-primary-400 mb-2">Voice Processing</h5>
                    <p className="text-sm text-light-text-200">Convert speech to text with high accuracy in varied acoustic conditions</p>
                  </div>
                  <div className="bg-dark-bg-600/50 rounded-lg p-4">
                    <h5 className="font-medium text-accent-primary-400 mb-2">Language Understanding</h5>
                    <p className="text-sm text-light-text-200">Interpret natural language commands and extract intent from complex multi-step tasks</p>
                  </div>
                  <div className="bg-dark-bg-600/50 rounded-lg p-4">
                    <h5 className="font-medium text-accent-primary-400 mb-2">Perception System</h5>
                    <p className="text-sm text-light-text-200">Understand visual environment and identify relevant objects in real-time</p>
                  </div>
                  <div className="bg-dark-bg-600/50 rounded-lg p-4">
                    <h5 className="font-medium text-accent-primary-400 mb-2">Action Execution</h5>
                    <p className="text-sm text-light-text-200">Execute plans using ROS 2 navigation and manipulation stacks</p>
                  </div>
                </div>
              </div>

              <h3 className="text-xl font-semibold text-light-text-50 mb-4">Example Scenario</h3>
              <p className="text-light-text-200 mb-4">
                Complete a complex multi-step task: "When you see John in the kitchen, tell him that the meeting has been moved to 3 PM,
                then go to the conference room and set up the projector for the presentation."
              </p>

              <div className="bg-dark-bg-700/50 rounded-lg p-6 mb-6">
                <h4 className="font-semibold text-light-text-50 mb-3">Scenario Execution Flow</h4>
                <ol className="list-decimal list-inside text-light-text-200 space-y-2">
                  <li><strong>Conditional Monitoring:</strong> Continuously monitor kitchen area for person identification</li>
                  <li><strong>Navigation & Communication:</strong> Navigate to John and deliver meeting update</li>
                  <li><strong>Secondary Task Execution:</strong> Navigate to conference room and set up projector</li>
                  <li><strong>Verification & Completion:</strong> Confirm projector setup and return to ready state</li>
                </ol>
              </div>

              <h3 className="text-xl font-semibold text-light-text-50 mb-4">Integration Points</h3>
              <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
                <div className="bg-accent-primary-500/10 border border-accent-primary-500/30 rounded-lg p-4 text-center">
                  <div className="text-2xl mb-2">🤖</div>
                  <h4 className="font-medium text-accent-primary-400 text-sm">Perception-Planning</h4>
                  <p className="text-xs text-light-text-200 mt-1">Conditional execution based on person detection</p>
                </div>
                <div className="bg-accent-success-500/10 border border-accent-success-500/30 rounded-lg p-4 text-center">
                  <div className="text-2xl mb-2">💬</div>
                  <h4 className="font-medium text-accent-success-400 text-sm">Language-Execution</h4>
                  <p className="text-xs text-light-text-200 mt-1">Natural language to specific actions</p>
                </div>
                <div className="bg-accent-warning-500/10 border border-accent-warning-500/30 rounded-lg p-4 text-center">
                  <div className="text-2xl mb-2">🧭</div>
                  <h4 className="font-medium text-accent-warning-400 text-sm">Navigation-Manipulation</h4>
                  <p className="text-xs text-light-text-200 mt-1">Coordinated movement and manipulation</p>
                </div>
              </div>

              <h3 className="text-xl font-semibold text-light-text-50 mb-4">Evaluation Criteria</h3>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                <div>
                  <h4 className="font-medium text-light-text-50 mb-3">Functional Metrics</h4>
                  <ul className="text-sm text-light-text-200 space-y-1">
                    <li>• Command Understanding: 95% accuracy</li>
                    <li>• Task Completion: 90% success rate</li>
                    <li>• Navigation Performance: Safe & accurate</li>
                    <li>• Manipulation Success: High precision</li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-medium text-light-text-50 mb-3">Safety & Robustness</h4>
                  <ul className="text-sm text-light-text-200 space-y-1">
                    <li>• Zero safety incidents</li>
                    <li>• Failure recovery capability</li>
                    <li>• Long-term operational stability</li>
                    <li>• Environmental adaptation</li>
                  </ul>
                </div>
              </div>
            </div>
          </div>

          <div className="flex flex-col sm:flex-row justify-center gap-4">
            <Link
              className="button button--primary button--lg px-8 py-4 rounded-lg text-lg font-semibold bg-accent-primary-500 hover:bg-accent-primary-600 transition-colors"
              to="/modules">
              Explore Modules
            </Link>
            <Link
              className="button button--secondary button--lg px-8 py-4 rounded-lg text-lg font-semibold bg-dark-bg-700 hover:bg-dark-bg-600 text-light-text-50 border border-dark-bg-600 transition-colors"
              to="/docs/vla/chapter-8-capstone">
              View Full Documentation
            </Link>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default CapstonePage;
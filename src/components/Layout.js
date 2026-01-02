import React, { useState } from 'react';
import { useLocation } from '@docusaurus/router';

function Layout({ children, title, description }) {
  const location = useLocation();
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);
  const [sidebarOpen, setSidebarOpen] = useState(false);

  // Navigation items
  const navItems = [
    { label: 'Home', href: '/', active: location.pathname === '/' },
    { label: 'Modules', href: '/modules', active: location.pathname.startsWith('/modules') }
   
  ];

  const moduleItems = [
    { label: 'Module 1: ROS 2', href: '/modules/ros2' },
    { label: 'Module 2: Gazebo', href: '/modules/gazebo' },
    { label: 'Module 3: Isaac', href: '/modules/isaac' },
    { label: 'Module 4: VLA', href: '/modules/vla' }
  ];

  return (
    <div className="min-h-screen bg-dark-bg-900 text-light-text-50 flex flex-col">
      {/* Header */}
      <header className="sticky top-0 z-50 bg-dark-bg-800 border-b border-dark-bg-700">
        <div className="container mx-auto px-4">
          <div className="flex items-center justify-between h-16">
            <div className="flex items-center">
              <button
                onClick={() => setSidebarOpen(!sidebarOpen)}
                className="md:hidden mr-3 p-2 rounded-md text-light-text-200 hover:text-light-text-50 hover:bg-dark-bg-700 focus:outline-none"
              >
                <svg className="h-6 w-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 6h16M4 12h16M4 18h16" />
                </svg>
              </button>

              <a href="/" className="flex items-center">
                <div className="bg-accent-primary-500 w-8 h-8 rounded-lg flex items-center justify-center mr-3">
                  <span className="font-bold text-white text-sm">AI</span>
                </div>
                <span className="text-xl font-bold text-white">Physical AI</span>
              </a>
            </div>

            {/* Desktop Navigation */}
            <nav className="hidden md:flex space-x-8">
              {navItems.map((item, index) => (
                <a
                  key={index}
                  href={item.href}
                  className={`${
                    item.active
                      ? 'text-accent-primary-400 border-b-2 border-accent-primary-400'
                      : 'text-light-text-200 hover:text-light-text-50'
                  } px-1 py-2 text-sm font-medium transition-colors duration-200`}
                >
                  {item.label}
                </a>
              ))}
            </nav>

            <div className="flex items-center">
              <button className="p-2 rounded-full text-light-text-200 hover:text-light-text-50 hover:bg-dark-bg-700 focus:outline-none">
                <svg className="h-6 w-6" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z" />
                </svg>
              </button>
              <a
                href="https://github.com/your-username/physical-ai-book"
                className="ml-4 text-light-text-200 hover:text-light-text-50"
              >
                <svg className="h-6 w-6" fill="currentColor" viewBox="0 0 24 24">
                  <path fillRule="evenodd" clipRule="evenodd" d="M12 2C6.477 2 2 6.484 2 12.017c0 4.425 2.865 8.18 6.839 9.504.5.092.682-.217.682-.483 0-.237-.008-.868-.013-1.703-2.782.605-3.369-1.343-3.369-1.343-.454-1.158-1.11-1.466-1.11-1.466-.908-.62.069-.608.069-.608 1.003.07 1.531 1.032 1.531 1.032.892 1.53 2.341 1.088 2.91.832.092-.647.35-1.088.636-1.338-2.22-.253-4.555-1.113-4.555-4.951 0-1.093.39-1.988 1.029-2.688-.103-.253-.446-1.272.098-2.65 0 0 .84-.27 2.75 1.026A9.564 9.564 0 0112 6.844c.85.004 1.705.115 2.504.337 1.909-1.296 2.747-1.027 2.747-1.027.546 1.379.202 2.398.1 2.651.64.7 1.028 1.595 1.028 2.688 0 3.848-2.339 4.695-4.566 4.943.359.309.677.92.677 1.855 0 1.338-.012 2.419-.012 2.747 0 .268.18.58.688.482A10.019 10.019 0 0022 12.017C22 6.484 17.522 2 12 2z" />
                </svg>
              </a>
            </div>
          </div>
        </div>
      </header>

      <div className="flex flex-1">
        {/* Sidebar for desktop */}
        <aside
          className={`fixed md:relative z-40 inset-y-0 left-0 w-64 bg-dark-bg-800 border-r border-dark-bg-700 transform transition-transform duration-300 ease-in-out ${
            sidebarOpen ? 'translate-x-0' : '-translate-x-full md:translate-x-0'
          } md:translate-x-0`}
        >
          <div className="h-full overflow-y-auto pt-4">
            <nav className="px-4 space-y-2">
              <h3 className="text-xs font-semibold text-light-text-200 uppercase tracking-wider px-2">Navigation</h3>
              {navItems.map((item, index) => (
                <a
                  key={index}
                  href={item.href}
                  className={`${
                    item.active
                      ? 'bg-accent-primary-500/20 text-accent-primary-400'
                      : 'text-light-text-200 hover:bg-dark-bg-700 hover:text-light-text-50'
                  } group flex items-center px-2 py-2 text-sm font-medium rounded-md transition-colors duration-200`}
                >
                  {item.label}
                </a>
              ))}

              <h3 className="text-xs font-semibold text-light-text-200 uppercase tracking-wider px-2 mt-6">Modules</h3>
              {moduleItems.map((item, index) => (
                <a
                  key={index}
                  href={item.href}
                  className="text-light-text-200 hover:bg-dark-bg-700 hover:text-light-text-50 group flex items-center px-2 py-2 text-sm font-medium rounded-md transition-colors duration-200"
                >
                  {item.label}
                </a>
              ))}
            </nav>
          </div>
        </aside>

        {/* Overlay for mobile */}
        {sidebarOpen && (
          <div
            className="fixed inset-0 z-30 bg-black bg-opacity-50 md:hidden"
            onClick={() => setSidebarOpen(false)}
          ></div>
        )}

        {/* Main content */}
        <main className="flex-1 py-8">
          <div className="container mx-auto px-4">
            {children}
          </div>
        </main>
      </div>

      {/* Footer */}
      <footer className="bg-dark-bg-800 border-t border-dark-bg-700">
        <div className="container mx-auto px-4 py-8">
          <div className="grid grid-cols-1 md:grid-cols-4 gap-8">
            <div>
              <h3 className="text-lg font-semibold text-light-text-50 mb-4">Physical AI</h3>
              <p className="text-light-text-200 text-sm">
                Advanced robotics and AI education platform focused on embodied intelligence.
              </p>
            </div>
            <div>
              <h4 className="text-sm font-semibold text-light-text-50 uppercase tracking-wider mb-4">Modules</h4>
              <ul className="space-y-2">
                {moduleItems.map((item, index) => (
                  <li key={index}>
                    <a href={item.href} className="text-light-text-200 hover:text-accent-primary-400 text-sm transition-colors duration-200">
                      {item.label}
                    </a>
                  </li>
                ))}
              </ul>
            </div>
            <div>
              <h4 className="text-sm font-semibold text-light-text-50 uppercase tracking-wider mb-4">Resources</h4>
              <ul className="space-y-2">
                <li><a href="/docs/intro" className="text-light-text-200 hover:text-accent-primary-400 text-sm transition-colors duration-200">Documentation</a></li>
                <li><a href="/hardware" className="text-light-text-200 hover:text-accent-primary-400 text-sm transition-colors duration-200">Hardware Setup</a></li>
                <li><a href="https://github.com/your-username/physical-ai-book" className="text-light-text-200 hover:text-accent-primary-400 text-sm transition-colors duration-200">GitHub</a></li>
              </ul>
            </div>
            <div>
              <h4 className="text-sm font-semibold text-light-text-50 uppercase tracking-wider mb-4">Legal</h4>
              <ul className="space-y-2">
                <li><a href="#" className="text-light-text-200 hover:text-accent-primary-400 text-sm transition-colors duration-200">License</a></li>
                <li><a href="#" className="text-light-text-200 hover:text-accent-primary-400 text-sm transition-colors duration-200">Privacy Policy</a></li>
                <li><a href="#" className="text-light-text-200 hover:text-accent-primary-400 text-sm transition-colors duration-200">Credits</a></li>
              </ul>
            </div>
          </div>
          <div className="border-t border-dark-bg-700 mt-8 pt-8 text-center">
            <p className="text-light-text-200 text-sm">
              Copyright © {new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.
            </p>
          </div>
        </div>
      </footer>
    </div>
  );
}

export default Layout;
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './modules.module.css';

// Import the modules data
import modulesData from '../data/modules.json';

function ModulesPage() {
  const { siteConfig } = useDocusaurusContext();
  const [selectedCategory, setSelectedCategory] = useState('all');
  const [searchTerm, setSearchTerm] = useState('');

  // Filter modules based on category and search term
  const filteredModules = modulesData.filter(module => {
    const matchesCategory = selectedCategory === 'all' ||
      module.difficulty.toLowerCase() === selectedCategory.toLowerCase();
    const matchesSearch = module.title.toLowerCase().includes(searchTerm.toLowerCase()) ||
      module.description.toLowerCase().includes(searchTerm.toLowerCase()) ||
      module.tools.some(tool => tool.toLowerCase().includes(searchTerm.toLowerCase()));
    return matchesCategory && matchesSearch;
  });

  const categories = ['all', 'beginner', 'intermediate', 'advanced'];

  return (
    <Layout
      title={`Modules | ${siteConfig.title}`}
      description="Browse all learning modules for Physical AI & Humanoid Robotics">
      <main className="modules-page">
        <div className="container">
          {/* Hero Section */}
          <section className="modules-hero">
            <div className="modules-hero-content">
              <h1 className="modules-title">Learning Modules</h1>
              <p className="modules-subtitle">
                Explore our comprehensive curriculum on Physical AI and Humanoid Robotics
              </p>
              <div className="modules-stats">
                <div className="stat-item">
                  <span className="stat-number">{modulesData.length}</span>
                  <span className="stat-label">Modules</span>
                </div>
                <div className="stat-item">
                  <span className="stat-number">{new Set(modulesData.flatMap(m => m.tools)).size}</span>
                  <span className="stat-label">Technologies</span>
                </div>
                <div className="stat-item">
                  <span className="stat-number">{categories.slice(1).length}</span>
                  <span className="stat-label">Skill Levels</span>
                </div>
              </div>
            </div>
          </section>

          {/* Filters Section */}
          <section className="modules-filters">
            <div className="filters-container">
              <div className="search-box">
                <input
                  type="text"
                  placeholder="Search modules..."
                  value={searchTerm}
                  onChange={(e) => setSearchTerm(e.target.value)}
                  className="filter-input"
                />
                <svg
                  className="search-icon"
                  width="20"
                  height="20"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  strokeWidth="2"
                >
                  <circle cx="11" cy="11" r="8"></circle>
                  <path d="m21 21-4.35-4.35"></path>
                </svg>
              </div>

              <div className="category-filters">
                {categories.map(category => (
                  <button
                    key={category}
                    className={`filter-btn ${selectedCategory === category ? 'filter-btn--active' : ''}`}
                    onClick={() => setSelectedCategory(category)}
                  >
                    {category.charAt(0).toUpperCase() + category.slice(1)}
                  </button>
                ))}
              </div>
            </div>
          </section>

          {/* Modules Grid */}
          <section className="modules-grid-section">
            <div className="modules-grid-header">
              <h2 className="modules-grid-title">
                {selectedCategory === 'all' ? 'All Modules' : `${selectedCategory.charAt(0).toUpperCase() + selectedCategory.slice(1)} Modules`}
                <span className="modules-count">({filteredModules.length})</span>
              </h2>
            </div>

            {filteredModules.length > 0 ? (
              <div className="module-grid">
                {filteredModules.map((module, index) => (
                  <ModuleCard key={`${module.id}-${index}`} module={module} />
                ))}
              </div>
            ) : (
              <div className="modules-empty-state">
                <div className="empty-state-content">
                  <svg
                    className="empty-state-icon"
                    width="64"
                    height="64"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    strokeWidth="2"
                  >
                    <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2"></path>
                    <circle cx="12" cy="7" r="4"></circle>
                  </svg>
                  <h3 className="empty-state-title">No modules found</h3>
                  <p className="empty-state-description">
                    Try adjusting your search or filter criteria
                  </p>
                  <button
                    className="empty-state-btn"
                    onClick={() => {
                      setSearchTerm('');
                      setSelectedCategory('all');
                    }}
                  >
                    Clear filters
                  </button>
                </div>
              </div>
            )}
          </section>
        </div>
      </main>
    </Layout>
  );
}

// Module Card Component
const ModuleCard = ({ module }) => {
  const getDifficultyColor = (difficulty) => {
    switch (difficulty.toLowerCase()) {
      case 'beginner':
        return 'beginner';
      case 'intermediate':
        return 'intermediate';
      case 'advanced':
        return 'advanced';
      default:
        return 'beginner';
    }
  };

  return (
    <div className="module-card">
      <div className="module-card__header">
        <div className="module-card__icon">
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"></path>
            <polyline points="14 2 14 8 20 8"></polyline>
            <line x1="16" y1="13" x2="8" y2="13"></line>
            <line x1="16" y1="17" x2="8" y2="17"></line>
            <polyline points="10 9 9 9 8 9"></polyline>
          </svg>
        </div>
        <h3 className="module-card__title">{module.title}</h3>
        <span className={`module-card__badge module-card__badge--${getDifficultyColor(module.difficulty)}`}>
          {module.difficulty}
        </span>
      </div>

      <p className="module-card__description">{module.description}</p>

      <div className="module-card__meta">
        <div className="module-card__tools">
          <strong>Technologies:</strong>
          <div className="module-card__tools-list">
            {module.tools.slice(0, 4).map((tool, index) => (
              <span key={index} className="module-card__tool-badge">
                {tool}
              </span>
            ))}
            {module.tools.length > 4 && (
              <span className="module-card__tool-badge module-card__tool-badge--more">
                +{module.tools.length - 4} more
              </span>
            )}
          </div>
        </div>

        <div className="module-card__lessons">
          <span className="lesson-count">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"></path>
              <polyline points="14 2 14 8 20 8"></polyline>
              <line x1="16" y1="13" x2="8" y2="13"></line>
              <line x1="16" y1="17" x2="8" y2="17"></line>
              <polyline points="10 9 9 9 8 9"></polyline>
            </svg>
            {Math.floor(Math.random() * 8) + 5} lessons
          </span>
          <span className="estimated-time">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <circle cx="12" cy="12" r="10"></circle>
              <polyline points="12 6 12 12 16 14"></polyline>
            </svg>
            {Math.floor(Math.random() * 10) + 8}h
          </span>
        </div>
      </div>

      <a
        className="module-card__cta"
        href={module.route}
        onClick={(e) => {
          if (!module.route.startsWith('/')) {
            e.preventDefault();
            window.open(module.route, '_blank');
          }
        }}
      >
        Start Learning
        <svg
          className="module-card__cta-icon"
          width="16"
          height="16"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
        >
          <path strokeLinecap="round" strokeLinejoin="round" d="M17.25 8.25L21 12m0 0l-3.75 3.75M21 12H3" />
        </svg>
      </a>
    </div>
  );
};

export default ModulesPage;
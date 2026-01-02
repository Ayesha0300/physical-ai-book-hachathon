import React from 'react';
import Layout from '@theme/Layout';
import ModuleGrid from '@site/src/components/Modules/ModuleGrid';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './modules.module.css';

// Import the modules data
import modulesData from '../data/modules.json';

function ModulesPage() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Modules | ${siteConfig.title}`}
      description="Browse all learning modules for Physical AI & Humanoid Robotics">
      <main className="modules-page">
        <div className="container">
          <div className="modules-header">
            <h1 className="modules-title">Learning Modules</h1>
            <p className="modules-subtitle">
              Explore our comprehensive curriculum on Physical AI and Humanoid Robotics
            </p>
          </div>

          <ModuleGrid modules={modulesData} />
        </div>
      </main>
    </Layout>
  );
}

export default ModulesPage;
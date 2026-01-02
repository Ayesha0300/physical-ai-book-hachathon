import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HeroSection from '@site/src/components/Homepage/HeroSection';
import CurriculumOverview from '@site/src/components/Homepage/CurriculumOverview';
import LearningPath from '@site/src/components/Homepage/LearningPath';
import VisualDiagram from '@site/src/components/Homepage/VisualDiagram';
import styles from './index.module.css';


function FeatureCard({ icon, title, description }) {
  return (
    <div className={styles.featureCard} role="article" aria-labelledby={`feature-title-${title.replace(/\s+/g, '-').toLowerCase()}`}>
      <div className={styles.featureIcon} aria-hidden="true">{icon}</div>
      <h3 id={`feature-title-${title.replace(/\s+/g, '-').toLowerCase()}`} className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
}

function FeaturesSection() {
  const features = [
    {
      icon: '🎯',
      title: 'Comprehensive Curriculum',
      description: 'From basic ROS 2 concepts to advanced Vision-Language-Action models, covering the complete physical AI stack.'
    },
    {
      icon: '💻',
      title: 'Hands-On Projects',
      description: 'Build real robots with practical exercises, simulations, and a capstone humanoid project.'
    },
    {
      icon: '🚀',
      title: 'Modern Technologies',
      description: 'Learn ROS 2, Gazebo, NVIDIA Isaac Sim, Isaac ROS, and cutting-edge VLA models.'
    },
    {
      icon: '📊',
      title: 'Industry-Relevant',
      description: 'Skills directly applicable to robotics companies, research labs, and AI startups.'
    },
    {
      icon: '🧩',
      title: 'Modular Learning',
      description: 'Progress at your own pace with structured modules building on each other.'
    },
    {
      icon: '🌐',
      title: 'Real-World Focus',
      description: 'Bridge the sim-to-real gap with practical strategies for deploying robots in production.'
    }
  ];

  return (
    <section className={styles.featuresSection} aria-labelledby="features-section-title">
      <div className={styles.container}>
        <div className={styles.sectionHeader}>
          <h2 id="features-section-title" className={styles.sectionTitle}>Why This Book?</h2>
          <p className={styles.sectionSubtitle}>
            A complete learning path designed for the next generation of robotics engineers
          </p>
        </div>
        <div className={styles.featuresGrid} role="list">
          {features.map((feature, idx) => (
            <div key={idx} role="listitem">
              <FeatureCard {...feature} />
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function ModuleCard({ number, title, description, technologies, route, difficulty }) {
  return (
    <div className={styles.moduleCard} role="article" aria-labelledby={`module-title-${title.replace(/\s+/g, '-').toLowerCase()}`}>
      <div className={styles.moduleHeader}>
        <div className={styles.moduleNumber} aria-label={`Module number ${number}`}>{number}</div>
        <div className={styles.moduleDifficulty} aria-label={`Difficulty: ${difficulty}`}>{difficulty}</div>
      </div>
      <h3 id={`module-title-${title.replace(/\s+/g, '-').toLowerCase()}`} className={styles.moduleTitle}>{title}</h3>
      <p className={styles.moduleDescription}>{description}</p>
      <div className={styles.moduleTech} aria-label="Technologies used">
        {technologies.map((tech, idx) => (
          <span key={idx} className={styles.techBadge} aria-label={`Technology: ${tech}`}>{tech}</span>
        ))}
      </div>
      <Link to={route} className={styles.moduleLink} aria-label={`Explore ${title} module`}>
        Explore Module
        <svg style={{ width: '1rem', height: '1rem', marginLeft: '0.375rem' }} fill="none" stroke="currentColor" viewBox="0 0 24 24" aria-hidden="true">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
        </svg>
      </Link>
    </div>
  );
}

function ModulesSection() {
  const modules = [
    {
      number: '01',
      title: 'ROS 2: The Nervous System',
      description: 'Master the foundation of modern robotics with ROS 2 architecture, nodes, topics, and Python integration.',
      technologies: ['ROS 2', 'Python', 'URDF'],
      route: '/docs/module1-ros2/chapter1-why-robots-need-nervous-system',
      difficulty: 'Beginner'
    },
    {
      number: '02',
      title: 'Digital Twin Simulation',
      description: 'Create realistic robot simulations with Gazebo and Unity for safe, cost-effective development.',
      technologies: ['Gazebo', 'Unity', 'Sensors'],
      route: '/docs/module2-digital-twin/chapter1-intro',
      difficulty: 'Intermediate'
    },
    {
      number: '03',
      title: 'AI-Robot Brain (Isaac)',
      description: 'Leverage NVIDIA Isaac for photorealistic simulation, perception, and hardware-accelerated AI.',
      technologies: ['Isaac Sim', 'Isaac ROS', 'Nav2'],
      route: '/docs/module3-isaac/chapter1-from-middleware-to-intelligence',
      difficulty: 'Intermediate'
    },
    {
      number: '04',
      title: 'Vision-Language-Action',
      description: 'Build intelligent robots that understand natural language and translate it into physical actions.',
      technologies: ['VLA', 'LLMs', 'Computer Vision'],
      route: '/docs/vla/chapter-1-why-vla-matters',
      difficulty: 'Advanced'
    }
  ];

  return (
    <section className={styles.modulesSection} aria-labelledby="modules-section-title">
      <div className={styles.container}>
        <div className={styles.sectionHeader}>
          <h2 id="modules-section-title" className={styles.sectionTitle}>Learning Modules</h2>
          <p className={styles.sectionSubtitle}>
            Four comprehensive modules taking you from beginner to advanced physical AI developer
          </p>
        </div>
        <div className={styles.modulesGrid} role="list">
          {modules.map((module, idx) => (
            <div key={idx} role="listitem">
              <ModuleCard {...module} />
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.ctaSection} aria-labelledby="cta-section-title">
      <div className={styles.container}>
        <div className={styles.ctaContent}>
          <h2 id="cta-section-title" className={styles.ctaTitle}>Ready to Build the Future?</h2>
          <p className={styles.ctaDescription}>
            Join the physical AI revolution. Start learning today and build robots that can see, understand, and act.
          </p>
          <div className={styles.ctaButtons} role="group" aria-labelledby="cta-section-title">
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Get Started Now
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/hardware">
              View Hardware Setup
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description={siteConfig.tagline}>
      <main>
        <HeroSection />
        <CurriculumOverview />
        <LearningPath />
        <VisualDiagram />
        <FeaturesSection />
        <ModulesSection />
        <CTASection />
      </main>
    </Layout>
  );
}

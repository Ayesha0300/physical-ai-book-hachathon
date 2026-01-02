import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './HeroSection.module.css';

const HeroSection = () => {
  const { siteConfig } = useDocusaurusContext();

  return (
    <section className="hero hero--primary" aria-labelledby="hero-title">
      <div className="container">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1 id="hero-title" className="hero__title">
              {siteConfig.title}
            </h1>
            <p className="hero__subtitle" id="hero-description">
              {siteConfig.tagline}
            </p>
            <p className="hero__subtitle">
              Bridging the Digital Brain with the Physical Body
            </p>
            <div className="hero__buttons" role="group" aria-labelledby="hero-title">
              <Link
                className="button button--secondary button--lg"
                to="/modules"
                aria-label="Explore learning modules">
                Explore Modules
              </Link>
              <Link
                className="button button--primary button--lg margin-left--sm"
                to="/capstone"
                aria-label="View capstone project">
                View Capstone
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default HeroSection;
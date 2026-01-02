import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

function NotFound() {
  return (
    <Layout title="Page Not Found" description="The page you're looking for doesn't exist">
      <main className="container margin-vert--xl">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="text--center padding-vert--xl">
              <h1 className="hero__title">404</h1>
              <p className="hero__subtitle">Page Not Found</p>
              <p className="margin-bottom--lg">
                We couldn't find the page you're looking for. This might be because:
              </p>
              <ul className="margin-bottom--lg">
                <li>The page has moved</li>
                <li>The page was renamed</li>
                <li>The URL was typed incorrectly</li>
                <li>The page no longer exists</li>
              </ul>

              <div className="button-group button-group--block margin-top--lg">
                <Link className="button button--primary button--lg" to="/">
                  Go Home
                </Link>
                <Link className="button button--secondary button--lg margin-left--sm" to="/modules">
                  View Modules
                </Link>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default NotFound;
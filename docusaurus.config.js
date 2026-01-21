// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive guide to robotics, AI, and humanoid systems',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://physical-ai-book-umber-seven.vercel.app',
  // Set the /<base>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-organization', // Usually your GitHub org/user name.
  projectName: 'physical-ai-book', // Usually your repo name.
  trailingSlash: false,
  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
          routeBasePath: 'docs', // Changed from '/' to 'docs' to avoid duplicate routes
        },
        blog: false, // Disable blog for now
        theme: {
          customCss: './src/css/custom.css',
        },
        gtag: {
          trackingID: 'G-XXXXXXXXXX',
          anonymizeIP: true,
        },
      }),
    ],
  ],


  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      metadata: [
        {name: 'keywords', content: 'robotics, AI, physical AI, humanoid robotics, ROS 2, simulation, VLA, NVIDIA Isaac, computer vision'},
        {name: 'author', content: 'Physical AI Book Team'},
        {name: 'og:type', content: 'website'},
        {name: 'og:site_name', content: 'Physical AI & Humanoid Robotics'},
        {name: 'twitter:card', content: 'summary_large_image'},
        {name: 'twitter:site', content: '@physicalai'},
      ],
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        hideOnScroll: true,
        items: [
          {
            to: '/',
            label: 'Home',
            position: 'left',
            exact: true,
            className: 'navbar-home-link',
          },
          {
            to: '/modules',
            label: 'Modules',
            position: 'left',
            className: 'navbar-modules-link',
          },
          {
            type: 'dropdown',
            label: 'Learning Paths',
            position: 'left',
            items: [
              {
                type: 'doc',
                label: 'ROS 2 Control Systems',
                docId: 'module1-ros2/chapter1-why-robots-need-nervous-system',
              },
              {
                type: 'doc',
                label: 'Simulation with Gazebo & Isaac',
                docId: 'module2-digital-twin/chapter1-intro',
              },
              {
                type: 'doc',
                label: 'Vision-Language-Action (VLA)',
                docId: 'vla/vla-fundamentals',
              },
              {
                type: 'doc',
                label: 'Autonomous Humanoid Capstone',
                docId: 'capstone/README',
              },
            ],
            className: 'navbar-learning-paths-dropdown',
          },
          {
            to: '/docs/capstone/README',
            label: 'Capstone',
            position: 'left',
            className: 'navbar-capstone-link',
          },
          {
            to: '/docs/hardware',
            label: 'Hardware',
            position: 'left',
            className: 'navbar-hardware-link',
          },
          {
            type: 'search',
            position: 'right',
            className: 'navbar-search-item',
          },
          {
            href: 'https://github.com/your-username/physical-ai-book',
            label: 'GitHub',
            position: 'right',
            className: 'navbar-github-link',
          },
          {
            type: 'docsVersionDropdown',
            position: 'right',
            className: 'navbar-version-dropdown',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learning Paths',
            items: [
              {
                label: 'All Modules',
                to: '/modules',
              },
              {
                label: 'ROS 2 Control Systems',
                to: '/docs/module1-ros2/chapter1-why-robots-need-nervous-system',
              },
              {
                label: 'Simulation with Gazebo & Isaac',
                to: '/docs/module2-digital-twin/chapter1-intro',
              },
              {
                label: 'Vision-Language-Action (VLA)',
                to: '/docs/vla/vla-fundamentals',
              },
              {
                label: 'Autonomous Humanoid Capstone',
                to: '/docs/capstone/README',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Getting Started',
                to: '/docs/intro',
              },
              {
                label: 'Hardware Guide',
                to: '/docs/hardware',
              },
              {
                label: 'Glossary',
                to: '/docs/glossary',
              },
              {
                label: 'API Reference',
                to: '/docs/api-reference',
              },
              {
                label: 'Troubleshooting',
                to: '/docs/troubleshooting',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub Repository',
                href: 'https://github.com/your-username/physical-ai-book',
              },
              {
                label: 'Discussions',
                href: 'https://github.com/your-username/physical-ai-book/discussions',
              },
              {
                label: 'Contributing Guide',
                href: 'https://github.com/your-username/physical-ai-book/blob/main/CONTRIBUTING.md',
              },
              {
                label: 'License',
                href: 'https://creativecommons.org/licenses/by/4.0/',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Blog',
                to: '/blog',
              },
              {
                label: 'Documentation',
                to: '/docs/intro',
              },
              {
                label: 'Support',
                href: 'mailto:support@example.com',
              },
              {
                label: 'Status',
                href: 'https://status.example.com',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
    }),

  // Add custom head tags for typography fonts - Optimized
  stylesheets: [
    // Add Inter font from Google Fonts - Optimized weights only
    {
      href: 'https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800;900&display=swap',
      type: 'text/css',
      rel: 'stylesheet',
    },
  ],

  // Preconnect to Google Fonts for better performance
  headTags: [
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.googleapis.com',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.gstatic.com',
        crossorigin: 'anonymous',
      },
    },
  ],

  // Add custom wrapper to include floating chatbot on all pages
  clientModules: [
    require.resolve('./src/Root.jsx'),
  ],
};

export default config;

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
  url: 'https://physical-ai-book-hachathon.vercel.app',
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
        title: 'Physical AI Book',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            to: '/',
            label: 'Home',
            position: 'left',
            exact: true,
          },
          {
            to: '/modules',
            label: 'Modules',
            position: 'left',
          },
          {
            to: '/docs/capstone',
            label: 'Capstone',
            position: 'left',
          },
          {
            to: '/docs/hardware',
            label: 'Hardware',
            position: 'left',
          },
          {
            href: 'https://github.com/your-username/physical-ai-book',
            label: 'GitHub',
            position: 'right',
            className: 'navbar-github-link',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'All Modules',
                to: '/modules',
              },
              {
                label: 'Module 1: ROS 2',
                to: '/docs/module1-ros2/chapter1-why-robots-need-nervous-system',
              },
              {
                label: 'Module 2: Digital Twin',
                to: '/docs/module2-digital-twin/chapter1-intro',
              },
              {
                label: 'Module 3: Isaac Sim',
                to: '/docs/module3-isaac/chapter1-from-middleware-to-intelligence',
              },
              {
                label: 'Module 4: VLA',
                to: '/docs/vla/vla-fundamentals',
              },
              {
                label: 'Capstone Project',
                to: '/docs/capstone',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Get Started',
                to: '/docs/intro',
              },
              {
                label: 'Hardware Setup',
                to: '/docs/hardware',
              },
              {
                label: 'Glossary',
                to: '/docs/glossary',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/your-username/physical-ai-book',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'License',
                href: 'https://creativecommons.org/licenses/by/4.0/',
              },
              {
                label: 'Contribute',
                href: 'https://github.com/your-username/physical-ai-book/blob/main/CONTRIBUTING.md',
              },
              {
                label: 'Discussions',
                href: 'https://github.com/your-username/physical-ai-book/discussions',
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
        respectPrefersColorScheme: false,
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
};

export default config;

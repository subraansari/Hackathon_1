// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'ROS 2 Nervous System for Humanoid Robots',
  tagline: 'Educational Modules for Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://humanoid-robotics-education.github.io', // Replace with your actual URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'humanoid-robotics-education', // Replace with your GitHub org/user name.
  projectName: 'ros2-humanoid-curriculum', // Replace with your repo name.

  onBrokenLinks: 'throw',

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
            'https://github.com/humanoid-robotics-education/ros2-humanoid-curriculum/edit/main/my-website/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/humanoid-robotics-education/ros2-humanoid-curriculum/edit/main/my-website/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
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
      image: 'img/docusaurus.png', // Using existing project image
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'ROS 2 Nervous System',
        logo: {
          alt: 'ROS 2 Nervous System Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Modules',
          },
          {to: '/blog', label: 'Resources', position: 'left'},
          {
            href: 'https://github.com/humanoid-robotics-education/ros2-humanoid-curriculum',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Curriculum',
            items: [
              {
                label: 'Module 1: ROS 2 Basics',
                to: '/docs/module-1',
              },
              {
                label: 'Module 2: Digital Twins',
                to: '/docs/digital-twin',
              },
              {
                label: 'Module 3: AI Brain',
                to: '/docs/isaac-ai-brain',
              },
              {
                label: 'Module 4: Vision-Language-Action (VLA)',
                to: '/docs/module-4',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'ROS Answers',
                href: 'https://answers.ros.org/',
              },
              {
                label: 'Robotics Stack Exchange',
                href: 'https://robotics.stackexchange.com/',
              },
              {
                label: 'Discord',
                href: 'https://discord.gg/robotics',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Blog',
                to: '/blog',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/humanoid-robotics-education/ros2-humanoid-curriculum',
              },
              {
                label: 'Tutorials',
                to: '/docs/intro',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} ROS 2 Nervous System for Humanoid Robots. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;

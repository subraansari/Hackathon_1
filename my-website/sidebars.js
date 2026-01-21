// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2 Fundamentals',
      items: [
        'ros2-fundamentals/index',
        'ros2-fundamentals/nodes-topics-services-actions',
        'ros2-fundamentals/ros2-architecture'
      ],
    },
    {
      type: 'category',
      label: 'Python Agents',
      items: [
        'python-agents/index',
        'python-agents/writing-ros2-nodes-python',
        'python-agents/connecting-ai-logic-controllers'
      ],
    },
    {
      type: 'category',
      label: 'Humanoid Modeling',
      items: [
        'humanoid-modeling/index',
        'humanoid-modeling/urdf-links-joints-sensors',
        'humanoid-modeling/structure-humanoid-robots'
      ],
    },
    {
      type: 'category',
      label: 'Module 1',
      items: ['tutorial-basics/create-a-document'],
    },
    {
      type: 'category',
      label: 'Module 2',
      items: [
        'digital-twin/index',
        'digital-twin/physics-simulation-with-gazebo',
        'digital-twin/sensor-simulation',
        'digital-twin/high-fidelity-environments-with-unity'
      ],
    },
    {
      type: 'category',
      label: 'Module 3',
      items: [
        'isaac-ai-brain/index',
        'isaac-ai-brain/isaac-sim-fundamentals',
        'isaac-ai-brain/isaac-ros-perception-localization',
        'isaac-ai-brain/navigation-with-nav2'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'isaac-ai-brain/voice-to-action-pipelines',
        'isaac-ai-brain/llm-cognitive-planning',
        'isaac-ai-brain/capstone-autonomous-humanoid'
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Module 1',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;

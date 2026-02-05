# Quickstart Guide: Digital Twin Educational Module

## Overview
This guide will help you set up and start working with the Digital Twin Educational Module built with Docusaurus. This module teaches digital twin technology for humanoid robots using Gazebo for physics simulation and Unity for high-fidelity visualization.

## Prerequisites
- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control
- Basic understanding of command line tools
- Familiarity with ROS 2 concepts (covered in Module 1)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Install Dependencies
```bash
npm install
# OR if using yarn
yarn install
```

### 3. Navigate to Documentation Directory
```bash
cd my-website
```

### 4. Add Digital Twin Content
The digital twin content is organized in the docs/digital-twin/ directory:

```
/docs/digital-twin/
├── index.md
├── physics-simulation-with-gazebo.md
├── sensor-simulation.md
└── high-fidelity-environments-with-unity.md
```

### 5. Configure Sidebar Navigation
Update `sidebars.js` to include the digital twin module:

```javascript
module.exports = {
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
  ],
};
```

### 6. Start Development Server
```bash
npm run start
# OR if using yarn
yarn start
```

### 7. Build for Production
```bash
npm run build
# OR if using yarn
yarn build
```

## Adding New Digital Twin Content

### Gazebo Simulation Content
To add new Gazebo simulation content:
1. Create a new Markdown file in the `docs/digital-twin/` directory
2. Add the file path to `sidebars.js`
3. Ensure the content follows the documentation contract
4. Test locally before committing

### Unity Integration Content
For Unity-specific content:
1. Focus on Unity-ROS integration patterns
2. Include C# code examples for Unity Robotics Package
3. Cover best practices for Unity simulation environments
4. Document performance optimization techniques

### Sensor Simulation Content
For sensor simulation topics:
1. Cover various sensor types (LiDAR, cameras, IMUs, etc.)
2. Include noise modeling and accuracy considerations
3. Discuss sensor fusion techniques
4. Provide ROS 2 integration examples

## Working with Gazebo and Unity

### Gazebo Environment Setup
1. Install Gazebo Garden/Harmonic
2. Set up ROS 2 Humble Hawksbill
3. Configure Gazebo ROS packages
4. Test basic simulation with sample robots

### Unity Development Environment
1. Install Unity Hub and Unity 2022.3 LTS
2. Import Unity Robotics Package
3. Install ROS-TCP-Connector
4. Configure network settings for ROS communication

## Deployment
The site can be deployed to GitHub Pages by pushing to the appropriate branch or using the deployment script.

## Troubleshooting
- If you encounter issues with the development server, try clearing the cache: `npm run clear`
- For build issues, check that all Markdown files have proper frontmatter
- Verify that all internal links use the correct Docusaurus linking syntax
- Ensure all code examples are properly formatted with language identifiers

## Next Steps
After completing this quickstart:
1. Explore the physics simulation content in Module 2
2. Set up your local Gazebo environment
3. Experiment with the Unity integration examples
4. Try the hands-on exercises in each section
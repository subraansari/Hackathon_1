# Quickstart Guide: Isaac AI Robot Brain Educational Module

## Overview
This guide will help you set up and start working with the Isaac AI Robot Brain Educational Module built with Docusaurus. This module teaches AI-robot brain concepts using NVIDIA Isaac, covering Isaac Sim fundamentals, Isaac ROS perception and localization, and navigation with Nav2 for humanoid robots.

## Prerequisites
- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control
- Basic understanding of command line tools
- Familiarity with ROS 2 and Isaac concepts (covered in previous modules)
- Access to NVIDIA Isaac Sim (with GPU support for accelerated rendering)

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

### 4. Add Isaac AI Brain Content
The Isaac AI Brain content is organized in the docs/isaac-ai-brain/ directory:

```
/docs/isaac-ai-brain/
├── index.md
├── isaac-sim-fundamentals.md
├── isaac-ros-perception-localization.md
└── navigation-with-nav2.md
```

### 5. Configure Sidebar Navigation
Update `sidebars.js` to include the Isaac AI Brain module:

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
      label: 'Digital Twin',
      items: [
        'digital-twin/index',
        'digital-twin/physics-simulation-with-gazebo',
        'digital-twin/sensor-simulation',
        'digital-twin/high-fidelity-environments-with-unity'
      ],
    },
    {
      type: 'category',
      label: 'Isaac AI Brain',
      items: [
        'isaac-ai-brain/index',
        'isaac-ai-brain/isaac-sim-fundamentals',
        'isaac-ai-brain/isaac-ros-perception-localization',
        'isaac-ai-brain/navigation-with-nav2'
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

## Adding New Isaac AI Content

### Isaac Sim Content
To add new Isaac Sim content:
1. Create a new Markdown file in the `docs/isaac-ai-brain/` directory
2. Add the file path to `sidebars.js`
3. Ensure the content follows the documentation contract
4. Test locally before committing

### Isaac ROS Integration Content
For Isaac ROS-specific content:
1. Focus on Isaac ROS perception and localization patterns
2. Include Python code examples for Isaac ROS nodes
3. Cover best practices for VSLAM and sensor pipelines
4. Document performance optimization techniques

### Nav2 Navigation Content
For navigation topics:
1. Cover Nav2 configuration for humanoid robots
2. Include path planning algorithms and their characteristics
3. Discuss humanoid kinematic constraints
4. Provide ROS 2 integration examples

## Working with Isaac Sim and Isaac ROS

### Isaac Sim Environment Setup
1. Install Isaac Sim from NVIDIA Developer website
2. Set up Isaac ROS packages
3. Configure GPU acceleration (CUDA)
4. Test basic simulation with sample scenes

### Isaac ROS Development Environment
1. Install Isaac ROS packages
2. Set up VSLAM pipeline
3. Configure sensor processing nodes
4. Test with Isaac Sim environments

## Deployment
The site can be deployed to GitHub Pages by pushing to the appropriate branch or using the deployment script.

## Troubleshooting
- If you encounter issues with the development server, try clearing the cache: `npm run clear`
- For build issues, check that all Markdown files have proper frontmatter
- Verify that all internal links use the correct Docusaurus linking syntax
- Ensure all code examples are properly formatted with language identifiers

## Next Steps
After completing this quickstart:
1. Explore the Isaac Sim fundamentals content
2. Set up your local Isaac Sim environment
3. Experiment with the Isaac ROS perception examples
4. Try the navigation exercises with Nav2
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const ModuleCards = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description: 'Learn the fundamentals of ROS 2, nodes, topics, services, and connecting AI logic with controllers.',
    to: '/docs/module-1/',
    color: '#1a73e8',
  },
  {
    title: 'Module 2: Digital Twin',
    description: 'Explore physics simulation with Gazebo, sensor simulation, and high-fidelity environments with Unity.',
    to: '/docs/digital-twin/',
    color: '#0d8050',
  },
  {
    title: 'Module 3: Isaac AI Brain',
    description: 'Master Isaac Sim fundamentals, perception localization, navigation with Nav2, and voice-to-action pipelines.',
    to: '/docs/isaac-ai-brain/',
    color: '#ff6d01',
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: 'Integrate multimodal AI models for advanced cognitive capabilities and autonomous humanoid behaviors.',
    to: '/docs/module-4/',
    color: '#aa00ff',
  },
];

function ModuleCard({ title, description, to, color }) {
  return (
    <div className={clsx('col col--6 margin-bottom--lg')}>
      <Link
        to={to}
        className={styles.moduleCard}
        style={{ borderColor: color }}
      >
        <div className={styles.cardHeader} style={{ backgroundColor: color }}>
          <h3 className={styles.cardTitle}>{title}</h3>
        </div>
        <div className={styles.cardBody}>
          <p className={styles.cardDescription}>{description}</p>
          <div className={styles.cardFooter}>
            <span className={styles.learnMore}>Start Learning →</span>
          </div>
        </div>
      </Link>
    </div>
  );
}

export default function ModuleCardsSection() {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className="row">
          {ModuleCards.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
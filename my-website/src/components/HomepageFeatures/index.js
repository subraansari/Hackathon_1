import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Complete Robotics Curriculum',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Four comprehensive modules covering ROS 2, Digital Twins, AI Brains, and Vision-Language-Action systems
        for humanoid robots.
      </>
    ),
  },
  {
    title: 'Hands-On Learning',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Practical exercises and projects that bridge the gap between theory and real-world
        robotics applications.
      </>
    ),
  },
  {
    title: 'Cutting-Edge Technologies',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Learn with industry-standard tools including NVIDIA Isaac, Gazebo, Unity, and
        state-of-the-art AI models.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

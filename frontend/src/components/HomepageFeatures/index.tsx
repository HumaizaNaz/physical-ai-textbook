import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Master Physical AI',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Build a strong foundation in Physical AI and embodied intelligence. Learn how AI systems sense, understand, and act in the real world using ROS 2, sensors, and humanoid control architectures.
      </>
    ),
  },
  {
    title: 'Simulate Humanoids in 3D Worlds',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Create high-fidelity digital twins using Gazebo and Unity. Simulate physics, collisions, LiDAR, depth cameras, and IMU sensors to test robots in realistic environments before deployment.
      </>
    ),
  },
  {
    title: 'Build Intelligent Robot Brains',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Integrate advanced AI using NVIDIA Isaac, VSLAM, and navigation systems. Combine Vision-Language-Action (VLA) models with ROS 2 to enable natural language commands, perception, and autonomous decision-making.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
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

export default function HomepageFeatures(): ReactNode {
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

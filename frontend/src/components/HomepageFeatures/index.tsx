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
    title: 'Master The Robotics',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Build a rock-solid foundationin modren robotics.Dive deep into the Robotic Operation System (ROS 2) , explorng its powerful arcitecture for creating modular and scalable roboti behaviors
      </>
    ),
  },
  {
    title: 'Build & Test in the Metaverse',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
       create and validate complex robotic system in high-fidelity simulations .Learn to build digital twins using Gazebo and leverage the power of NVIDIA isaac Sim for realistic physics-based testing
      </>
    ),
  },
  {
    title: 'Deploy Intelligent Brains',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Go beyond simple automation integrate cutting edge AI into your robots,exploring Vision-Language Action (VLA) models thar allow for natural language interaction and complex task execution
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

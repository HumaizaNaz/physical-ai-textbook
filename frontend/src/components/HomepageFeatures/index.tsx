import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  videoSrc: string; // Changed from Svg to videoSrc
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Master Physical AI',
    videoSrc: '/videos/Master Physical AI.mp4', // Updated video source
    description: (
      <>
        Build a strong foundation in Physical AI and embodied intelligence. Learn how AI systems sense, understand, and act in the real world using ROS 2, sensors, and humanoid control architectures.
      </>
    ),
  },
  {
    title: 'Simulate Humanoids in 3D Worlds',
    videoSrc: '/videos/Simulate Humanoids in 3D Worlds.mp4', // Updated video source
    description: (
      <>
        Create high-fidelity digital twins using Gazebo and Unity. Simulate physics, collisions, LiDAR, depth cameras, and IMU sensors to test robots in realistic environments before deployment.
      </>
    ),
  },
  {
    title: 'Build Intelligent Robot Brains',
    videoSrc: '/videos/Build Intelligent Robot Brains.mp4', // Updated video source
    description: (
      <>
        Integrate advanced AI using NVIDIA Isaac, VSLAM, and navigation systems. Combine Vision-Language-Action (VLA) models with ROS 2 to enable natural language commands, perception, and autonomous decision-making.
      </>
    ),
  },
];

function Feature({title, videoSrc, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <video className={styles.featureVideo} autoPlay loop muted playsInline>
          <source src={videoSrc} type="video/mp4" />
          Your browser does not support the video tag.
        </video>
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

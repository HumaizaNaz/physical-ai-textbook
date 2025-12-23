import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext'; // Import useDocusaurusContext

type FeatureItem = {
  title: string;
  videoSrc: string; 
  description: ReactNode;
};

const RawFeatureList: (
  Omit<FeatureItem, 'videoSrc'> & { videoFileName: string }
)[] = [
  {
    title: 'Master Physical AI',
    videoFileName: 'digital-twin.mp4',
    description: (
      <>
        Build a strong foundation in Physical AI and embodied intelligence.
  Learn how AI systems sense, understand, and act in the real world
  using ROS 2, sensors, and humanoid control architectures.
      </>
    ),
  },
  {
    title: 'Simulate Humanoids in 3D Worlds',
    videoFileName: 'vla.mp4',
    description: (
      <>
        Create high-fidelity digital twins using Gazebo and Unity.
  Simulate physics, collisions, LiDAR, depth cameras, and IMU sensors
  to test robots in realistic environments before deployment.
      </>
    ),
  },
  {
    title: 'Build Intelligent Robot Brains',
    videoFileName: 'robot-brain.mp4',
    description: (
      <>
         Integrate advanced AI using NVIDIA Isaac, VSLAM, and navigation systems.
  Combine Vision-Language-Action models with ROS 2 to enable natural
  language commands, perception, and autonomous decision-making.
      </>
    ),
  },
];

function Feature({title, videoSrc, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4', styles.featureCard)}>
      <div className="text--center">
        <video className={styles.featureVideo} autoPlay loop muted playsInline>
          <source src={videoSrc} type="video/mp4" />
        </video>
      </div>

      <div className={clsx("padding-horiz--md", styles.featureContent)}>
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}


export default function HomepageFeatures(): ReactNode {
  const {siteConfig} = useDocusaurusContext(); // Get siteConfig from context
  const baseUrl = siteConfig.baseUrl; // Extract baseUrl

  const FeatureList: FeatureItem[] = RawFeatureList.map(item => ({
    ...item,
    videoSrc: `${baseUrl}videos/${item.videoFileName}`, // Prepend baseUrl
  }));

  return (
    <section className={styles.features}>
      <div className={clsx("container", styles.featureCardsContainer)}>
        <div className={clsx("row", styles.featureCardsRow)}>
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

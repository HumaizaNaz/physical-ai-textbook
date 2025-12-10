import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import VideoCarousel from '@site/src/components/VideoCarousel'; // Import the new VideoCarousel component

import styles from './index.module.css'; // Keep existing module CSS for index

function HomepageContent() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <div className={clsx('container', styles.homepageContainer)}>
      {/* Left Column: Video Carousel */}
      <div className={styles.leftColumn}>
        <VideoCarousel />
      </div>

      {/* Right Column: Existing Heading and Paragraph */}
      <div className={styles.rightColumn}>
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">
          An open-source textbook on convergence of robotics, artificial intelligence and embodied cognition. Master ROS 2, Digital Twins and advanced Vision-language Action models to bring intelligent machines to life.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/introduction">
            Start Your Journey
          </Link>
        </div>
      </div>
    </div>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="An open-source textbook on convergence of robotics, artificial intelligence and embodied cognition.">
      <main>
        <HomepageContent /> {/* Use the new HomepageContent component */}
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
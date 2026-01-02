import type {ReactNode} from 'react';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

function CTASection(): ReactNode {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2" className={styles.ctaTitle}>
            Ready to Build Humanoid Robots?
          </Heading>
          <p className={styles.ctaSubtitle}>
            Start your journey into the future of robotics and AI
          </p>
          <div className={styles.ctaButtons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/introduction">
              Start Learning Now
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/module-1-ros2/ros2-introduction">
              Explore Modules
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default CTASection;

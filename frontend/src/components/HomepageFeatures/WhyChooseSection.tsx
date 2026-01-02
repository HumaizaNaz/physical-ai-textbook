import type {ReactNode} from 'react';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import { FaRocket, FaCode, FaBrain, FaEye, FaCog } from 'react-icons/fa';

function WhyChooseSection(): ReactNode {
  return (
    <section className={styles.whyChooseSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            <FaRocket className={styles.titleIcon} />
            Why Choose This Course?
          </Heading>
        </div>
        <div className={styles.benefitsGrid}>
          <div className={styles.benefitCard}>
            <FaCode className={styles.benefitIcon} />
            <h3>Hands-On Projects</h3>
            <p>Build real-world robots with practical coding exercises and simulations</p>
          </div>
          <div className={styles.benefitCard}>
            <FaBrain className={styles.benefitIcon} />
            <h3>AI-Powered Learning</h3>
            <p>Get instant help from our 24/7 RAG chatbot trained on course content</p>
          </div>
          <div className={styles.benefitCard}>
            <FaEye className={styles.benefitIcon} />
            <h3>Industry Standard Tools</h3>
            <p>Learn ROS2, Gazebo, Isaac Sim, and modern robotics frameworks</p>
          </div>
          <div className={styles.benefitCard}>
            <FaCog className={styles.benefitIcon} />
            <h3>Personalized Experience</h3>
            <p>Content adapted to your programming and robotics background</p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default WhyChooseSection;

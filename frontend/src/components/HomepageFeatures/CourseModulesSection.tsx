import type {ReactNode} from 'react';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import { FaCode, FaCubes, FaMicrochip, FaBrain, FaBook, FaRocket } from 'react-icons/fa'; // Added FaRocket
import Link from '@docusaurus/Link';

const CourseModules = [
  {
    title: 'Introduction',
    icon: FaBook,
    description: 'Foundations of Physical AI and embodied intelligence for humanoid robotics.',
    id: 'introduction',
  },
  {
    title: 'Phase 1: The Robotic Nervous System (ROS 2)',
    icon: FaCode,
    description: 'Learn the fundamentals of ROS 2 for robotic control and communication.',
    id: 'module-1-ros2',
  },
  {
    title: 'Phase 2: The Digital Twin (Gazebo & Unity)',
    icon: FaCubes,
    description: 'Explore robotics simulation with Gazebo and high-fidelity visualization with Unity.',
    id: 'module-2-digital-twin',
  },
  {
    title: 'Phase 3: The AI-Robot Brain (NVIDIA Isaac™)',
    icon: FaMicrochip,
    description: 'Delve into NVIDIA Isaac Sim, Isaac ROS, and AI techniques for robotics.',
    id: 'module-3-isaac',
  },
  {
    title: 'Phase 4: Vision-Language-Action (VLA)',
    icon: FaBrain,
    description: 'Integrate vision, language, and action for advanced humanoid robot control.',
    id: 'module-4-vla',
  },
  {
    title: 'Capstone Project: The Autonomous Humanoid',
    icon: FaRocket,
    description: 'A final project where a simulated robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it.',
    id: 'capstone',
  },
];

export default function CourseModulesSection(): ReactNode {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>
            <FaBook className={styles.titleIcon} />
            Course Curriculum
          </Heading>
          <p className={styles.sectionSubtitle}>
            A complete journey from basics to advanced humanoid robotics
          </p>
        </div>
        <div className={styles.modulesGrid}>
          {CourseModules.map((module, idx) => (
            <Link to={`/docs/category/${module.id}`} key={idx} className={`${styles.moduleCard} ${styles.cardLink}`}>
              <div className={styles.moduleIcon}>
                <module.icon />
              </div>
              <h3 className={styles.moduleTitle}>{module.title}</h3>
              <p className={styles.moduleDescription}>{module.description}</p>
              <div className={styles.moduleNumber}>{idx + 1}</div> {/* Changed to idx + 1 for correct numbering */}
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

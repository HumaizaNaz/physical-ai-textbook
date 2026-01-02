import type {ReactNode} from 'react';
import { FaRobot, FaCubes, FaGamepad } from 'react-icons/fa';

import Feature from './Feature';
import StatsSection from './StatsSection';
import CourseModulesSection from './CourseModulesSection';
import WhyChooseSection from './WhyChooseSection';
import CTASection from './CTASection';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Icon: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Comprehensive Introduction to ROS2',
    Icon: FaRobot,
    description: (
      <>
        Dive deep into the Robot Operating System 2 (ROS2), the industry-standard framework for robotics. Learn the core concepts, from nodes and topics to services and actions, and build a solid foundation for developing complex robot behaviors.
      </>
    ),
  },
  {
    title: 'Build Your Own Digital Twin',
    Icon: FaCubes,
    description: (
      <>
        Create a detailed virtual model of a humanoid robot from scratch. Master the use of URDF and Xacro to define the robot's physical properties, and learn how to assemble a complete digital twin for simulation and testing.
      </>
    ),
  },
  {
    title: 'Simulate and Control',
    Icon: FaGamepad,
    description: (
      <>
        Bring your robot to life in a simulated environment. Use Gazebo to test and refine your robot's movements and interactions. Learn how to develop and implement control strategies in ROS2 to make your humanoid robot walk, grasp, and perform tasks.
      </>
    ),
  },
];

export default function HomepageFeatures(): ReactNode {
  return (
    <>
      {/* Stats Section */}
      <StatsSection />

      {/* Main Features */}
      <section className={styles.features}>
        <div className="container">
          <div className="row">
            {FeatureList.map((props, idx) => (
              <Feature key={idx} {...props} />
            ))}
          </div>
        </div>
      </section>

      {/* Course Modules */}
      <CourseModulesSection />

      {/* Why Choose */}
      <WhyChooseSection />

      {/* Call to Action */}
      <CTASection />
    </>
  );
}
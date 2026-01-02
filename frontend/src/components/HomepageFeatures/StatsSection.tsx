import type {ReactNode} from 'react';
import styles from './styles.module.css';

const Stats = [
  { number: '4+', label: 'Modules' },
  { number: '30+', label: 'Lessons' },
  { number: '100+', label: 'Code Examples' },
  { number: '24/7', label: 'AI Chatbot Support' },
];

export default function StatsSection(): ReactNode {
  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className={styles.statsGrid}>
          {Stats.map((stat, idx) => (
            <div key={idx} className={styles.statCard}>
              <div className={styles.statNumber}>{stat.number}</div>
              <div className={styles.statLabel}>{stat.label}</div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

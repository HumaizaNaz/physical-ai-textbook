import React from 'react';
import styles from './ChatMessage.module.css';
import { JSX } from 'react';

interface Props {
  role: 'user' | 'assistant';
  content: string;
  sources?: string[];
  isStreaming?: boolean;
}

export default function ChatMessage({ role, content, sources, isStreaming }: Props): JSX.Element {
  return (
    <div className={`${styles.message} ${styles[role]}`}>
      <div className={styles.avatar}>{role === 'user' ? '👤' : '🤖'}</div>
      <div className={styles.content}>
        <div className={styles.text}>{content}{isStreaming && <span className={styles.cursor}>▌</span>}</div>
        {sources && sources.length > 0 && (
          <div className={styles.sources}>
            <div className={styles.sourcesLabel}>Sources:</div>
            <div className={styles.sourceChips}>
              {sources.map((src, idx) => <a key={idx} className={styles.sourceChip} href={src}>{src.split('/').pop()}</a>)}
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
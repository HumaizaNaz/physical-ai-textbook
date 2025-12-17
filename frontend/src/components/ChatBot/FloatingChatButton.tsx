import React from 'react';
import styles from './FloatingChatButton.module.css';
import { JSX } from 'react';

interface Props {
  onClick: () => void;
  isOpen: boolean;
}

export default function FloatingChatButton({ onClick, isOpen }: Props): JSX.Element {
  return (
    <button onClick={onClick} className={styles.button}>
      <span className={styles.icon}>{isOpen ? '✕' : '🤖'}</span>
    </button>
  );
}
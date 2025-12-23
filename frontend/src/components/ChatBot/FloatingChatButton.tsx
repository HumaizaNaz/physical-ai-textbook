import React from 'react';
import clsx from 'clsx';
import styles from './FloatingChatButton.module.css';
import { JSX } from 'react';

interface Props {
  onClick: () => void;
  isOpen: boolean;
}

export default function FloatingChatButton({ onClick, isOpen }: Props): JSX.Element {
  return (
    <button onClick={onClick} className={styles.button}>
      <span className={clsx(styles.icon)}>
        {isOpen ? (
          '✕'
        ) : (
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="currentColor"
            className={styles.svgIcon} // Apply CSS module styling to SVG
          >
            <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zM9.5 13H8v-2h1.5v2zm3.25 0h-1.5v-2h1.5v2zm3.25 0h-1.5v-2h1.5v2z" />
          </svg>
        )}
      </span>
    </button>
  );
}